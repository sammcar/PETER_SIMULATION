#!/usr/bin/env python3
"""
cmd_vel_server.py — Servidor TCP que reenvía caracteres de la red neuronal al cliente C++ en la Pi.

Flujo:
  ZMQ /robot_cmd (String)  →  este servidor  →  TCP:8003  →  C++ en Pi  →  serial  →  Arduino

El cliente C++ recibe 1 byte, lo escribe en el puerto serial (+ '\n') y devuelve '1' de confirmación.

Uso:
  python cmd_vel_server.py [--host 0.0.0.0] [--port 8003]
"""

import argparse
import socket
import sys
import os
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'zmq_metrics'))

from transport import Node, String
from topics import TOPIC_ROBOT_CMD

RECV_TIMEOUT = 2.0   # segundos máximos esperando confirmación de la Pi


class CmdVelServer(Node):

    def __init__(self, host: str, port: int):
        super().__init__('cmd_vel_server')
        self._host   = host
        self._port   = port
        self._client = None
        self._lock   = threading.Lock()

        self.create_subscription(String, TOPIC_ROBOT_CMD, self._cb, 10)
        self.get_logger().info(f'[cmd_vel_server] Suscrito a {TOPIC_ROBOT_CMD}')

    def set_client(self, conn: socket.socket):
        with self._lock:
            if self._client is not None:
                try:
                    self._client.close()
                except OSError:
                    pass
            self._client = conn
            conn.settimeout(RECV_TIMEOUT)

    def _cb(self, msg: String):
        if not msg.data:
            return
        cmd = msg.data[0].encode()   # solo el primer carácter

        with self._lock:
            if self._client is None:
                return
            try:
                self._client.sendall(cmd)
                self.get_logger().info(f'[cmd_vel_server] Enviado: {cmd.decode()!r}')

                conf = self._client.recv(1)
                if conf != b'1':
                    self.get_logger().warn(f'[cmd_vel_server] Confirmación inesperada: {conf!r}')

            except socket.timeout:
                self.get_logger().warn('[cmd_vel_server] Timeout esperando confirmación de la Pi')

            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                self.get_logger().error(f'[cmd_vel_server] Cliente desconectado: {e}')
                try:
                    self._client.close()
                except OSError:
                    pass
                self._client = None


def main():
    parser = argparse.ArgumentParser(description='ZMQ /robot_cmd → TCP → Arduino bridge')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8003)
    args = parser.parse_args()

    node = CmdVelServer(args.host, args.port)

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((args.host, args.port))
    server_sock.listen(1)

    print(f'[cmd_vel_server] Esperando cliente C++ en {args.host}:{args.port}...')

    spin_thread = threading.Thread(target=node.spin, daemon=True)
    spin_thread.start()

    try:
        while True:
            conn, addr = server_sock.accept()
            print(f'[cmd_vel_server] Cliente conectado: {addr}')
            node.set_client(conn)
    except KeyboardInterrupt:
        print('\n[cmd_vel_server] Detenido.')
    finally:
        server_sock.close()
        node.destroy_node()


if __name__ == '__main__':
    main()

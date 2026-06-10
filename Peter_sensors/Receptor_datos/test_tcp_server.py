#!/usr/bin/env python3
"""
test_tcp_server.py — Servidor TCP simple para probar el cliente C++ directamente.

Espera la conexión del cliente C++, luego permite enviar caracteres manualmente.

Uso:
  python test_tcp_server.py [--port 8003]
"""

import argparse
import socket

PORT = 8003

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=PORT)
    args = parser.parse_args()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', args.port))
    server.listen(1)

    print(f'Esperando cliente C++ en puerto {args.port}...')
    conn, addr = server.accept()
    print(f'Cliente conectado: {addr}')
    print('Escribe un carácter y presiona Enter (Ctrl+C para salir):\n')

    try:
        while True:
            entrada = input('> ').strip()
            if not entrada:
                continue

            cmd = entrada[0].encode()
            conn.sendall(cmd)
            print(f'Enviado: {cmd.decode()}')

            conf = conn.recv(1)
            print(f'Confirmación recibida: {conf.decode()}\n')

    except KeyboardInterrupt:
        print('\nDetenido.')
    except (BrokenPipeError, ConnectionResetError):
        print('Cliente desconectado.')
    finally:
        conn.close()
        server.close()

if __name__ == '__main__':
    main()

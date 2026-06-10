#!/usr/bin/env python3
"""
test_cmd_vel.py — Publica caracteres en /robot_cmd para probar cmd_vel_server.py.

Uso:
  python test_cmd_vel.py            # modo interactivo (escribe un carácter + Enter)
  python test_cmd_vel.py --auto     # envía caracteres automáticamente cada segundo
"""

import argparse
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'zmq_metrics'))

from transport import Node, String
from topics import TOPIC_ROBOT_CMD


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--auto', action='store_true',
                        help='Enviar caracteres automáticamente')
    args = parser.parse_args()

    node = Node('test_cmd_vel')
    pub  = node.create_publisher(String, TOPIC_ROBOT_CMD, 10)

    time.sleep(0.3)  # dar tiempo al broker para registrar la suscripción
    print(f'[test] Publicando en {TOPIC_ROBOT_CMD}')

    def enviar(c: str):
        msg      = String()
        msg.data = c
        pub.publish(msg)
        print(f'[test] Enviado: {c!r}')

    if args.auto:
        secuencia = ['F', 'F', 'L', 'R', 'B', 'S']
        try:
            while True:
                for c in secuencia:
                    enviar(c)
                    time.sleep(1.0)
        except KeyboardInterrupt:
            pass
    else:
        print('Escribe un carácter y presiona Enter (Ctrl+C para salir):')
        try:
            while True:
                entrada = input('> ').strip()
                if entrada:
                    enviar(entrada[0])
        except KeyboardInterrupt:
            pass

    print('\n[test] Listo.')


if __name__ == '__main__':
    main()

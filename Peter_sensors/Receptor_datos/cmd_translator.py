#!/usr/bin/env python3
"""
cmd_translator.py — Traduce /cmd_vel + /peter_mode a caracteres del ESP.

Suscripciones ZMQ:
  /cmd_vel      (Twist)  — velocidad de la red neuronal
  /peter_mode   (String) — modo de locomoción: C / H / X

Publicación ZMQ:
  /robot_cmd    (String) — carácter único para el ESP (mismo protocolo que el teleop)

Mapeo de velocidad → carácter:
  linear.x  >  umbral  → 'i'  (adelante)
  linear.x  < -umbral  → ','  (atrás)
  linear.y  >  umbral  → 'j'  (lateral izquierda)
  linear.y  < -umbral  → 'l'  (lateral derecha)
  angular.z >  umbral  → 'u'  (giro izquierda)
  angular.z < -umbral  → 'o'  (giro derecha)
  todos < umbral        → 'k'  (stop)

Mapeo de modo → carácter:
  'H' → 'z'  |  'X' → 'x'  |  'C' → 'c'

Comportamiento de envío:
  - Se publica siempre que el carácter cambia.
  - Si el carácter no cambia pero pasaron más de --keepalive segundos
    desde el último envío, se reenvía igual (evita timeout del ESP).
  - Los cambios de modo se publican de forma independiente al movimiento.

Uso:
  python cmd_translator.py [--threshold 0.1] [--keepalive 0.4]
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'zmq_metrics'))

from transport import Node, spin, Twist, String
from topics import TOPIC_CMD_VEL, TOPIC_PETER_MODE, TOPIC_ROBOT_CMD

MODE_MAP = {'H': 'z', 'X': 'x', 'C': 'c'}


class CmdTranslator(Node):

    def __init__(self, threshold: float = 0.1, keepalive: float = 0.4):
        super().__init__('cmd_translator')
        self._threshold      = threshold
        self._keepalive      = keepalive
        self._last_move      = None    # último carácter de movimiento enviado
        self._last_mode      = None    # último modo enviado
        self._last_send_time = 0.0    # timestamp del último envío de movimiento

        self._pub = self.create_publisher(String, TOPIC_ROBOT_CMD, 10)

        self.create_subscription(Twist,  TOPIC_CMD_VEL,    self._cmd_vel_cb, 10)
        self.create_subscription(String, TOPIC_PETER_MODE, self._mode_cb,    10)

        print(
            f'[cmd_translator] umbral={threshold}  keepalive={keepalive}s | '
            f'{TOPIC_CMD_VEL} + {TOPIC_PETER_MODE} → {TOPIC_ROBOT_CMD}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        char = self._twist_to_char(msg)
        now  = time.monotonic()

        changed   = char != self._last_move
        timed_out = (now - self._last_send_time) >= self._keepalive

        if changed or timed_out:
            self._last_move      = char
            self._last_send_time = now
            self._publish(char)

    def _mode_cb(self, msg: String):
        mode = msg.data
        if mode == self._last_mode:
            return
        self._last_mode = mode
        char = MODE_MAP.get(mode)
        if char:
            self._publish(char)

    # ── Traducción ────────────────────────────────────────────────────────────

    def _twist_to_char(self, twist: Twist) -> str:
        t = self._threshold

        lin = twist.linear.x  if abs(twist.linear.x)  >= t else 0.0
        lat = twist.linear.y  if abs(twist.linear.y)  >= t else 0.0
        ang = twist.angular.z if abs(twist.angular.z) >= t else 0.0

        if lin == 0.0 and lat == 0.0 and ang == 0.0:
            return 'k'

        # El eje con mayor magnitud decide el carácter
        dominant = max(
            (abs(ang), 'u' if ang > 0 else 'o'),
            (abs(lat), 'j' if lat > 0 else 'l'),
            (abs(lin), 'i' if lin > 0 else ','),
        )
        return dominant[1]

    # ── Publicación ───────────────────────────────────────────────────────────

    def _publish(self, char: str):
        msg      = String()
        msg.data = char
        self._pub.publish(msg)
        print(f'[cmd_translator] → {char!r}')


def main():
    parser = argparse.ArgumentParser(description='Cmd translator — Twist/mode → ESP char')
    parser.add_argument('--threshold', type=float, default=0.1,
                        help='Velocidad mínima para no contar como cero (default 0.1)')
    parser.add_argument('--keepalive', type=float, default=0.4,
                        help='Reenviar último comando cada N segundos aunque no cambie (default 0.4)')
    args = parser.parse_args()

    node = CmdTranslator(threshold=args.threshold, keepalive=args.keepalive)
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()

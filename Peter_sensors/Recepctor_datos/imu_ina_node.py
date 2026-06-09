#!/usr/bin/env python3
"""
imu_ina_node.py — Recepción UDP del IMU + INA226 y publicación ZMQ.

Formato del paquete UDP (48 bytes):
  timestamp_ns  uint64   nanosegundos (epoch del microcontrolador)
  roll          float32  grados
  pitch         float32  grados
  ax ay az      float32  aceleración lineal  m/s²
  gx gy gz      float32  velocidad angular   rad/s
  current       float32  corriente           A
  voltage       float32  tensión             V

Publicaciones ZMQ:
  /imu/data  (Imu)              → red neuronal + stability_monitor
  /ina/data  (Float32MultiArray) → [current_A, voltage_V, power_W]

La red neuronal espera cuaterniones, no roll/pitch directos.
Este nodo convierte roll/pitch → cuaternión (yaw = 0) antes de publicar.

Registro opcional:
  --csv   guarda imu_ina_<timestamp>.csv con todas las lecturas crudas

Uso:
  python imu_ina_node.py [--host 0.0.0.0] [--port 9999] [--csv] [--show]
"""

import argparse
import csv
import math
import os
import socket
import struct
import sys
import time
from datetime import datetime

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'zmq_metrics'))

from transport import Node, Imu, Float32MultiArray, _Vec3, _Orientation
from topics import TOPIC_IMU, TOPIC_INA

# ── Formato UDP ───────────────────────────────────────────────────────────────
FORMAT = "=Qffffffffff"          # timestamp + 10 floats
SIZE   = struct.calcsize(FORMAT) # 48 bytes

CAMPOS_CSV = [
    'time_s',
    'roll_deg', 'pitch_deg',
    'ax', 'ay', 'az',
    'gx', 'gy', 'gz',
    'current_A', 'voltage_V', 'power_W',
]


# ── Conversión roll/pitch → cuaternión ────────────────────────────────────────

def euler_a_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float = 0.0):
    """
    Convierte ángulos de Euler (grados, convención ZYX) a cuaternión (qx, qy, qz, qw).

    La red neuronal reconstruye roll/pitch desde el cuaternión con:
        roll  = 180 - |degrees(atan2(2(qw·qx + qy·qz), 1 - 2(qx²+qy²)))|
        pitch = |degrees(arcsin(2(qw·qy - qz·qx)))|
    Usando yaw=0 se preservan los valores de roll y pitch sin distorsión de yaw.
    """
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


# ── Nodo ZMQ ──────────────────────────────────────────────────────────────────

class ImuInaNode(Node):

    def __init__(self, host: str, port: int, save_csv: bool, show: bool):
        super().__init__('imu_ina_node')
        self._host     = host
        self._port     = port
        self._show     = show

        self._pub_imu = self.create_publisher(Imu,              TOPIC_IMU, 10)
        self._pub_ina = self.create_publisher(Float32MultiArray, TOPIC_INA, 10)

        # ── CSV opcional ───────────────────────────────────────────────────
        self._csv_writer = None
        self._csv_file   = None
        if save_csv:
            out_dir  = os.path.expanduser('~/peter_experiments')
            os.makedirs(out_dir, exist_ok=True)
            ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
            csv_path = os.path.join(out_dir, f'imu_ina_{ts}.csv')
            self._csv_file   = open(csv_path, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(CAMPOS_CSV)
            self.get_logger().info(f'[imu_ina] CSV: {csv_path}')

        self.get_logger().info(
            f'[imu_ina] UDP {host}:{port} → {TOPIC_IMU}  {TOPIC_INA}'
        )

    # ── Parseo ─────────────────────────────────────────────────────────────

    def _parsear(self, data: bytes) -> dict | None:
        if len(data) < SIZE:
            self.get_logger().warn(
                f'[imu_ina] paquete incompleto: {len(data)}/{SIZE} bytes'
            )
            return None
        ts, roll, pitch, ax, ay, az, gx, gy, gz, current, voltage = \
            struct.unpack(FORMAT, data[:SIZE])
        return dict(
            ts=ts, roll=roll, pitch=pitch,
            ax=ax, ay=ay, az=az,
            gx=gx, gy=gy, gz=gz,
            current=current, voltage=voltage,
        )

    # ── Publicación ────────────────────────────────────────────────────────

    def _publicar(self, d: dict, t: float):
        # ── IMU ────────────────────────────────────────────────────────────
        qx, qy, qz, qw = euler_a_quaternion(d['roll'], d['pitch'])

        imu_msg                        = Imu()
        imu_msg.orientation            = _Orientation(qx, qy, qz, qw)
        imu_msg.linear_acceleration    = _Vec3(d['ax'], d['ay'], d['az'])
        imu_msg.angular_velocity       = _Vec3(d['gx'], d['gy'], d['gz'])
        self._pub_imu.publish(imu_msg)

        # ── INA ────────────────────────────────────────────────────────────
        power    = d['current'] * d['voltage']
        ina_msg  = Float32MultiArray([d['current'], d['voltage'], power])
        self._pub_ina.publish(ina_msg)

        # ── CSV ────────────────────────────────────────────────────────────
        if self._csv_writer:
            self._csv_writer.writerow([
                round(t, 4),
                round(d['roll'],    3), round(d['pitch'],   3),
                round(d['ax'],      4), round(d['ay'],      4), round(d['az'],      4),
                round(d['gx'],      4), round(d['gy'],      4), round(d['gz'],      4),
                round(d['current'], 4), round(d['voltage'], 4), round(power,        4),
            ])
            self._csv_file.flush()

        # ── Log ────────────────────────────────────────────────────────────
        if self._show:
            self.get_logger().info(
                f'roll={d["roll"]:+6.1f}°  pitch={d["pitch"]:+6.1f}°  '
                f'acc=({d["ax"]:+5.2f},{d["ay"]:+5.2f},{d["az"]:+5.2f})  '
                f'I={d["current"]:.3f}A  V={d["voltage"]:.2f}V  P={power:.2f}W'
            )

    # ── Loop principal ─────────────────────────────────────────────────────

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self._host, self._port))
        sock.settimeout(2.0)

        self.get_logger().info(
            f'[imu_ina] escuchando en {self._host}:{self._port}...'
        )

        t0 = time.monotonic()
        n  = 0
        try:
            while True:
                try:
                    data, _ = sock.recvfrom(SIZE + 32)
                except socket.timeout:
                    self.get_logger().warn('[imu_ina] timeout esperando paquete')
                    continue

                d = self._parsear(data)
                if d is None:
                    continue

                t = time.monotonic() - t0
                self._publicar(d, t)
                n += 1

        except KeyboardInterrupt:
            pass
        finally:
            elapsed = time.monotonic() - t0
            hz      = n / elapsed if elapsed > 0 else 0
            self.get_logger().info(
                f'[imu_ina] detenido — {n} lecturas en {elapsed:.1f}s ({hz:.1f} Hz)'
            )
            if self._csv_file:
                self._csv_file.close()
            sock.close()
            self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description='IMU + INA226 Node — UDP → ZMQ')
    parser.add_argument('--host', default='0.0.0.0',
                        help='IP de escucha UDP (default 0.0.0.0)')
    parser.add_argument('--port', type=int, default=9999,
                        help='Puerto UDP (default 9999)')
    parser.add_argument('--csv',  action='store_true',
                        help='Guardar CSV con todas las lecturas')
    parser.add_argument('--show', action='store_true',
                        help='Imprimir lecturas en consola')
    args = parser.parse_args()

    node = ImuInaNode(args.host, args.port, args.csv, args.show)
    node.run()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
lidar_node.py — Recepción UDP del lidar físico + publicación ZMQ como LaserScan.

Comportamiento del grid:
  - Cada celda (1° de resolución) guarda la distancia mínima vista en esa dirección
    y el momento en que se recibió el último dato para ese ángulo.
  - Antes de cada publicación se barren las celdas: si la última lectura tiene más
    de `--expiry` segundos (default 1.0 s), la celda vuelve a inf.
  - Así el grid refleja solo lo que el sensor ve "ahora"; los sectores que dejan
    de reportar se limpian automáticamente sin reiniciar el array completo.

Uso:
  python lidar_node.py [--host 0.0.0.0] [--port 8888]
                       [--rate 10] [--expiry 1.0] [--show]

  --rate    Hz de publicación ZMQ (default 10)
  --expiry  segundos antes de expirar una celda sin renovación (default 1.0)
  --show    imprime stats de cada scan publicado
"""

import argparse
import math
import os
import socket
import struct
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'zmq_metrics'))

from transport import Node, LaserScan
from topics import TOPIC_SCAN

# ── Formato del paquete UDP ───────────────────────────────────────────────────
SIZE_DATOS       = 500
LIDAR_FMT        = "=Qdii"                          # timestamp, angle, dist, intensity
LIDAR_POINT_SIZE = struct.calcsize(LIDAR_FMT)       # 24 bytes
PACKET_SIZE      = LIDAR_POINT_SIZE * SIZE_DATOS    # 12 000 bytes

# Filtro de distancia básico — la red neuronal aplica su propio r_min/r_max
DIST_MIN_M = 0.05
DIST_MAX_M = 12.0

GRID_N   = 360          # una celda por grado
GRID_RAD = math.radians(1.0)


def parsear_paquete(data: bytes) -> list:
    """
    Desempaqueta bytes UDP en lista de puntos.
    Usa len(data) para calcular cuántos puntos completos llegaron,
    tolerando paquetes incompletos.
    """
    n      = len(data) // LIDAR_POINT_SIZE
    puntos = []
    for i in range(n):
        offset = i * LIDAR_POINT_SIZE
        ts, angle, dist_mm, intensity = struct.unpack(
            LIDAR_FMT, data[offset : offset + LIDAR_POINT_SIZE]
        )
        puntos.append({
            "angle":    angle,
            "distance": dist_mm,
            "intensity": intensity,
        })
    return puntos


# ── Grid con expiración por celda ─────────────────────────────────────────────

class ScanGrid:
    """
    Grid de 360 celdas donde cada celda expira individualmente.

    Hilo de recepción  →  update(puntos)
    Hilo de publicación →  snapshot(expiry_s)  →  LaserScan
    """

    def __init__(self):
        self._lock      = threading.Lock()
        # distancia mínima vista en cada dirección
        self._dist      = np.full(GRID_N, np.inf,  dtype=np.float64)
        self._intensity = np.zeros(GRID_N,          dtype=np.float64)
        # cuándo se recibió el último dato válido por celda
        # -inf → nunca recibido (expirado desde el inicio)
        self._last_ts   = np.full(GRID_N, -np.inf, dtype=np.float64)

    def update(self, puntos: list):
        """
        Incorpora nuevos puntos al grid.
        - Siempre actualiza el timestamp de la celda.
        - Guarda la distancia mínima (obstáculo más cercano en esa dirección).
        Thread-safe.
        """
        now = time.monotonic()
        with self._lock:
            for p in puntos:
                dist_m = p["distance"] * 1e-3
                if not (DIST_MIN_M <= dist_m <= DIST_MAX_M):
                    continue

                idx = int(p["angle"] % 360.0) % GRID_N

                # timestamp siempre al más reciente
                self._last_ts[idx] = now

                # distancia: guardar la mínima (obstáculo más cercano)
                if dist_m < self._dist[idx]:
                    self._dist[idx]      = dist_m
                    self._intensity[idx] = float(p["intensity"])

    def snapshot(self, expiry_s: float) -> tuple:
        """
        1. Expira celdas cuya última lectura sea más antigua que expiry_s.
        2. Devuelve copia del estado actual (dist, intensity).
        Thread-safe.
        """
        now = time.monotonic()
        with self._lock:
            expiradas = (now - self._last_ts) > expiry_s
            self._dist[expiradas]      = np.inf
            self._intensity[expiradas] = 0.0
            self._last_ts[expiradas]   = -np.inf

            return self._dist.copy(), self._intensity.copy()

    @property
    def celdas_vivas(self) -> int:
        """Número de celdas con dato válido (no inf). Para debug."""
        with self._lock:
            return int(np.isfinite(self._dist).sum())


# ── Nodo ZMQ ──────────────────────────────────────────────────────────────────

class LidarNode(Node):

    def __init__(self, host: str, port: int, rate_hz: float,
                 expiry_s: float, show: bool):
        super().__init__('lidar_node')
        self._host      = host
        self._port      = port
        self._period_s  = 1.0 / rate_hz
        self._expiry_s  = expiry_s
        self._show      = show
        self._grid      = ScanGrid()
        self._pub       = self.create_publisher(LaserScan, TOPIC_SCAN, 10)
        self._running   = False
        self._scan_n    = 0

        self.get_logger().info(
            f'[lidar_node] {host}:{port} UDP | '
            f'{rate_hz} Hz | expiry {expiry_s} s → {TOPIC_SCAN}'
        )

    # ── Hilo recepción ─────────────────────────────────────────────────────

    def _recv_loop(self, sock: socket.socket):
        paquetes = 0
        while self._running:
            try:
                data, _ = sock.recvfrom(PACKET_SIZE + 512)
            except socket.timeout:
                continue
            except OSError:
                break

            puntos = parsear_paquete(data)
            if puntos:
                self._grid.update(puntos)
                paquetes += 1
                if self._show and paquetes % 100 == 0:
                    self.get_logger().info(
                        f'[lidar_node] paquetes={paquetes} | '
                        f'celdas vivas={self._grid.celdas_vivas}/{GRID_N}'
                    )

    # ── Hilo publicación ───────────────────────────────────────────────────

    def _pub_loop(self):
        while self._running:
            time.sleep(self._period_s)
            if not self._running:
                break

            dist, intensity = self._grid.snapshot(self._expiry_s)

            scan                 = LaserScan()
            scan.angle_min       = 0.0
            scan.angle_max       = 2.0 * math.pi
            scan.angle_increment = GRID_RAD
            scan.ranges          = dist.tolist()
            scan.intensities     = intensity.tolist()

            self._pub.publish(scan)
            self._scan_n += 1

            if self._show:
                validas    = np.isfinite(dist)
                n_validas  = int(validas.sum())
                min_dist   = float(dist[validas].min()) if n_validas else float('inf')
                self.get_logger().info(
                    f'[lidar_node] scan #{self._scan_n} | '
                    f'{n_validas}/{GRID_N} celdas | '
                    f'min={min_dist:.3f} m'
                )

    # ── Arranque ───────────────────────────────────────────────────────────

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self._host, self._port))
        sock.settimeout(1.0)

        self._running = True

        t_recv = threading.Thread(target=self._recv_loop, args=(sock,), daemon=True)
        t_pub  = threading.Thread(target=self._pub_loop,               daemon=True)
        t_recv.start()
        t_pub.start()

        self.get_logger().info(
            f'[lidar_node] escuchando en {self._host}:{self._port}...'
        )

        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            self._running = False
            sock.close()
            t_recv.join(timeout=2.0)
            t_pub.join(timeout=2.0)
            self.get_logger().info(
                f'[lidar_node] detenido — scans publicados: {self._scan_n}'
            )
            self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description='Lidar Node — UDP → ZMQ /scan')
    parser.add_argument('--host',   default='0.0.0.0',
                        help='IP de escucha UDP (default 0.0.0.0)')
    parser.add_argument('--port',   type=int,   default=8888,
                        help='Puerto UDP (default 8888)')
    parser.add_argument('--rate',   type=float, default=10.0,
                        help='Hz de publicación ZMQ (default 10)')
    parser.add_argument('--expiry', type=float, default=1.0,
                        help='Segundos para expirar celda sin renovación (default 1.0)')
    parser.add_argument('--show',   action='store_true',
                        help='Imprimir stats por scan')
    args = parser.parse_args()

    node = LidarNode(args.host, args.port, args.rate, args.expiry, args.show)
    node.run()


if __name__ == '__main__':
    main()

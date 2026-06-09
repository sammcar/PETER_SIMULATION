#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, mmap, struct, time, math, threading
import numpy as np
import matplotlib.pyplot as plt

# ================== CONFIGURACIÓN ==================
SHM_NAME_LIDAR      = "/dev/shm/shared_lidar"
MAX_LIDAR_POINTS    = 500
LIDAR_STRUCT_FORMAT = "Qdii"        # (uint64_t ts, double angle_deg, int distance_mm, int intensity)
LIDAR_STRUCT_SIZE   = struct.calcsize(LIDAR_STRUCT_FORMAT)

# Filtros (sin CLI)
DIST_MIN_MM   = 1
DIST_MAX_MM   = 1000           # 4 m
ANGLE_MIN_DEG = 0              # muestra todo (0..359)
ANGLE_MAX_DEG = 359

# Frecuencias
REFRESH_SEC   = 0.5            # refresco del plot
READ_SLEEP_S  = 0.01           # pausa corta del hilo lector (evita 100% CPU)

# Estética
POINT_SIZE = 12
SHOW_GRID_RINGS_M = [0.5, 1.0, 2.0, 3.0, 4.0]
# ===================================================


class SharedLidarReader(threading.Thread):
    """
    Hilo que lee la SHM y deja el último snapshot en self.snapshot
    """
    def __init__(self, path, stop_event, lock, angle_min=ANGLE_MIN_DEG, angle_max=ANGLE_MAX_DEG,
                 dmin_mm=DIST_MIN_MM, dmax_mm=DIST_MAX_MM):
        super().__init__(daemon=True)
        self.path = path
        self.stop_event = stop_event
        self.lock = lock
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.dmin_mm = dmin_mm
        self.dmax_mm = dmax_mm

        self._file = None
        self._shm = None

        # snapshot compartido: (angles_deg, dists_m, t_monotonic)
        self.snapshot = (np.empty(0, np.float32), np.empty(0, np.float32), 0.0)

    def open_shared_memory(self):
        if not os.path.exists(self.path):
            raise FileNotFoundError(
                f"❌ Memoria compartida no encontrada en {self.path}. "
                "Asegúrate de que el servidor esté en ejecución."
            )
        self._file = open(self.path, "r+b")
        self._shm = mmap.mmap(self._file.fileno(), 0, mmap.MAP_SHARED, mmap.PROT_READ)

    def close_shared_memory(self):
        try:
            if self._shm is not None:
                self._shm.close()
            if self._file is not None:
                self._file.close()
        except Exception:
            pass
        finally:
            self._shm = None
            self._file = None

    def read_once(self):
        angles = []
        dists_m = []

        shm = self._shm
        if shm is None:
            return

        for i in range(MAX_LIDAR_POINTS):
            off = i * LIDAR_STRUCT_SIZE
            data = shm[off: off + LIDAR_STRUCT_SIZE]
            if len(data) < LIDAR_STRUCT_SIZE:
                break
            ts, angle_deg, distance_mm, intensity = struct.unpack(LIDAR_STRUCT_FORMAT, data)

            angle_deg = float(angle_deg) % 360.0
            if not (self.angle_min <= angle_deg <= self.angle_max):
                continue
            if distance_mm is None or distance_mm <= 0:
                continue
            if distance_mm < self.dmin_mm or distance_mm > self.dmax_mm:
                continue

            angles.append(angle_deg)
            dists_m.append(distance_mm / 1000.0)

        a = np.asarray(angles, dtype=np.float32) if angles else np.empty(0, np.float32)
        d = np.asarray(dists_m, dtype=np.float32) if dists_m else np.empty(0, np.float32)

        with self.lock:
            self.snapshot = (a, d, time.monotonic())

    def run(self):
        try:
            self.open_shared_memory()
            while not self.stop_event.is_set():
                self.read_once()
                time.sleep(READ_SLEEP_S)
        except FileNotFoundError as e:
            print(e)
        except Exception as e:
            print(f"⚠️ Error en hilo lector: {e}")
        finally:
            self.close_shared_memory()


def polar_to_cartesian(angles_deg: np.ndarray, dists_m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    th = np.deg2rad(angles_deg)
    x = dists_m * np.cos(th)
    y = dists_m * np.sin(th)
    return -x, y  # ← invierte izquierda/derecha, mantiene frente/atrás

def setup_plot(max_range_m: float):
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_title("LiDAR (vista superior) — puntos en metros")
    ax.set_aspect("equal", adjustable="box")

    lim = max_range_m * 1.05
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)

    ax.axhline(0, lw=0.8, alpha=0.6)
    ax.axvline(0, lw=0.8, alpha=0.6)

    for r in SHOW_GRID_RINGS_M:
        circ = plt.Circle((0, 0), r, fill=False, lw=0.5, alpha=0.4)
        ax.add_patch(circ)
        ax.text(r, 0, f"{r:g} m", fontsize=8, ha="left", va="bottom", alpha=0.7)

    ax.plot(0, 0, marker="o", markersize=6, alpha=0.9)  # sensor

    sc = ax.scatter([], [], s=POINT_SIZE)
    info = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", ha="left", fontsize=9)
    return fig, ax, sc, info


def main():
    stop_event = threading.Event()
    lock = threading.Lock()

    # Lanza hilo lector
    reader = SharedLidarReader(SHM_NAME_LIDAR, stop_event, lock)
    reader.start()

    # Plot en hilo principal (requisito de muchos backends GUI)
    max_range_m = DIST_MAX_MM / 1000.0
    fig, ax, sc, info = setup_plot(max_range_m)
    plt.ion()
    plt.show(block=False)

    try:
        last_draw = 0.0
        while True:
            now = time.monotonic()
            if now - last_draw >= REFRESH_SEC:
                last_draw = now
                with lock:
                    angles_deg, dists_m, t_snap = reader.snapshot

                if angles_deg.size == 0:
                    sc.set_offsets(np.empty((0, 2)))
                    info.set_text("0 puntos válidos")
                else:
                    x, y = polar_to_cartesian(angles_deg, dists_m)
                    sc.set_offsets(np.column_stack((x, y)))
                    i_min = int(np.argmin(dists_m))
                    d_min = float(dists_m[i_min])
                    info.set_text(f"{len(dists_m)} puntos | más cercano: {d_min:.3f} m")

                fig.canvas.draw_idle()
                plt.pause(0.001)

            # duerme breve para eventos GUI sin bloquear
            plt.pause(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        reader.join(timeout=1.0)
        plt.ioff()
        plt.close(fig)


if __name__ == "__main__":
    main()

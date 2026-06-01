"""
peter_simu_v6_motores.py — Simulación + envío serial de ángulos articulares
════════════════════════════════════════════════════════════════════════════
Extiende peter_simu_IMU_v5 añadiendo un hilo de fondo que envía por serial
los ángulos de las 12 articulaciones a una frecuencia fija (SEND_HZ).

Protocolo serial — mismo formato que joint_mapper.py:
    Una línea por articulación:  leg,joint,angle\\n
    · leg   : 0=FL  1=FR  2=RL  3=RR
    · joint : 0=Coxa  1=Fémur  2=Tibia
    · angle : grados con 1 decimal  (ej. +11.2  ó  -95.5)

Por cada ciclo se envían 12 comandos consecutivos (una línea por joint).
Ejemplo en reposo (FL):
    0,0,0.0
    0,1,11.2
    0,2,-95.5
    ...

Ajusta SERIAL_PORT y BAUDRATE según tu configuración antes de correr.
Si el puerto no está disponible la simulación corre sin error — ideal para
verificar en pantalla sin tener el robot conectado.

Ejecutar desde ver_def/:
    python peter_simu_v6_motores.py
O desde Python_simu/:
    python -m ver_def.peter_simu_v6_motores
"""

import sys
import os
import time
import threading

_here   = os.path.dirname(os.path.abspath(__file__))   # …/ver_def
_parent = os.path.dirname(_here)                        # …/Python_simu
for _p in (_parent, _here):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np

from robot_params     import RobotParams
from robot_kinematics import SpiderQuadruped
from robot_gait       import CrawlGait
from robot_viz        import animate_crawl


# ── Configuración serial ───────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyACM0'   # ← ajusta al puerto de tu ESP32
BAUDRATE    = 115200
SEND_HZ     = 10.0             # paquetes por segundo enviados al robot

LEG_ORDER = ['FL', 'FR', 'RL', 'RR']   # índice 0-3 → igual que joint_mapper


# ──────────────────────────────────────────────────────────────────────
#  CLASE SerialSender
# ──────────────────────────────────────────────────────────────────────

class SerialSender:
    """
    Hilo de fondo que envía los ángulos de las 12 articulaciones por serial
    a frecuencia fija (send_hz). Siempre envía — no requiere que cambien.

    Si pyserial no está instalado o el puerto no existe, funciona en modo
    "dry-run": imprime los paquetes en consola con el mismo formato.
    """

    def __init__(self, robot: SpiderQuadruped,
                 port: str     = SERIAL_PORT,
                 baudrate: int = BAUDRATE,
                 send_hz: float = SEND_HZ):

        self.robot         = robot
        self._running      = threading.Event()
        self._ser          = None
        self._dry_run      = False
        self._packets_sent = 0

        try:
            import serial
            self._ser = serial.Serial(port, baudrate, timeout=0.1)
            print(f'[serial] ✓ Conectado  →  {port} @ {baudrate} baud')
        except ImportError:
            print('[serial] ✗ pyserial no instalado  →  modo dry-run (solo consola)')
            print('[serial]   Instalar con: pip install pyserial')
            self._dry_run = True
        except Exception as e:
            print(f'[serial] ✗ No se pudo abrir {port}: {e}')
            print('[serial]   Modo dry-run activo  →  paquetes impresos en consola.')
            self._dry_run = True

        self._running.set()
        self._thread = threading.Thread(
            target=self._worker, args=(send_hz,), daemon=True, name='SerialSender')
        self._thread.start()

    # ── Hilo de fondo ─────────────────────────────────────────────────

    def _worker(self, send_hz: float):
        interval = 1.0 / send_hz
        while self._running.is_set():
            self._send()
            time.sleep(interval)

    def _send(self):
        """Envía 12 comandos individuales: 'leg,joint,angle\\n' por articulación."""
        self._packets_sent += 1
        show_dry = (self._packets_sent % int(SEND_HZ * 3) == 1)   # log cada ~3 s

        for leg_idx, leg_name in enumerate(LEG_ORDER):
            angles_deg = np.degrees(self.robot.legs[leg_name].q)
            for joint_idx, angle in enumerate(angles_deg):
                cmd = f'{leg_idx},{joint_idx},{angle:.1f}\n'
                if self._dry_run:
                    if show_dry:
                        print(f'[dry-run #{self._packets_sent}] {cmd}', end='')
                else:
                    try:
                        self._ser.write(cmd.encode())
                    except Exception as e:
                        print(f'[serial] Error de escritura: {e}')
                        return

    # ── API pública ────────────────────────────────────────────────────

    @property
    def packets_sent(self) -> int:
        return self._packets_sent

    def stop(self):
        """Detiene el hilo y cierra el puerto serial."""
        self._running.clear()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        if self._ser and self._ser.is_open:
            self._ser.close()
            print(f'[serial] Puerto cerrado. Paquetes enviados: {self._packets_sent}')
        elif self._dry_run:
            print(f'[serial] Fin dry-run. Paquetes que se habrían enviado: {self._packets_sent}')


# ──────────────────────────────────────────────────────────────────────
#  PUNTO DE ENTRADA
# ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':

    params = RobotParams(
        body_length = 0.145,
        body_width  = 0.105,
        L_coxa      = 0.042,
        L_femur     = 0.068,
        L_tibia     = 0.123,
        rest_x      = 0.11,
        rest_z      = -0.10,
    )

    robot  = SpiderQuadruped(params)
    gait   = CrawlGait(robot,
                       step_length   = 0.06,
                       step_height   = 0.03,
                       step_duration = 0.50)

    sender = SerialSender(robot,
                          port     = SERIAL_PORT,
                          baudrate = BAUDRATE,
                          send_hz  = SEND_HZ)
    try:
        anim = animate_crawl(robot, gait, autostart=False)
    finally:
        sender.stop()

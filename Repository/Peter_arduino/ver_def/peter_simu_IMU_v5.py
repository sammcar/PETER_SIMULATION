"""
peter_simu_IMU_v5.py — Punto de entrada principal
══════════════════════════════════════════════════
Ensambla los módulos portables (robot_*) con la visualización.

Estructura de archivos:
    robot_math.py        ← utilidades matemáticas       [PORTABLE C++]
    robot_params.py      ← parámetros físicos del robot  [PORTABLE C++]
    robot_kinematics.py  ← IK / FK por pata              [PORTABLE C++]
    robot_gait.py        ← controlador de marcha crawl   [PORTABLE C++]
    robot_imu.py         ← compensación IMU              [PORTABLE C++]
    robot_viz.py         ← visualización matplotlib      [solo Python]
    peter_simu_IMU_v5.py ← este archivo (main)           [solo Python]

Para pasar al ESP32, los módulos marcados PORTABLE C++ se traducen
directamente a C++ usando <math.h>. Ver encabezados de cada módulo.
"""

import sys, os
_here = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_here)
for _p in (_parent, _here):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from robot_params     import RobotParams
from robot_kinematics import SpiderQuadruped
from robot_gait       import CrawlGait
from robot_viz        import animate_crawl

if __name__ == "__main__":

    params = RobotParams(
        body_length = 0.145,
        body_width  = 0.105,
        L_coxa      = 0.042,
        L_femur     = 0.068,
        L_tibia     = 0.123,
        rest_x      = 0.11,
        rest_z      = -0.10,
    )

    robot = SpiderQuadruped(params)
    gait  = CrawlGait(robot,
                      step_length   = 0.06,   # m  — ajustable con slider
                      step_height   = 0.03,   # m  — ajustable con slider
                      step_duration = 0.50)   # s  — ajustable con slider

    # ── Animación principal ──────────────────────────────────────────
    # n_cycles=None → corre indefinidamente (cerrar ventana para salir)
    # n_cycles=2    → para tras 2 ciclos completos (8 pasos)
    anim = animate_crawl(robot, gait)

    # ── Otras vistas (descomenta la que quieras) ─────────────────────
    # robot.rest_pose()
    # visualize(robot, "Postura de Reposo")
    # visualize_interactive(robot)

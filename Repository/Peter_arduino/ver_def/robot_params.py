"""
robot_params.py — Parámetros físicos del robot cuadrúpedo
══════════════════════════════════════════════════════════
PORTABLE A ESP32 / C++

Equivalente C++:
    typedef struct {
        float body_length, body_width;
        float L_coxa, L_femur, L_tibia;
        float coxa_lim[2], femur_lim[2], tibia_lim[2];  // [min_deg, max_deg]
        float rest_x, rest_y, rest_z;
    } RobotParams;
"""

from dataclasses import dataclass
from typing import Tuple


@dataclass
class RobotParams:
    """Parámetros físicos del robot. Modificar según tu plataforma."""

    # Dimensiones del cuerpo [m]
    body_length: float = 0.20
    body_width:  float = 0.14

    # Longitudes de segmentos [m]
    L_coxa:  float = 0.05
    L_femur: float = 0.10
    L_tibia: float = 0.12

    # Límites articulares [grados] — adaptar a tus servos
    coxa_lim:  Tuple[float, float] = (-80.0,  80.0)
    femur_lim: Tuple[float, float] = (-90.0,  90.0)
    tibia_lim: Tuple[float, float] = (-150.0,  0.0)

    # Postura de reposo del pie (frame de pata, relativo a pivot coxa) [m]
    rest_x: float =  0.12   # extensión radial
    rest_y: float =  0.0    # lateral
    rest_z: float = -0.10   # altura (negativo = abajo)

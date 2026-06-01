"""
robot_math.py — Utilidades matemáticas comunes
═══════════════════════════════════════════════
PORTABLE A ESP32 / C++

Equivalencias directas en C++ (<math.h>):
    np.cos(a)   → cosf(a)
    np.sin(a)   → sinf(a)
    np.radians  → (deg * M_PI / 180.0f)

Las matrices 3×3 se representan como arrays planos en C++:
    float R[9];   // fila mayor: R[row*3 + col]
"""

import numpy as np


def rot_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Matriz de rotación ZYX intrínseco: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).

    C++ (inline):
        Rx = {{1,0,0},{0,cr,-sr},{0,sr,cr}}
        Ry = {{cp,0,sp},{0,1,0},{-sp,0,cp}}
        Rz = {{cy,-sy,0},{sy,cy,0},{0,0,1}}
        R  = Rz * Ry * Rx
    """
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)
    Rx = np.array([[1,   0,   0 ],
                   [0,  cr, -sr ],
                   [0,  sr,  cr ]])
    Ry = np.array([[ cp, 0,  sp],
                   [  0, 1,   0],
                   [-sp, 0,  cp]])
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]])
    return Rz @ Ry @ Rx


def rot_z(angle: float) -> np.ndarray:
    """
    Rotación pura alrededor del eje Z.

    C++ (inline):
        float c = cosf(a), s = sinf(a);
        float Rz[9] = {c,-s,0, s,c,0, 0,0,1};
    """
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0., 0., 1.0]])

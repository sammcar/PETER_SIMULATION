"""
robot_imu.py — Simulación y compensación de inclinación IMU
════════════════════════════════════════════════════════════
PORTABLE A ESP32 / C++: solo IMUCompensator
NO PORTABLE (simulación): IMUSimulator

En el robot real, IMUSimulator se reemplaza por la lectura I2C del BMI160:
    bmi160_get_roll_pitch(&roll_raw, &pitch_raw);

El compensador en C++ (ver comentarios de compensate()):
    Variables de estado globales:
        float roll_f = 0.f, pitch_f = 0.f;

    Por ciclo de control:
        imu_compensate(foot_body, roll_raw, pitch_raw);
        for (int i=0; i<4; i++) leg_ik(i, foot_body[i]);
"""

import numpy as np

from ver_def.robot_math import rot_rpy


# ──────────────────────────────────────────────────────────────────────
#  SIMULADOR IMU  (solo Python, no se porta a ESP32)
# ──────────────────────────────────────────────────────────────────────

class IMUSimulator:
    """
    Simula la salida del BMI160: roll y pitch con perturbación sinusoidal.

    En el hardware real, reemplazar read(t) por la lectura del BMI160.
    El BMI160 entrega roll y pitch ya filtrados internamente (filtro
    complementario integrado en el chip).
    """

    def __init__(self,
                 roll_amp:  float = np.radians(8.0),
                 pitch_amp: float = np.radians(6.0),
                 freq:      float = 0.4):
        self.roll_amp  = roll_amp
        self.pitch_amp = pitch_amp
        self.freq      = freq

    def read(self, t: float):
        """Retorna (roll_rad, pitch_rad) en el instante t [s]."""
        w     = 2.0 * np.pi * self.freq
        roll  = self.roll_amp  * np.sin(w * t)
        pitch = self.pitch_amp * np.sin(w * t * 1.3 + 0.8)
        return float(roll), float(pitch)


# ──────────────────────────────────────────────────────────────────────
#  COMPENSADOR IMU  (PORTABLE A ESP32 / C++)
# ──────────────────────────────────────────────────────────────────────

class IMUCompensator:
    """
    Mantiene el cuerpo nivelado aplicando la rotación inversa al IMU.

    Pipeline por ciclo:
        1. Filtro pasa-bajos 1er orden: filtered = alpha*raw + (1-alpha)*prev
        2. Corrección inversa:          roll_c  = clip(-roll_filt,  ±max_corr)
                                        pitch_c = clip(-pitch_filt, ±max_corr)
        3. Rotar pies:                  foot = rot_rpy(roll_c, pitch_c, 0) @ foot

    alpha → 0 : suavizado fuerte (lento pero estable)
    alpha → 1 : respuesta rápida (puede vibrar)

    C++ equivalente (inline, sin clases):
    ─────────────────────────────────────
        // Estado global
        static float roll_f = 0.f, pitch_f = 0.f;

        void imu_compensate(float foot[4][3], float roll, float pitch) {
            roll_f  = ALPHA*roll  + (1-ALPHA)*roll_f;
            pitch_f = ALPHA*pitch + (1-ALPHA)*pitch_f;

            float rc = fmaxf(-MAX_C, fminf(MAX_C, -roll_f));
            float pc = fmaxf(-MAX_C, fminf(MAX_C, -pitch_f));

            float cr=cosf(rc), sr=sinf(rc), cp=cosf(pc), sp=sinf(pc);
            for (int i=0; i<4; i++) {
                float x0=foot[i][0];
                float y1=cr*foot[i][1] - sr*foot[i][2];
                float z1=sr*foot[i][1] + cr*foot[i][2];
                foot[i][0] = cp*x0 + sp*z1;
                foot[i][1] = y1;
                foot[i][2] = -sp*x0 + cp*z1;
            }
        }
    """

    def __init__(self, alpha: float = 0.15, max_correction_deg: float = 20.0):
        self.alpha        = alpha
        self.max_corr_rad = np.radians(max_correction_deg)
        self._roll_f  = 0.0
        self._pitch_f = 0.0

    @property
    def roll_filtered(self)  -> float: return self._roll_f

    @property
    def pitch_filtered(self) -> float: return self._pitch_f

    def compensate(self, foot_targets: dict, roll: float, pitch: float) -> dict:
        """
        Aplica compensación IMU a las posiciones de los pies (frame cuerpo).

        foot_targets : {name: np.array([x,y,z])}
        roll, pitch  : medición del IMU [rad]
        Retorna el mismo dict con posiciones rotadas.
        """
        self._roll_f  = self.alpha * roll  + (1.0 - self.alpha) * self._roll_f
        self._pitch_f = self.alpha * pitch + (1.0 - self.alpha) * self._pitch_f

        roll_c  = float(np.clip(-self._roll_f,  -self.max_corr_rad, self.max_corr_rad))
        pitch_c = float(np.clip(-self._pitch_f, -self.max_corr_rad, self.max_corr_rad))

        R_comp = rot_rpy(roll_c, pitch_c, 0.0)
        return {name: R_comp @ pos for name, pos in foot_targets.items()}

    def reset(self):
        """Reinicia el filtro pasa-bajos."""
        self._roll_f  = 0.0
        self._pitch_f = 0.0

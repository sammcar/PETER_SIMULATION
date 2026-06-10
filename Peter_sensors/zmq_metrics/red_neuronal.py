#!/usr/bin/env python3
"""
red_neuronal.py — Red neuronal biomimética de ganglios basales. Puerto ZMQ.

Suscripciones ZMQ:
  /bounding_box/red   (Float32MultiArray) — posición y área del estímulo rojo
  /bounding_box/blue  (Float32MultiArray) — posición y área del estímulo azul
  /imu/data           (Imu)              — orientación y aceleración
  /scan               (LaserScan)        — distancias del lidar

Publicaciones ZMQ:
  /cmd_vel            (Twist)            — comandos de velocidad al robot
  /peter_mode         (String)           — modo de locomoción: C / H / X
  neuron_activity     (Float32MultiArray)— actividad neuronal completa (85+ valores)
  imu_activity        (Float32MultiArray)— [accel_std, pitch]
  /experiment/metrics → publicado por neural_recorder (suscrito a neuron_activity)
  /Metrics            (Float32MultiArray)— [Tresponse, Tswitch, roll_rms, pitch_rms, noise_idx]
  /Gpe_Hostil         (Float64)          — diagnóstico GPe rojo
  /Gpe_Obstaculo      (Float64)          — diagnóstico GPe obstáculo
  /Gpe_Apetente       (Float64)          — diagnóstico GPe azul

Nota: /peter/stability_data NO se publica aquí porque requiere cinemática directa
      de las patas (sensores no disponibles en el robot físico).

Uso:
  python red_neuronal.py [--nl 0]
  --nl   índice de nivel de ruido: 0=0% 1=5% 2=10% 3=20% 4=30% (default 0)
"""

import argparse
import sys
import os
import time
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))

from transport import (
    Node, spin,
    Twist, String,
    Float32MultiArray, Float64,
    Imu, LaserScan,
)
from topics import (
    TOPIC_BBOX_RED, TOPIC_BBOX_BLUE,
    TOPIC_IMU, TOPIC_SCAN,
    TOPIC_CMD_VEL, TOPIC_PETER_MODE,
    TOPIC_METRICS,
    TOPIC_NEURON_ACTIVITY,
)

# Tópicos de diagnóstico GPe (solo para monitoreo)
TOPIC_GPE_R = '/Gpe_Hostil'
TOPIC_GPE_G = '/Gpe_Obstaculo'
TOPIC_GPE_B = '/Gpe_Apetente'
TOPIC_IMU_ACTIVITY = 'imu_activity'


class NetworkPublisher(Node):

    def __init__(self, noise_level_idx: int = 0):
        super().__init__('network_publisher')

        # ── Publicadores ───────────────────────────────────────────────────
        self.cmd_vel_pub   = self.create_publisher(Twist,            TOPIC_CMD_VEL,       10)
        self.mode_pub      = self.create_publisher(String,           TOPIC_PETER_MODE,    10)
        self.GPEr          = self.create_publisher(Float64,          TOPIC_GPE_R,         10)
        self.GPEg          = self.create_publisher(Float64,          TOPIC_GPE_G,         10)
        self.GPEb          = self.create_publisher(Float64,          TOPIC_GPE_B,         10)
        self.publisher_    = self.create_publisher(Float32MultiArray, TOPIC_NEURON_ACTIVITY, 10)
        self.publisher_imu = self.create_publisher(Float32MultiArray, TOPIC_IMU_ACTIVITY,  10)
        self.metrics_pub   = self.create_publisher(Float32MultiArray, TOPIC_METRICS,       10)

        # ── Estado inicial ─────────────────────────────────────────────────
        self.current_mode     = 'C'
        self.areaBoundingBoxR = 0.0
        self.areaBoundingBoxG = 0.0
        self.areaBoundingBoxB = 0.0
        self.posR = 0.0
        self.posG = 0.0
        self.posB = 0.0

        # ── Suscriptores ───────────────────────────────────────────────────
        self.create_subscription(Float32MultiArray, TOPIC_BBOX_RED,  self.red_callback,   100)
        self.create_subscription(Float32MultiArray, TOPIC_BBOX_BLUE, self.blue_callback,  100)
        self.create_subscription(Imu,               TOPIC_IMU,       self.imu_callback,    10)
        self.create_subscription(LaserScan,         TOPIC_SCAN,      self.lidar_callback,  10)

        # ── Constantes y variables de dinámica neuronal ────────────────────
        self._noise_level_idx = noise_level_idx
        self.initctes()

        # ── Timer de control (≈6.7 Hz) ─────────────────────────────────────
        self.create_timer(0.15, self.run_network)

    # ══════════════════════════════════════════════════════════════════════
    # INICIALIZACIÓN (sin cambios respecto al original)
    # ══════════════════════════════════════════════════════════════════════

    def initctes(self):

        # ── Neuronas ───────────────────────────────────────────────────────
        self.Gpi      = np.zeros((3, 2))
        self.Gpe      = np.zeros((3, 2))
        self.StN      = np.zeros((3, 2))
        self.StR      = np.zeros((3, 2))
        self.z        = np.zeros((20, 2))
        self.lidar    = np.zeros((5, 2))
        self.Response = np.zeros((16, 2))
        self.Aux      = np.zeros((16, 2))

        # ── Debug ──────────────────────────────────────────────────────────
        self.MOVEMENT = True
        self.maxstd   = []

        # ── IMU ────────────────────────────────────────────────────────────
        self.ignore_imu      = False
        self.ignore_timer    = time.time()
        self.ignore_duration = 2
        self.accel_buffer    = deque(maxlen=100)
        self.accel_std       = 0.0
        self.last_accel_std  = 0.0
        self.accel_std2      = 0.0

        self.terrainchanger  = False
        self.terrain_timer   = 0.0

        self.ang_p   = 90
        self.ang_s   = 90
        self.epsilem = 0.01
        self.dt      = 1
        self.cte     = 3
        self.Area    = 28

        self.roll  = 0.0
        self.pitch = 0.0
        self.std_dev_accel_z   = 0.0
        self.accel_z_history   = []
        self.w  = 10
        self.j  = 2
        self.A  = 5
        self.Sigma    = 0.3
        self.SigmaIMU = 1.5

        self.tau      = 1
        self.tauMotor = 2
        self.TaoGpi   = 1
        self.TaoGpe   = 2
        self.TaoSTN   = 2
        self.TaoSTR   = 1

        self.Usigma_az = 2.9
        self.Upitch    = 20
        self.Uroll     = 270

        self.W_input_to_response  = np.fliplr(np.eye(16))
        self.weights_r_r          = -np.ones((16, 16)) + np.eye(16)
        self.W_response_to_aux    = np.triu(np.ones((16, 16)))

        # ── Lidar ──────────────────────────────────────────────────────────
        self.n_neuronas       = 16
        self.n_por_cuadrante  = self.n_neuronas // 4
        self.activaciones_totales = np.zeros(self.n_neuronas)

        cuadrante_1 = np.linspace(0,   90,  self.n_por_cuadrante, endpoint=False)
        cuadrante_2 = np.linspace(90,  180, self.n_por_cuadrante, endpoint=False)
        cuadrante_3 = np.linspace(180, 270, self.n_por_cuadrante, endpoint=False)
        cuadrante_4 = np.linspace(270, 360, self.n_por_cuadrante, endpoint=False)

        self.Directions_deg = np.concatenate([cuadrante_1, cuadrante_2, cuadrante_3, cuadrante_4])
        self.Directions_rad = np.radians(self.Directions_deg)
        self.Om    = np.vstack((np.cos(self.Directions_rad), np.sin(self.Directions_rad)))
        self.sigma = 0.06
        self.umbral = 0.95

        # ── Métricas ───────────────────────────────────────────────────────
        self.metricsArr = [0.0, 0.0, 0.0, 0.0]

        self.starttime        = time.time()
        self.tchange          = self.starttime + 11.0
        self.Tresponse        = None
        self.response_measured = False

        self.tcmd                    = None
        self.Tswitch                 = None
        self.mode_transition_active  = False
        self.switch_measured         = False

        self.roll_history  = []
        self.pitch_history = []
        self.roll_rms      = 0.0
        self.pitch_rms     = 0.0

        # ── Nivel de ruido (reemplaza declare_parameter / get_parameter) ───
        self.NoiseLevel = [0.0, 0.05, 0.10, 0.20, 0.30]
        self.ACTIVE_NOISE_LEVEL_IDX = self._noise_level_idx
        self._rng         = np.random.default_rng(42)
        self._sigma_noise = self.NoiseLevel[self.ACTIVE_NOISE_LEVEL_IDX]

        print(
            f'[ROBUSTEZ] Nivel de ruido: '
            f'σ={self._sigma_noise*100:.0f}%  '
            f'(índice {self.ACTIVE_NOISE_LEVEL_IDX} de {self.NoiseLevel})'
        )

    # ══════════════════════════════════════════════════════════════════════
    # RUIDO (sin cambios)
    # ══════════════════════════════════════════════════════════════════════

    def _add_gaussian_noise(self, value: float, sigma_fraction: float) -> float:
        if sigma_fraction == 0.0:
            return value
        sigma_abs = max(abs(value) * sigma_fraction, 1e-4)
        return value + self._rng.normal(0.0, sigma_abs)

    def _add_gaussian_noise_array(self, arr: np.ndarray, sigma_fraction: float) -> np.ndarray:
        if sigma_fraction == 0.0:
            return arr.copy()
        sigma_abs = np.maximum(np.abs(arr) * sigma_fraction, 1e-4)
        return arr + self._rng.normal(0.0, sigma_abs)

    def gausiana(self, theta, omega):
        return np.exp((np.dot(theta, omega) - 1) / (2 * (self.sigma ** 2)))

    # ══════════════════════════════════════════════════════════════════════
    # CALLBACKS (sin cambios en la lógica)
    # ══════════════════════════════════════════════════════════════════════

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)

        if self._sigma_noise > 0.0:
            valid = np.isfinite(ranges)
            noisy = ranges.copy()
            noisy[valid] = self._add_gaussian_noise_array(ranges[valid], self._sigma_noise)
            noisy[valid] = np.maximum(noisy[valid], 0.0)
            ranges = noisy

        angle_min       = msg.angle_min
        angle_increment = msg.angle_increment

        self.activaciones_totales = np.zeros(self.n_neuronas)
        r_min, r_max = 0.2, 1.0

        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_increment
            if np.isnan(r) or np.isinf(r) or r < r_min or r > r_max:
                continue
            vector_input = np.array([np.cos(angle), np.sin(angle)])
            activaciones = np.array([
                self.gausiana(vector_input, self.Om[:, j])
                for j in range(self.n_neuronas)
            ]) * 130 / (1000 * r)
            self.activaciones_totales = np.maximum(self.activaciones_totales, activaciones)

    def red_callback(self, msg):
        if len(msg.data) >= 2:
            self.posR             = self._add_gaussian_noise(msg.data[0], self._sigma_noise)
            self.areaBoundingBoxR = max(0.0, self._add_gaussian_noise(msg.data[1], self._sigma_noise))

    def blue_callback(self, msg):
        if len(msg.data) >= 2:
            self.posB             = self._add_gaussian_noise(msg.data[0], self._sigma_noise)
            self.areaBoundingBoxB = max(0.0, self._add_gaussian_noise(msg.data[1], self._sigma_noise))

    def imu_callback(self, msg):
        qx, qy, qz, qw = (msg.orientation.x, msg.orientation.y,
                          msg.orientation.z, msg.orientation.w)

        self.roll  = 180 - abs(np.degrees(np.arctan2(2*(qw*qx + qy*qz),
                                                      1 - 2*(qx**2 + qy**2))))
        roll_centered = self.roll - 180.0
        self.pitch = abs(np.degrees(np.arcsin(2*(qw*qy - qz*qx))))

        ax = self._add_gaussian_noise(msg.linear_acceleration.x, self._sigma_noise)
        ay = self._add_gaussian_noise(msg.linear_acceleration.y, self._sigma_noise)
        az = self._add_gaussian_noise(msg.linear_acceleration.z, self._sigma_noise)

        accel_mag = np.sqrt(ax**2 + ay**2 + az**2)
        self.accel_buffer.append(accel_mag)

        if self.ignore_imu:
            if time.time() - self.ignore_timer < self.ignore_duration:
                self.accel_std = self.last_accel_std
                return
            else:
                self.ignore_imu = False

        self.accel_std  = np.std(self.accel_buffer) if len(self.accel_buffer) > 1 else 0.0
        self.accel_std2 = self.accel_std
        self.last_accel_std = self.accel_std

        self.maxstd.append(self.accel_std)
        self.roll_history.append(roll_centered)
        self.pitch_history.append(self.pitch)
        self.roll_rms  = np.sqrt(np.mean(np.square(self.roll_history)))
        self.pitch_rms = np.sqrt(np.mean(np.square(self.pitch_history)))

    # ══════════════════════════════════════════════════════════════════════
    # DINÁMICA NEURONAL PRINCIPAL (sin cambios)
    # ══════════════════════════════════════════════════════════════════════

    def run_network(self):
        cmd_lineal  = 0.0
        cmd_ang     = 0.0
        cmd_lateral = 0.0

        # ── Lidar ──────────────────────────────────────────────────────────
        self.lidar[0,1] = self.lidar[0,0] + (self.dt/10)*(-self.lidar[0,0] + (np.sum(self.activaciones_totales[0:4]) + np.sum(self.activaciones_totales[12:16]) - self.lidar[1,0]))
        self.lidar[1,1] = self.lidar[1,0] + (self.dt/10)*(-self.lidar[1,0] + (np.sum(self.activaciones_totales[4:12]) - self.lidar[0,0]))
        self.lidar[2,1] = self.lidar[2,0] + (self.dt/10)*(-self.lidar[2,0] + (np.sum(self.activaciones_totales[0:8]) - self.lidar[3,0]))
        self.lidar[3,1] = self.lidar[3,0] + (self.dt/10)*(-self.lidar[3,0] + (np.sum(self.activaciones_totales[8:16]) - self.lidar[2,0]))

        for r in range(16):
            self.Response[r,1] = self.Response[r,0] + (self.dt/5)*(-self.Response[r,0] + max(0, (self.W_input_to_response @ self.activaciones_totales)[r] + self.weights_r_r[r,:] @ self.Response[:,0]))
            self.Aux[r,1]      = self.Aux[r,0]      + (self.dt/5)*(-self.Aux[r,0]      + max(0, self.W_response_to_aux[r,:] @ self.Response[:,1]))

        self.lidar[4,1] = self.lidar[4,0] + (self.dt/self.tau)*(-self.lidar[4,0]*1.1 + max(0, np.sum(self.Aux[:,0])))

        R = self.areaBoundingBoxR / 500
        #R = 0
        G = self.lidar[4,0]*15 if self.lidar[4,0]*15 > 0.2 else 0
        #G = 0
        B = self.areaBoundingBoxB / 500
        #B = 0

        print("R: ", str(R))
        print("G: ", str(G))
        print("B: ", str(B))

        # ── Ganglios basales ───────────────────────────────────────────────
        self.StN[0,1] = np.clip(self.StN[0,0] + (1/self.TaoSTN)*(-self.StN[0,0] + R - self.Gpi[0,0] - self.Gpe[1,0] - self.Gpe[2,0] - 1.0), 0, None)
        self.StN[1,1] = np.clip(self.StN[1,0] + (1/self.TaoSTN)*(-self.StN[1,0] + G - self.Gpi[1,0] - self.Gpe[0,0] - self.Gpe[2,0] - 1.0), 0, None)
        self.StN[2,1] = np.clip(self.StN[2,0] + (1/self.TaoSTN)*(-self.StN[2,0] + B*0.7 - self.Gpi[2,0] - self.Gpe[0,0] - self.Gpe[1,0] - 1.0), 0, None)

        self.Gpi[0,1] = np.clip(self.Gpi[0,0] + (1/self.TaoGpi)*(-self.Gpi[0,0] + self.StN[1,0] + self.StN[2,0] - self.Gpe[0,0] - self.StR[0,0]), 0, None)
        self.Gpi[1,1] = np.clip(self.Gpi[1,0] + (1/self.TaoGpi)*(-self.Gpi[1,0] + self.StN[0,0] + self.StN[2,0] - self.Gpe[1,0] - self.StR[1,0]), 0, None)
        self.Gpi[2,1] = np.clip(self.Gpi[2,0] + (1/self.TaoGpi)*(-self.Gpi[2,0] + self.StN[0,0] + self.StN[1,0] - self.Gpe[2,0] - self.StR[2,0]), 0, None)

        self.Gpe[0,1] = np.clip(self.Gpe[0,0] + (1/self.TaoGpe)*(-self.Gpe[0,0] + self.StN[0,0]),       0, None)
        self.Gpe[1,1] = np.clip(self.Gpe[1,0] + (1/self.TaoGpe)*(-self.Gpe[1,0] + self.StN[1,0]*5),     0, None)
        self.Gpe[2,1] = np.clip(self.Gpe[2,0] + (1/self.TaoGpe)*(-self.Gpe[2,0] + self.StN[2,0]*0.5),   0, None)

        self.StR[0,1] = np.clip(self.StR[0,0] + (1/self.TaoSTR)*(-self.StR[0,0] + self.StN[0,0]), 0, None)
        self.StR[1,1] = np.clip(self.StR[1,0] + (1/self.TaoSTR)*(-self.StR[1,0] + self.StN[1,0]), 0, None)
        self.StR[2,1] = np.clip(self.StR[2,0] + (1/self.TaoSTR)*(-self.StR[2,0] + self.StN[2,0]), 0, None)

        if   self.Gpe[0,1] > 1.5 and R > 0.5: self.ang_s = self.posR
        elif self.Gpe[2,1] > 1.5 and B > 0.5: self.ang_s = self.posB
        else: self.ang_s = 90 * (self.lidar[4,1] < 0.3)

        #self.ang_s = 90  # fijo para prueba de terreno inclinado

        # ── Módulo IMU ─────────────────────────────────────────────────────
        self.z[0,1]  = self.z[0,0]  + (self.dt/self.tau)*(-self.z[0,0]  + (self.A * max(0, self.std_dev_accel_z - self.Usigma_az)**2) / (self.SigmaIMU**2 + (-self.z[0,0]  + self.std_dev_accel_z - self.Usigma_az)**2))
        self.z[1,1]  = self.z[1,0]  + (self.dt/self.tau)*(-self.z[1,0]  + (self.A * max(0, self.pitch - self.Upitch)**2)              / (self.SigmaIMU**2 + (-self.z[1,0]  + self.pitch - self.Upitch)**2))
        self.z[2,1]  = self.z[2,0]  + (self.dt/self.tau)*(-self.z[2,0]  + (self.A * max(0, self.roll  - self.Uroll)**2)               / (self.SigmaIMU**2 + (-self.z[2,0]  + self.roll  - self.Uroll)**2))
        self.z[3,1]  = self.z[3,0]  + (self.dt/self.tau)*(-self.z[3,0]  + max(0, self.Gpe[2,0]))
        self.z[4,1]  = self.z[4,0]  + (self.dt/self.tau)*(-self.z[4,0]  + max(0, self.Gpe[1,0] + self.j*self.Gpe[0,0]))

        self.z[5,1]  = self.z[5,0]  + (self.dt/self.tau)*(-self.z[5,0]  + max(0, self.lidar[2,0]*160 + (self.lidar[4,1]<0.3)*((self.ang_s - self.ang_p) - 15)))
        self.z[6,1]  = self.z[6,0]  + (self.dt/self.tau)*(-self.z[6,0]  + max(0, self.lidar[3,0]*160 + (self.lidar[4,1]<0.3)*((self.ang_p - self.ang_s) - 15)))

        self.z[7,1]  = self.z[7,0]  + (self.dt/self.tau)*(-self.z[7,0]  + max(0, self.z[5,0] + self.z[3,0] - self.w*self.z[4,0]))
        self.z[8,1]  = self.z[8,0]  + (self.dt/self.tau)*(-self.z[8,0]  + max(0, self.z[5,0] + self.z[4,0] - self.w*self.z[3,0]))
        self.z[9,1]  = self.z[9,0]  + (self.dt/self.tau)*(-self.z[9,0]  + max(0, self.z[4,0] + self.z[6,0] - self.w*self.z[3,0]))
        self.z[10,1] = self.z[10,0] + (self.dt/self.tau)*(-self.z[10,0] + max(0, self.z[3,0] + self.z[6,0] - self.w*self.z[4,0]))

        self.z[11,1] = self.z[11,0] + (self.dt/self.tau)*(-self.z[11,0] + max(0, self.z[7,0]  + self.z[9,0]))
        self.z[12,1] = self.z[12,0] + (self.dt/self.tau)*(-self.z[12,0] + max(0, self.z[10,0] + self.z[8,0]))
        self.z[13,1] = self.z[13,0] + (self.dt/self.tau)*(-self.z[13,0] + max(0, -self.w*abs(cmd_ang)*self.z[11,0] - self.w*abs(cmd_ang)*self.z[12,0] - 2*self.w*self.z[17,0] + self.cte + self.Gpe[2,0]))

        self.z[14,1] = self.z[14,0] + (self.dt/self.tau)*(-self.z[14,0] + (self.A * max(0, 100*self.Gpe[1,0] - self.w*self.Gpi[0,0] - self.w*self.Gpi[2,0] - self.w*self.z[15,0] - self.w*self.z[16,0])**2) / (self.Sigma**2 + (100*self.Gpe[1,0] - self.w*self.Gpi[0,0] - self.w*self.Gpi[2,0] - self.w*self.z[15,0] - self.w*self.z[16,0])**2))
        self.z[15,1] = self.z[15,0] + (self.dt/self.tau)*(-self.z[15,0] + (self.A * max(0, -self.Gpe[1,0]*100 - self.cte + self.z[4,0]*self.w + 2*self.z[0,0] - self.w*self.z[14,0]*1.5 - self.w*self.z[16,0])**2) / (self.Sigma**2 + (-self.Gpe[1,0]*100 - self.cte + self.z[4,0]*self.w + 2*self.z[0,0] - self.w*self.z[14,0]*1.5 - self.w*self.z[16,0])**2))
        self.z[16,1] = self.z[16,0] + (self.dt/self.tau)*(-self.z[16,0] + (self.A * max(0, self.z[3,0] - 100*self.Gpe[1,0] - self.w*self.z[14,0]*1.5 - self.w*self.z[15,0]*1.5 + 5*self.z[1,0] + 5*self.z[2,0] + self.cte)**2) / (self.Sigma**2 + (self.z[3,0] - 100*self.Gpe[1,0] - self.w*self.z[14,0]*1.5 - self.w*self.z[15,0]*1.5 + 5*self.z[1,0] + 5*self.z[2,0] + self.cte)**2))

        self.z[17,1] = self.z[17,0] + (self.dt/self.tau)*(-self.z[17,0] + max(0, self.Gpe[2,0] - self.Area))

        # ── Comandos de velocidad ──────────────────────────────────────────
        cmd_ang     = (self.z[11,0]*(self.lidar[4,0]<0.3)) - (self.z[12,0]*(self.lidar[4,0]<0.3))
        cmd_lateral = (self.lidar[2,0]*1.5 + self.lidar[3,0]*1.5 + self.z[11,0]*((self.Gpe[1,0]>0.5) and (self.Gpe[2,0]<0.5))) - (self.z[12,0]*((self.Gpe[1,0]>0.5) and (self.Gpe[2,0]<0.5)))
        cmd_lineal  = self.lidar[0,0]*1.5 - self.lidar[1,0]*1.5 + self.z[13,0] - self.j*self.z[4,0]*(self.z[5,0]<self.epsilem and self.z[6,0]<self.epsilem)

        # ── Actualizar estados (t → t+1) ───────────────────────────────────
        for i in range(len(self.z)):        self.z[i,0]        = self.z[i,1]        * (self.z[i,1]        > self.epsilem)
        for i in range(len(self.lidar)):    self.lidar[i,0]    = self.lidar[i,1]    * (self.lidar[i,1]    > self.epsilem)
        for i in range(len(self.Response)): self.Response[i,0] = self.Response[i,1] * (self.Response[i,1] > self.epsilem)
        for i in range(len(self.Aux)):      self.Aux[i,0]      = self.Aux[i,1]      * (self.Aux[i,1]      > self.epsilem)
        for i in range(len(self.StN)):      self.StN[i,0]      = self.StN[i,1]      * (self.StN[i,1]      > self.epsilem)
        for i in range(len(self.Gpi)):      self.Gpi[i,0]      = self.Gpi[i,1]      * (self.Gpi[i,1]      > self.epsilem)
        for i in range(len(self.Gpe)):      self.Gpe[i,0]      = self.Gpe[i,1]      * (self.Gpe[i,1]      > self.epsilem)
        for i in range(len(self.StR)):      self.StR[i,0]      = self.StR[i,1]      * (self.StR[i,1]      > self.epsilem)

        print("GpeR: ", str(self.Gpe[0,1]))
        print("GpeG: ", str(self.Gpe[1,1]))
        print("GpeB: ", str(self.Gpe[2,1]))

        print("ang_p: ", str(self.ang_p))
        print("ang_s: ", str(self.ang_s))

        #print("5: ", str(self.z[5, 1]))
        #print("3: ", str(self.z[3, 1]))
        #print("4: ", str(self.z[4, 1]))
        #print("6: ", str(self.z[6, 1]))

        #print("7: ", str(self.z[7, 1]))
        #print("8: ", str(self.z[8, 1]))
        #print("9: ", str(self.z[9, 1]))
        #print("10: ", str(self.z[10, 1]))

        #print("11: ", str(self.z[11, 1]))
        #print("12: ", str(self.z[12, 1]))
        #print("13: ", str(self.z[13, 1]))

        print("14: ", str(self.z[14, 1]))
        print("16: ", str(self.z[16, 1]))  
        print("15: ", str(self.z[15, 1]))  
        print("17: ", str(self.z[17, 1]))

        #print("0: ", str(self.z[0, 1]))
        #print("1: ", str(self.z[1, 1]))
        #print("2: ", str(self.z[2, 1])) 
        #print("a: ", str(self.std_dev_accel_z))
        #print("roll: ", str(self.roll))
        #print("pitch: ", str(self.pitch))

        print("cmd_ang: ", str(cmd_ang))
        print("cmd_lineal: ", str(cmd_lineal))
        print("cmd_lateral: ", str(cmd_lateral))

        print("lidar frente: ", str(self.lidar[0,0]))
        print("lidar atras: ", str(self.lidar[1,0]))
        print("lidar izquierda: ", str(self.lidar[2,0]))
        print("lidar derecha:", str(self.lidar[3,0]))
        print("lidar 4:", str(self.lidar[4,0]))

        # ── Terrain changer ────────────────────────────────────────────────
        if self.accel_std > self.Usigma_az and not self.terrainchanger:
            self.terrainchanger  = True
            self.std_dev_accel_z = 9
            self.terrain_timer   = time.time()

        if self.terrainchanger:
            if time.time() - self.terrain_timer < 40:
                self.std_dev_accel_z = 9
            else:
                self.terrainchanger  = False
                self.std_dev_accel_z = 0
        else:
            self.std_dev_accel_z = 0

        # ── Log consola ────────────────────────────────────────────────────

        if not self.MOVEMENT:
            return

        # ── Publicar movimiento ────────────────────────────────────────────
        cmd_ang     = self.limit(cmd_ang,     1)
        cmd_lineal  = self.limit(cmd_lineal,  5)
        cmd_lateral = self.limit(cmd_lateral, 5)

        ang = cmd_ang     if abs(cmd_ang)     > self.epsilem else 0.0
        lat = cmd_lateral if abs(cmd_lateral) > self.epsilem else 0.0
        lin = cmd_lineal  if abs(cmd_lineal)  > self.epsilem else 0.0

        tw_ang, tw_lat, tw_lin = 0.0, 0.0, 0.0
        if   self.z[17,1] > 0.25:                      pass          # STOP
        elif ang != 0.0:                                tw_ang = ang
        elif lat != 0.0 and self.z[16,1] < 0.5:        tw_lat = lat
        elif lin != 0.0:                                tw_lin = lin

        self.publish_twist(tw_lin, tw_lat, tw_ang)

        # ── Modo ───────────────────────────────────────────────────────────
        if   self.z[15,1] > 0.5: 
            self.publish_mode('C')
            print("Cuadrupedo")
        elif self.z[16,1] > 0.5: 
            self.publish_mode('H')
            print("Móvil H")
        elif self.z[14,1] > 0.5: 
            self.publish_mode('X')
            print("Omnidireccional")

        # ── Actividad neuronal y GPe ───────────────────────────────────────
        self.publish_data()
        self.publish_imu()
        self.publicarGPE(self.Gpe[0,1], self.Gpe[1,1], self.Gpe[2,1])

        # ── Métricas ───────────────────────────────────────────────────────
        stable = (self.accel_std < 2 and self.pitch < 1.5 and self.roll > 178)

        if self.terrainchanger and not self.response_measured and stable:
            self.Tresponse         = time.time() - self.tchange
            self.response_measured = True

        if self.mode_transition_active and not self.switch_measured and stable:
            self.Tswitch                = time.time() - self.tcmd
            self.switch_measured        = True
            self.mode_transition_active = False

        self.metricsArr = [
            self.Tresponse, self.Tswitch,
            self.roll_rms,  self.pitch_rms,
            self.ACTIVE_NOISE_LEVEL_IDX,
        ]
        self.publicarMetricas()

    # ══════════════════════════════════════════════════════════════════════
    # FUNCIONES AUXILIARES
    # ══════════════════════════════════════════════════════════════════════

    def limit(self, value, maximum):
        return max(-maximum, min(maximum, value))

    def publicarGPE(self, gper=0.0, gpeg=0.0, gpeb=0.0):
        for pub, val in [(self.GPEr, gper), (self.GPEg, gpeg), (self.GPEb, gpeb)]:
            msg      = Float64()
            msg.data = float(val)
            pub.publish(msg)

    def publicarMetricas(self):
        msg      = Float32MultiArray()
        msg.data = [float(x) if x is not None else 0.0 for x in self.metricsArr]
        self.metrics_pub.publish(msg)

    def publish_twist(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        if not hasattr(self, '_twist'):
            self._twist = Twist()
        self._twist.linear.x  = float(linear_x)
        self._twist.linear.y  = float(linear_y)
        self._twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(self._twist)

    def publish_mode(self, mode: str):
        msg      = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        if self.current_mode != mode and self.current_mode in ('C', 'H'):
            self.ignore_timer           = time.time()
            self.ignore_imu             = True
            self.tcmd                   = time.time()
            self.mode_transition_active = True
        self.current_mode = mode

    def publish_data(self):
        parts = [
            np.asarray(self.Gpi)[:,1],
            np.asarray(self.Gpe)[:,1],
            np.asarray(self.StN)[:,1],
            np.asarray(self.StR)[:,1],
            np.asarray(self.z)[:,1],
            np.asarray(self.lidar)[:,1],
            np.asarray(self.activaciones_totales),
            np.asarray(self.Response)[:,1],
            np.asarray(self.Aux)[:,1],
        ]
        msg      = Float32MultiArray()
        msg.data = np.concatenate([p.ravel() for p in parts]).astype(np.float32).tolist()
        self.publisher_.publish(msg)

    def publish_imu(self):
        msg      = Float32MultiArray()
        msg.data = [float(self.accel_std2), float(self.pitch)]
        self.publisher_imu.publish(msg)


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='Red Neuronal PETER — ZMQ')
    parser.add_argument('--nl', type=int, default=0,
                        help='Índice de nivel de ruido: 0=0%% 1=5%% 2=10%% 3=20%% 4=30%%')
    args = parser.parse_args()

    node = NetworkPublisher(noise_level_idx=args.nl)
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()

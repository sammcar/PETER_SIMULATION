#!/usr/bin/env python3
"""
test_manager.py — Orquestador híbrido de experimentos científicos para PETER.

Arquitectura concurrente (Executor Híbrido):
  - Hilo Principal  : Gestiona el ciclo de vida de procesos (subprocess.Popen) y la
                      máquina de estados (INIT → WARMUP → RUNNING → TERMINATION → TEARDOWN).
  - Hilo Secundario : Ejecuta rclpy.spin() sobre TestJudgeNode para escuchar tópicos
                      de ROS 2 sin bloquear al hilo principal.

Sincronización de reloj:
  El orquestador consume tiempo exclusivamente desde /clock (simulación).
  PROHIBIDO usar time.time() para calcular timeouts de experimento.

Señales de terminación:
  SIGINT → grace_period (5 s) → SIGKILL si quedan procesos zombis.
"""

from __future__ import annotations

import csv
import json
import logging
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from enum import Enum, auto
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseArray
import yaml

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s][%(levelname)s] %(message)s',
    datefmt='%H:%M:%S',
)
log = logging.getLogger('test_manager')

# ── Constantes globales ───────────────────────────────────────────────────────
CRIT_TR: float = 1.0          # Riesgo de volteo crítico (volcadura)
X17_THRESHOLD: float = 0.8    # Umbral activación neurona de parada
SUCCESS_HOLD_S: float = 2.0   # Segundos continuos de éxito requeridos
WARMUP_S: float = 10.5        # Segundos de calentamiento (TimerActions en launch)
SAFETY_STABLE_S: float = 12.0 # Ventana de estabilidad post-transición (Familia C)
ACCEL_Z_THRESH: float = 3.7   # Umbral σ aceleración Z para terreno rugoso (C1)
PITCH_THRESH_DEG: float = 5.0 # Umbral pitch para pendiente inclinada (C2)
GRACE_PERIOD_S: float = 5.0   # Tiempo de gracia para flush de CSVs tras SIGINT
STABILITY_LOG_SRC: Path = Path.home() / 'stability_log.csv'

# ── Suites con lógica de éxito específica ────────────────────────────────────
# Usadas para seleccionar la rama correcta de evaluación en RUNNING.
SUITE_APPETITIVE = {'familia_a_apetitivo', 'familia_b_compleja'}
SUITE_EVASIVE    = {'familia_a_obstaculo', 'familia_a_aversivo'}
SUITE_TERRAIN_C1 = {'familia_c1_terreno_rugoso'}
SUITE_TERRAIN_C2 = {'familia_c2_pendiente'}


# ── Máquina de estados ────────────────────────────────────────────────────────
class State(Enum):
    INIT        = auto()
    WARMUP      = auto()
    RUNNING     = auto()
    TERMINATION = auto()
    TEARDOWN    = auto()


class Verdict(str, Enum):
    SUCCESS          = 'SUCCESS'
    FAILURE_TIPOVER  = 'FAILURE_TIPOVER'
    FAILURE_TIMEOUT  = 'FAILURE_TIMEOUT'
    FAILURE_CRASH    = 'FAILURE_CRASH'


# ── Nodo juez de ROS 2 (hilo secundario) ─────────────────────────────────────
class TestJudgeNode(Node):
    """
    Nodo ROS 2 que escucha tópicos de métricas y expone variables atómicas
    protegidas por Lock al hilo principal.

    Suscripciones activas:
      /clock              → sincronización de tiempo de simulación
      /neuron_activity    → actividades neuronales (X0..X17)
      /experiment/metrics → [latency, firing_var, consistency, lambda] (neural_recorder)
      /Metrics            → [Tresponse, Tswitch, roll_rms, pitch_rms, noise_idx]
      /peter/stability_data → PoseArray para cálculo en línea de TR (3 patas)
      /peter_mode         → modo locomotor actual
    """

    def __init__(self) -> None:
        super().__init__('test_judge_node', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self._lock = threading.Lock()

        # ── Estado compartido ──────────────────────────────────────────────
        # Reloj de simulación (nanosegundos)
        self._sim_time_ns: int = 0

        # Neuronas (índice = número de neurona)
        self._neuron_activity: List[float] = [0.0] * 20

        # /Metrics: [Tresponse, Tswitch, roll_rms, pitch_rms, noise_idx]
        self._tresponse: float = 0.0
        self._tswitch: float = 0.0
        self._roll_rms: float = 0.0
        self._pitch_rms: float = 0.0
        self._noise_idx: int = 0

        # /experiment/metrics: [latency, firing_var, consistency, lambda]
        self._exp_latency: float = -1.0
        self._exp_lambda: float = 0.0

        # Riesgo de volteo calculado en línea desde /peter/stability_data
        self._tr: float = 0.0

        # Modo locomotor actual (String del tópico /peter_mode)
        self._current_mode: str = ''

        # Bounding boxes (para A2/A3: estímulo fuera de vista)
        self._bb_red_area: float = 0.0
        self._bb_blue_area: float = 0.0
        self._bb_green_area: float = 0.0

        # ── QoS ───────────────────────────────────────────────────────────
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        qos_reliable = QoSProfile(depth=10)

        # ── Suscriptores ──────────────────────────────────────────────────
        self.create_subscription(Clock, '/clock', self._cb_clock, qos_best_effort)
        self.create_subscription(
            Float32MultiArray, '/neuron_activity', self._cb_neurons, 50
        )
        self.create_subscription(
            Float32MultiArray, '/experiment/metrics', self._cb_exp_metrics, 10
        )
        self.create_subscription(
            Float32MultiArray, '/Metrics', self._cb_metrics, 10
        )
        self.create_subscription(
            PoseArray, '/peter/stability_data', self._cb_stability, 10
        )
        self.create_subscription(String, '/peter_mode', self._cb_mode, 10)
        self.create_subscription(
            Float32MultiArray, '/bounding_box/red', self._cb_bb_red, 100
        )
        self.create_subscription(
            Float32MultiArray, '/bounding_box/blue', self._cb_bb_blue, 100
        )
        self.create_subscription(
            Float32MultiArray, '/bounding_box/green', self._cb_bb_green, 100
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_clock(self, msg: Clock) -> None:
        ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        with self._lock:
            self._sim_time_ns = ns

    def _cb_neurons(self, msg: Float32MultiArray) -> None:
        with self._lock:
            data = list(msg.data)
            # Mantener buffer de al menos 20 neuronas para acceso seguro
            while len(data) < 20:
                data.append(0.0)
            self._neuron_activity = data

    def _cb_exp_metrics(self, msg: Float32MultiArray) -> None:
        with self._lock:
            if len(msg.data) >= 4:
                self._exp_latency = msg.data[0]
                self._exp_lambda  = msg.data[3]

    def _cb_metrics(self, msg: Float32MultiArray) -> None:
        # Layout: [Tresponse, Tswitch, roll_rms, pitch_rms, noise_idx]
        with self._lock:
            if len(msg.data) >= 4:
                self._tresponse = msg.data[0]
                self._tswitch   = msg.data[1]
                self._roll_rms  = msg.data[2]
                self._pitch_rms = msg.data[3]
                if len(msg.data) >= 5:
                    self._noise_idx = int(msg.data[4])

    def _cb_stability(self, msg: PoseArray) -> None:
        """
        Replica la lógica de compute_stability() de peter_stability_monitor
        para obtener TR en tiempo real sin depender del flush del CSV.
        Solo válido con exactamente 3 patas en contacto.
        """
        if len(msg.poses) < 5:
            return

        feet_all = [
            (-msg.poses[i].position.y, msg.poses[i].position.x)
            for i in range(4)
        ]
        on_ground = [msg.poses[i].orientation.w > 0.5 for i in range(4)]
        grounded  = [feet_all[i] for i in range(4) if on_ground[i]]
        com       = (-msg.poses[4].position.y, msg.poses[4].position.x)

        if len(grounded) != 3:
            return

        tr = self._compute_tr(grounded, com)
        with self._lock:
            self._tr = tr

    def _compute_tr(
        self,
        feet3: List[tuple[float, float]],
        com: tuple[float, float],
    ) -> float:
        """Calcula el Riesgo de Volteo (TR = 1 − SM_norm) a partir de 3 contactos."""
        a, b, c = self._ensure_ccw(feet3)
        side_lens = [
            math.hypot(b[0]-a[0], b[1]-a[1]),
            math.hypot(c[0]-b[0], c[1]-b[1]),
            math.hypot(a[0]-c[0], a[1]-c[1]),
        ]
        perim = sum(side_lens)
        area  = abs(
            (a[0]*(b[1]-c[1]) + b[0]*(c[1]-a[1]) + c[0]*(a[1]-b[1])) / 2.0
        )
        r_in  = (2.0 * area / perim) if perim > 1e-9 else 1e-9
        dists = [
            self._signed_dist(a, b, com),
            self._signed_dist(b, c, com),
            self._signed_dist(c, a, com),
        ]
        sm      = min(dists)
        sm_norm = sm / r_in
        return 1.0 - sm_norm

    @staticmethod
    def _signed_dist(
        pi: tuple[float, float],
        pj: tuple[float, float],
        c:  tuple[float, float],
    ) -> float:
        num = (pj[0]-pi[0])*(c[1]-pi[1]) - (pj[1]-pi[1])*(c[0]-pi[0])
        den = math.hypot(pj[0]-pi[0], pj[1]-pi[1])
        return num / den if den > 1e-9 else 0.0

    @staticmethod
    def _ensure_ccw(
        pts: List[tuple[float, float]],
    ) -> tuple[tuple, tuple, tuple]:
        a, b, c = pts
        cross = (b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1])
        return (a, b, c) if cross > 0 else (a, c, b)

    def _cb_mode(self, msg: String) -> None:
        with self._lock:
            self._current_mode = msg.data

    def _cb_bb_red(self, msg: Float32MultiArray) -> None:
        with self._lock:
            self._bb_red_area = float(msg.data[1]) if len(msg.data) >= 2 else 0.0

    def _cb_bb_blue(self, msg: Float32MultiArray) -> None:
        with self._lock:
            self._bb_blue_area = float(msg.data[1]) if len(msg.data) >= 2 else 0.0

    def _cb_bb_green(self, msg: Float32MultiArray) -> None:
        with self._lock:
            self._bb_green_area = float(msg.data[1]) if len(msg.data) >= 2 else 0.0

    # ── Accesores atómicos (llamados desde el hilo principal) ─────────────────

    def get_sim_time_s(self) -> float:
        with self._lock:
            return self._sim_time_ns * 1e-9

    def snapshot(self) -> Dict[str, Any]:
        """Devuelve una copia thread-safe de todas las variables de estado."""
        with self._lock:
            return {
                'sim_time_s':    self._sim_time_ns * 1e-9,
                'neurons':       list(self._neuron_activity),
                'tresponse':     self._tresponse,
                'tswitch':       self._tswitch,
                'roll_rms':      self._roll_rms,
                'pitch_rms':     self._pitch_rms,
                'noise_idx':     self._noise_idx,
                'exp_latency':   self._exp_latency,
                'exp_lambda':    self._exp_lambda,
                'tr':            self._tr,
                'mode':          self._current_mode,
                'bb_red':        self._bb_red_area,
                'bb_blue':       self._bb_blue_area,
                'bb_green':      self._bb_green_area,
            }


# ── Helpers de proceso ────────────────────────────────────────────────────────

def _kill_process_group(proc: subprocess.Popen, grace_s: float) -> None:
    """Envía SIGINT al grupo de procesos; tras grace_s aplica SIGKILL si persiste."""
    try:
        pgid = os.getpgid(proc.pid)
        log.info(f'Enviando SIGINT a PGID={pgid}')
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return  # Ya terminó

    # Fase de gracia: esperar cierre limpio de buffers CSV
    deadline = time.monotonic() + grace_s
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            log.info('Proceso terminó limpiamente dentro del grace period.')
            return
        time.sleep(0.2)

    # Terminación forzada si quedan procesos zombis
    if proc.poll() is None:
        log.warning('Grace period expirado. Aplicando SIGKILL al grupo...')
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        # Limpiar residuos de Gazebo/ROS 2 que puedan haber quedado
        for pattern in ('gz sim', 'ros2 launch', 'robot_state_publisher'):
            subprocess.run(['pkill', '-9', '-f', pattern],
                           capture_output=True, check=False)
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()


# ── Lógica de evaluación por familia ─────────────────────────────────────────

class TrialEvaluator:
    """
    Encapsula la lógica de éxito/fracaso para una iteración de prueba.
    Lleva contadores internos de tiempo continuo sobre condiciones de éxito.
    Usada exclusivamente desde el hilo principal.
    """

    def __init__(self, suite_name: str, timeout_s: float) -> None:
        self.suite_name = suite_name
        self.timeout_s  = timeout_s
        self._success_hold_start: Optional[float] = None  # tiempo de simulación
        self._mode_switched: bool = False
        self._post_switch_start: Optional[float] = None   # tiempo de simulación

    def evaluate(
        self,
        snap: Dict[str, Any],
        sim_start_s: float,
        warmup_done: bool,
    ) -> Optional[Verdict]:
        """
        Evalúa el estado actual y retorna un Verdict si se alcanza una condición
        terminal, o None si la prueba debe continuar.

        Args:
            snap         : snapshot atómico del nodo juez.
            sim_start_s  : tiempo de simulación en el que inició RUNNING.
            warmup_done  : si True, la fase WARMUP ya concluyó.

        Returns:
            Verdict o None.
        """
        if not warmup_done:
            return None

        current_sim_s: float = snap['sim_time_s']
        elapsed: float = current_sim_s - sim_start_s

        # ── Fracaso global: timeout ────────────────────────────────────────
        if elapsed >= self.timeout_s:
            log.warning(f'[TIMEOUT] {elapsed:.1f}s >= {self.timeout_s}s')
            return Verdict.FAILURE_TIMEOUT

        # ── Fracaso global: volteo ─────────────────────────────────────────
        if snap['tr'] >= CRIT_TR:
            log.warning(f'[TIPOVER] TR={snap["tr"]:.3f} >= {CRIT_TR}')
            return Verdict.FAILURE_TIPOVER

        # ── Éxito por familia ──────────────────────────────────────────────
        if self.suite_name in SUITE_APPETITIVE:
            return self._eval_appetitive(snap, current_sim_s)

        if self.suite_name in SUITE_EVASIVE:
            return self._eval_evasive(snap, current_sim_s)

        if self.suite_name in SUITE_TERRAIN_C1:
            return self._eval_terrain_c1(snap, current_sim_s)

        if self.suite_name in SUITE_TERRAIN_C2:
            return self._eval_terrain_c2(snap, current_sim_s)

        # Suite desconocida: solo comprueba timeout/tipover (ya aplicado)
        return None

    def _eval_appetitive(
        self, snap: Dict[str, Any], current_sim_s: float
    ) -> Optional[Verdict]:
        """
        Familias A1 y B: X17 > 0.8 Y Tresponse > 0 mantenidos 2 s continuos.
        """
        neurons = snap['neurons']
        x17 = neurons[17] if len(neurons) > 17 else 0.0
        tresponse_active = snap['tresponse'] > 0.0
        cond = (x17 > X17_THRESHOLD) and tresponse_active

        if cond:
            if self._success_hold_start is None:
                self._success_hold_start = current_sim_s
            elif (current_sim_s - self._success_hold_start) >= SUCCESS_HOLD_S:
                log.info(f'[SUCCESS] X17={x17:.3f}, Tresponse={snap["tresponse"]:.3f}')
                return Verdict.SUCCESS
        else:
            self._success_hold_start = None  # Reiniciar contador si se pierde la condición

        return None

    def _eval_evasive(
        self, snap: Dict[str, Any], current_sim_s: float
    ) -> Optional[Verdict]:
        """
        Familias A2 y A3: transición de modo + estímulo fuera del campo visual
        + TR < 1.0 + Tresponse válido.
        """
        neurons = snap['neurons']
        x14_active = (neurons[14] > 0.3) if len(neurons) > 14 else False
        # Para aversivo (A3) también puede ser modo 'H' (diferencial)
        mode_switched = x14_active or (snap['mode'] in ('H', 'C'))
        stimulus_gone = (snap['bb_red_area'] < 1.0) and (snap['bb_green_area'] < 1.0)
        tr_safe       = snap['tr'] < CRIT_TR
        tresponse_ok  = snap['tresponse'] > 0.0

        cond = mode_switched and stimulus_gone and tr_safe and tresponse_ok

        if cond:
            if self._success_hold_start is None:
                self._success_hold_start = current_sim_s
            elif (current_sim_s - self._success_hold_start) >= SUCCESS_HOLD_S:
                log.info(
                    f'[SUCCESS] evasión confirmada — modo={snap["mode"]}, '
                    f'bb_red={snap["bb_red_area"]:.1f}, TR={snap["tr"]:.3f}'
                )
                return Verdict.SUCCESS
        else:
            self._success_hold_start = None

        return None

    def _eval_terrain_c1(
        self, snap: Dict[str, Any], current_sim_s: float
    ) -> Optional[Verdict]:
        """
        Familia C1 (terreno rugoso → plano):
        1. std_dev_z > ACCEL_Z_THRESH dispara X0 → cambio a cuadrúpedo (X15 > 0).
        2. Tswitch publicado (transición mecánica completada).
        3. SAFETY_STABLE_S posteriores sin TR >= 1.0.
        """
        neurons = snap['neurons']
        x0 = neurons[0] if neurons else 0.0
        x15 = neurons[15] if len(neurons) > 15 else 0.0
        tswitch_done = snap['tswitch'] > 0.0

        # Detectar transición: X0 activo y X15 activo (cuadrúpedo habilitado)
        transition_detected = (x0 > 0.1) and (x15 > 0.1) and tswitch_done

        if not self._mode_switched and transition_detected:
            self._mode_switched    = True
            self._post_switch_start = current_sim_s
            log.info(f'[C1] Transición a cuadrúpedo detectada @ {current_sim_s:.1f}s')

        if self._mode_switched and self._post_switch_start is not None:
            post_elapsed = current_sim_s - self._post_switch_start
            if snap['tr'] >= CRIT_TR:
                log.warning(f'[C1] TR crítico post-transición: {snap["tr"]:.3f}')
                return Verdict.FAILURE_TIPOVER
            if post_elapsed >= SAFETY_STABLE_S:
                log.info(f'[SUCCESS] C1 — terreno rugoso superado con estabilidad.')
                return Verdict.SUCCESS

        return None

    def _eval_terrain_c2(
        self, snap: Dict[str, Any], current_sim_s: float
    ) -> Optional[Verdict]:
        """
        Familia C2 (pendiente inclinada):
        1. pitch > PITCH_THRESH_DEG dispara X1 → cambio a modo móvil H.
        2. Tswitch publicado.
        3. SAFETY_STABLE_S posteriores sin TR >= 1.0.
        """
        neurons = snap['neurons']
        x1 = neurons[1] if len(neurons) > 1 else 0.0
        tswitch_done = snap['tswitch'] > 0.0

        # pitch_rms es en grados según la nota en red_neuronal.py (Upitch = 20)
        pitch_triggered = (snap['pitch_rms'] > PITCH_THRESH_DEG) or (x1 > 0.1)
        transition_detected = pitch_triggered and tswitch_done

        if not self._mode_switched and transition_detected:
            self._mode_switched     = True
            self._post_switch_start = current_sim_s
            log.info(
                f'[C2] Transición a modo móvil @ {current_sim_s:.1f}s '
                f'(pitch={snap["pitch_rms"]:.2f}°, X1={x1:.3f})'
            )

        if self._mode_switched and self._post_switch_start is not None:
            post_elapsed = current_sim_s - self._post_switch_start
            if snap['tr'] >= CRIT_TR:
                log.warning(f'[C2] TR crítico post-transición: {snap["tr"]:.3f}')
                return Verdict.FAILURE_TIPOVER
            if post_elapsed >= SAFETY_STABLE_S:
                log.info(f'[SUCCESS] C2 — pendiente superada con estabilidad.')
                return Verdict.SUCCESS

        return None


# ── Gestión de artefactos ─────────────────────────────────────────────────────

def _collect_artifacts(
    trial_dir: Path,
    suite_name: str,
    trial_idx: int,
    verdict: Verdict,
    seed_info: Dict[str, Any],
    snap_final: Dict[str, Any],
) -> None:
    """
    Mueve los artefactos generados por la simulación al directorio de resultados:
      - stability_log.csv (~/stability_log.csv)
      - metrics.csv  (~/peter_experiments/<exp>_<ts>/metrics.csv, el más reciente)
    Además, escribe un resumen JSON de la iteración.
    """
    trial_dir.mkdir(parents=True, exist_ok=True)

    # stability_log.csv
    if STABILITY_LOG_SRC.exists():
        dst = trial_dir / 'stability_log.csv'
        shutil.move(str(STABILITY_LOG_SRC), str(dst))
        log.info(f'  → stability_log.csv → {dst}')
    else:
        log.debug('  stability_log.csv no encontrado (modo sin cuadrúpedo)')

    # metrics.csv del neural_recorder (directorio más reciente en ~/peter_experiments)
    exp_base = Path.home() / 'peter_experiments'
    if exp_base.exists():
        candidates = sorted(
            exp_base.iterdir(),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
        for candidate in candidates:
            csv_src = candidate / 'metrics.csv'
            if csv_src.exists():
                dst = trial_dir / 'metrics.csv'
                shutil.move(str(csv_src), str(dst))
                log.info(f'  → metrics.csv → {dst}')
                # Limpiar el directorio vacío resultante
                try:
                    candidate.rmdir()
                except OSError:
                    pass
                break

    # Resumen JSON de la iteración
    summary = {
        'trial_index': trial_idx,
        'suite_name':  suite_name,
        'verdict':     verdict.value,
        'seed_info':   seed_info,
        'final_metrics': {
            k: (round(v, 4) if isinstance(v, float) else v)
            for k, v in snap_final.items()
            if k not in ('neurons',)  # omitir arrays largos del JSON
        },
    }
    with open(trial_dir / 'trial_summary.json', 'w') as f:
        json.dump(summary, f, indent=2)
    log.info(f'  → trial_summary.json escrito')


def _update_manifest(
    manifest_path: Path,
    trial_idx: int,
    seed_info: Dict[str, Any],
    verdict: Verdict,
) -> None:
    """Añade o actualiza la entrada del manifiesto de la suite."""
    data: List[Dict] = []
    if manifest_path.exists():
        with open(manifest_path) as f:
            data = json.load(f)
    data.append({
        'trial_index': trial_idx,
        'verdict':     verdict.value,
        **seed_info,
    })
    with open(manifest_path, 'w') as f:
        json.dump(data, f, indent=2)


# ── Inyección de parámetros dinámicos ────────────────────────────────────────

def _build_launch_args(
    suite_cfg: Dict[str, Any],
    trial_idx: int,
    noise_idx: int,
    rng: np.random.Generator,
) -> List[str]:
    """
    Construye la lista de argumentos para ros2 launch.
    Aplica perturbación gaussiana y ruido sensorial según la config de la suite.
    """
    dp: Dict[str, Any] = suite_cfg.get('dynamic_parameters', {})
    args: List[str] = []
    seed_info: Dict[str, Any] = {
        'trial_index': trial_idx,
        'noise_level_idx': noise_idx,
        'perturbations': {},
    }

    random_perturb: bool = dp.get('random_perturbation', False)
    sigma: float = float(dp.get('variance_sigma', 0.15))

    def _perturbed(base_key: str) -> tuple[str, float]:
        base_val = float(dp.get(base_key, 0.0))
        if random_perturb:
            delta = float(rng.normal(0.0, sigma))
            perturbed = base_val + delta
            seed_info['perturbations'][base_key] = {
                'base': base_val, 'delta': round(delta, 4)
            }
            return str(round(perturbed, 4)), perturbed
        return str(base_val), base_val

    # Parámetros independientes de la familia
    if 'world_name' in dp:
        args += [f'world_name:={dp["world_name"]}']

    # Familia A: single_stimulus
    if 'stimulus_type' in dp:
        args += [f'stimulus_type:={dp["stimulus_type"]}']

    if 'stimulus_x' in dp:
        sx_str, _ = _perturbed('stimulus_x')
        args += [f'stimulus_x:={sx_str}']

    if 'stimulus_y' in dp:
        sy_str, _ = _perturbed('stimulus_y')
        args += [f'stimulus_y:={sy_str}']

    # Familia B: multiple_stimuli
    for flag in ('spawn_green', 'spawn_red', 'spawn_blue'):
        if flag in dp:
            args += [f'{flag}:={str(dp[flag]).lower()}']

    for coord in ('green_x', 'green_y', 'red_x', 'red_y', 'blue_x', 'blue_y'):
        if coord in dp:
            c_str, _ = _perturbed(coord) if random_perturb else (str(dp[coord]), None)
            args += [f'{coord}:={c_str}']

    # Ruido sensorial (índice inyectado a red_neuronal vía parámetro)
    # La red neuronal expone el parámetro 'nl' (noise level index)
    args += [f'noise_level_idx:={noise_idx}']

    return args, seed_info


# ── Bucle principal ───────────────────────────────────────────────────────────

class TestManager:
    """
    Orquestador principal. Gestiona:
      - Carga de la configuración YAML.
      - Ciclo de vida del nodo ROS 2 en hilo secundario.
      - Iteración sobre suites y repeticiones.
      - Máquina de estados por iteración.
      - Recolección y almacenamiento de artefactos.
    """

    def __init__(self, config_path: str) -> None:
        with open(config_path) as f:
            self._cfg: Dict[str, Any] = yaml.safe_load(f)

        gs = self._cfg.get('global_settings', {})
        self._workspace: Path = Path(os.path.expanduser(
            gs.get('workspace_path', '~/PETER_SIMULATION/ros2_ws')
        ))
        self._output_base: Path = Path(os.path.expanduser(
            gs.get('output_base_dir', '~/PETER_SIMULATION/Findings')
        ))
        self._grace_period: float = float(
            gs.get('grace_period_teardown_s', GRACE_PERIOD_S)
        )

        # Ejecutable ROS 2 fuente
        self._ros_setup  = f'source /opt/ros/humble/setup.bash && source {self._workspace}/install/setup.bash'
        self._rng        = np.random.default_rng()

        # Estado interno del nodo ROS 2
        self._judge_node: Optional[TestJudgeNode] = None
        self._ros_thread: Optional[threading.Thread] = None

    # ── Ciclo de vida de ROS 2 ─────────────────────────────────────────────

    def _start_ros_thread(self) -> None:
        """Inicializa rclpy y arranca el hilo secundario con el nodo juez."""
        if not rclpy.ok():
            rclpy.init()
        self._judge_node = TestJudgeNode()

        def _spin() -> None:
            try:
                rclpy.spin(self._judge_node)
            except Exception as exc:
                log.error(f'[ROS spin] excepción: {exc}')

        self._ros_thread = threading.Thread(target=_spin, name='ros2_spin', daemon=True)
        self._ros_thread.start()
        log.info('Hilo de ROS 2 (TestJudgeNode) iniciado.')

    def _stop_ros_thread(self) -> None:
        if self._judge_node is not None:
            self._judge_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if self._ros_thread is not None:
            self._ros_thread.join(timeout=5.0)
        log.info('Hilo de ROS 2 detenido.')

    # ── Lanzamiento de simulación ──────────────────────────────────────────

    def _launch_simulation(
        self,
        launch_file: str,
        extra_args: List[str],
    ) -> subprocess.Popen:
        """
        Arranca la simulación con subprocess.Popen en un nuevo grupo de procesos
        para poder enviar SIGINT/SIGKILL al grupo completo.
        """
        cmd_str = (
            f'{self._ros_setup} && '
            f'ros2 launch peter_robot {launch_file} '
            + ' '.join(extra_args)
        )
        log.info(f'[LAUNCH] {cmd_str}')
        proc = subprocess.Popen(
            cmd_str,
            shell=True,
            executable='/bin/bash',
            # Nuevo grupo de procesos → permite killpg
            preexec_fn=os.setsid,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        return proc

    # ── Máquina de estados por iteración ──────────────────────────────────

    def _run_trial(
        self,
        suite_cfg: Dict[str, Any],
        trial_idx: int,
        noise_idx: int,
    ) -> tuple[Verdict, Dict[str, Any], subprocess.Popen]:
        """
        Ejecuta un ciclo completo INIT → WARMUP → RUNNING → TERMINATION.
        Retorna (Verdict, seed_info, proceso).
        """
        dp = suite_cfg.get('dynamic_parameters', {})
        timeout_s: float = float(dp.get('timeout_s', 60.0))
        suite_name: str  = suite_cfg['suite_name']

        # ── INIT: preparar parámetros ──────────────────────────────────────
        log.info(f'[INIT] suite={suite_name} trial={trial_idx:03d} noise_idx={noise_idx}')
        launch_args, seed_info = _build_launch_args(
            suite_cfg, trial_idx, noise_idx, self._rng
        )
        seed_info['noise_level_idx'] = noise_idx

        # Lanzar proceso
        proc = self._launch_simulation(suite_cfg['launch_file'], launch_args)

        # ── WARMUP: esperar a que los nodos se estabilicen ─────────────────
        log.info(f'[WARMUP] {WARMUP_S:.1f}s para estabilización de nodos...')
        state = State.WARMUP
        warmup_start_wall = time.monotonic()
        while True:
            if proc.poll() is not None:
                log.error('[WARMUP] Proceso terminó inesperadamente.')
                return Verdict.FAILURE_CRASH, seed_info, proc
            if time.monotonic() - warmup_start_wall >= WARMUP_S:
                break
            time.sleep(0.2)

        # ── RUNNING: escuchar tópicos y evaluar condiciones ────────────────
        log.info(f'[RUNNING] Evaluación activa. timeout={timeout_s}s')
        state = State.RUNNING
        evaluator = TrialEvaluator(suite_name=suite_name, timeout_s=timeout_s)

        # Capturar el tiempo de simulación al inicio de RUNNING
        # Reintentar hasta 5 s si /clock todavía no ha llegado
        sim_start_s: float = 0.0
        for _ in range(50):
            candidate = self._judge_node.get_sim_time_s()
            if candidate > 0.0:
                sim_start_s = candidate
                break
            time.sleep(0.1)

        log.info(f'[RUNNING] sim_start_s={sim_start_s:.2f}')
        snap_final: Dict[str, Any] = {}
        verdict: Optional[Verdict] = None

        while verdict is None:
            if proc.poll() is not None:
                log.error('[RUNNING] Proceso terminó inesperadamente.')
                verdict = Verdict.FAILURE_CRASH
                break

            snap = self._judge_node.snapshot()
            snap_final = snap

            verdict = evaluator.evaluate(snap, sim_start_s, warmup_done=True)
            time.sleep(0.05)  # 20 Hz de evaluación

        # ── TERMINATION: veredicto emitido ────────────────────────────────
        state = State.TERMINATION
        log.info(f'[TERMINATION] Veredicto: {verdict.value}')
        _kill_process_group(proc, self._grace_period)

        return verdict, seed_info, proc

    # ── Teardown: recolección de artefactos ───────────────────────────────

    def _teardown(
        self,
        suite_name: str,
        trial_idx: int,
        verdict: Verdict,
        seed_info: Dict[str, Any],
        snap_final: Dict[str, Any],
    ) -> None:
        state = State.TEARDOWN
        trial_label = f'test_{trial_idx:03d}_{verdict.value}'
        trial_dir   = self._output_base / suite_name / trial_label

        log.info(f'[TEARDOWN] Recolectando artefactos → {trial_dir}')
        _collect_artifacts(trial_dir, suite_name, trial_idx, verdict, seed_info, snap_final)

        manifest_path = self._output_base / suite_name / 'suite_execution_manifest.json'
        _update_manifest(manifest_path, trial_idx, seed_info, verdict)

    # ── Punto de entrada ──────────────────────────────────────────────────

    def run(self) -> None:
        """Punto de entrada del orquestador. Itera sobre todas las suites."""
        self._start_ros_thread()
        log.info('=== TEST MANAGER INICIADO ===')

        noise_levels: List[int] = [0, 1, 2, 3, 4]  # Índices 0%..30%

        try:
            for suite_cfg in self._cfg.get('experiment_suites', []):
                suite_name  = suite_cfg['suite_name']
                repetitions = int(suite_cfg.get('repetitions', 1))

                log.info(f'\n{"="*60}')
                log.info(f'Suite: {suite_name} | Repeticiones: {repetitions}')
                log.info(f'{"="*60}')

                (self._output_base / suite_name).mkdir(parents=True, exist_ok=True)

                trial_global_idx = 0
                for rep in range(repetitions):
                    # Ciclo de inyección de ruido: itera sobre niveles disponibles
                    noise_idx = noise_levels[rep % len(noise_levels)]

                    trial_global_idx += 1
                    log.info(
                        f'\n[TRIAL {trial_global_idx:03d}/{repetitions}] '
                        f'suite={suite_name} rep={rep+1} noise_idx={noise_idx}'
                    )

                    verdict, seed_info, proc = self._run_trial(
                        suite_cfg, trial_global_idx, noise_idx
                    )

                    # Asegurar que el proceso está terminado antes de teardown
                    if proc.poll() is None:
                        try:
                            proc.wait(timeout=3.0)
                        except subprocess.TimeoutExpired:
                            proc.kill()

                    # Obtener snapshot final para el resumen
                    snap_final = self._judge_node.snapshot()
                    self._teardown(suite_name, trial_global_idx, verdict, seed_info, snap_final)

                    # Breve pausa entre iteraciones para que Gazebo libere recursos
                    log.info('Pausa entre iteraciones (3 s)...')
                    time.sleep(3.0)

        except KeyboardInterrupt:
            log.warning('Interrupción manual del orquestador (SIGINT recibido).')
        finally:
            self._stop_ros_thread()
            log.info('=== TEST MANAGER FINALIZADO ===')


# ── Punto de entrada del script ───────────────────────────────────────────────

def main() -> None:
    # Resolver ruta de configuración: argumento o ruta canónica del paquete
    if len(sys.argv) > 1:
        config_path = sys.argv[1]
    else:
        # Ruta canónica dentro del workspace de ROS 2
        config_path = os.path.join(
            os.path.expanduser('~/PETER_SIMULATION/ros2_ws'),
            'src', 'peter_robot', 'config', 'experiments_config.yaml',
        )

    if not os.path.isfile(config_path):
        log.error(f'Archivo de configuración no encontrado: {config_path}')
        sys.exit(1)

    manager = TestManager(config_path)
    manager.run()


if __name__ == '__main__':
    main()

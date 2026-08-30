#!/usr/bin/env python3
"""
test_manager.py — Orquestador híbrido de experimentos científicos para PETER.
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
from collections import deque
from pynput import keyboard
from typing import Dict, Any, Optional

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s][%(levelname)s] %(message)s',
    datefmt='%H:%M:%S',
)
log = logging.getLogger('test_manager')

# ── Constantes globales ───────────────────────────────────────────────────────
CRIT_TR: float = 1.0
TIPOVER_HOLD_S: float = 3.0          
X17_THRESHOLD: float = 0.45    # Umbral activación neurona de parada
SUCCESS_HOLD_S: float = 2.0   
WARMUP_S: float = 10.5        
SAFETY_STABLE_S: float = 12.0 
ACCEL_Z_THRESH: float = 3.7   
PITCH_THRESH_DEG: float = 5.0 
GRACE_PERIOD_S: float = 5.0   
STABILITY_LOG_SRC: Path = Path.home() / 'stability_log.csv'

# ── Suites con lógica de éxito específica ────────────────────────────────────
SUITE_APPETITIVE = {'familia_a_apetitivo', 'familia_b_compleja'}
SUITE_EVASIVE    = {'familia_a_obstaculo', 'familia_a_aversivo'}
SUITE_TERRAIN_C1 = {'familia_c1_terreno_rugoso'}
SUITE_TERRAIN_C2 = {'familia_c2_pendiente'}

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

class TestJudgeNode(Node):
    def __init__(self) -> None:
        super().__init__('test_judge_node', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])
        self._lock = threading.Lock()
        self._sim_time_ns: int = 0
        self._neuron_activity: List[float] = [0.0] * 30
        self._tcall: float = 0.0
        self._tresponse: float = 0.0
        self._tswitch: float = 0.0
        self._roll_rms: float = 0.0
        self._pitch_rms: float = 0.0
        self._noise_idx: int = 0
        self._exp_latency: float = -1.0
        self._exp_lambda: float = 0.0
        self._tr: float = 0.0
        self._current_mode: str = ''
        self._bb_red_area: float = 0.0
        self._bb_blue_area: float = 0.0
        self._bb_green_area: float = 0.0

        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Clock, '/clock', self._cb_clock, qos_best_effort)
        self.create_subscription(Float32MultiArray, '/neuron_activity', self._cb_neurons, 50)
        self.create_subscription(Float32MultiArray, '/experiment/metrics', self._cb_exp_metrics, 10)
        self.create_subscription(Float32MultiArray, '/Metrics', self._cb_metrics, 10)
        self.create_subscription(PoseArray, '/peter/stability_data', self._cb_stability, 10)
        self.create_subscription(String, '/peter_mode', self._cb_mode, 10)
        self.create_subscription(Float32MultiArray, '/bounding_box/red', self._cb_bb_red, 100)
        self.create_subscription(Float32MultiArray, '/bounding_box/blue', self._cb_bb_blue, 100)
        self.create_subscription(Float32MultiArray, '/bounding_box/green', self._cb_bb_green, 100)

    def _cb_clock(self, msg: Clock) -> None:
        ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        with self._lock: self._sim_time_ns = ns

    def _cb_neurons(self, msg: Float32MultiArray) -> None:
        with self._lock:
            data = list(msg.data)
            while len(data) < 20: data.append(0.0)
            self._neuron_activity = data

    def _cb_exp_metrics(self, msg: Float32MultiArray) -> None:
        with self._lock:
            if len(msg.data) >= 4:
                self._exp_latency = msg.data[0]
                self._exp_lambda  = msg.data[3]

    def _cb_metrics(self, msg: Float32MultiArray) -> None:
        with self._lock:
            if len(msg.data) >= 4:
                self._tcall       = msg.data[0]
                self._tresponse   = msg.data[0] 
                self._tswitch     = msg.data[1]
                self._roll_rms    = msg.data[2]
                self._pitch_rms   = msg.data[3]
                if len(msg.data) >= 5:
                    self._noise_idx = int(msg.data[4])

    def _cb_stability(self, msg: PoseArray) -> None:
        if len(msg.poses) < 5: return
        feet_all = [(-msg.poses[i].position.y, msg.poses[i].position.x) for i in range(4)]
        on_ground = [msg.poses[i].orientation.w > 0.5 for i in range(4)]
        grounded  = [feet_all[i] for i in range(4) if on_ground[i]]
        com       = (-msg.poses[4].position.y, msg.poses[4].position.x)
        
        # Corrección del valor congelado
        if len(grounded) == 4:
            tr = 0.0  # Recuperó el balance de las 4 patas: máxima estabilidad
        elif len(grounded) == 3:
            tr = self._compute_tr(grounded, com)
        else:
            return  # Cayendo o saltando: mantiene el último valor para la ventana de tiempo
            
        with self._lock: self._tr = tr

    def _compute_tr(self, feet3: List[tuple[float, float]], com: tuple[float, float]) -> float:
        a, b, c = self._ensure_ccw(feet3)
        side_lens = [math.hypot(b[0]-a[0], b[1]-a[1]), math.hypot(c[0]-b[0], c[1]-b[1]), math.hypot(a[0]-c[0], a[1]-c[1])]
        perim = sum(side_lens)
        area  = abs((a[0]*(b[1]-c[1]) + b[0]*(c[1]-a[1]) + c[0]*(a[1]-b[1])) / 2.0)
        r_in  = (2.0 * area / perim) if perim > 1e-9 else 1e-9
        dists = [self._signed_dist(a, b, com), self._signed_dist(b, c, com), self._signed_dist(c, a, com)]
        sm_norm = min(dists) / r_in
        return 1.0 - sm_norm

    @staticmethod
    def _signed_dist(pi: tuple[float, float], pj: tuple[float, float], c:  tuple[float, float]) -> float:
        num = (pj[0]-pi[0])*(c[1]-pi[1]) - (pj[1]-pi[1])*(c[0]-pi[0])
        den = math.hypot(pj[0]-pi[0], pj[1]-pi[1])
        return num / den if den > 1e-9 else 0.0

    @staticmethod
    def _ensure_ccw(pts: List[tuple[float, float]]) -> tuple[tuple, tuple, tuple]:
        a, b, c = pts
        cross = (b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1])
        return (a, b, c) if cross > 0 else (a, c, b)

    def _cb_mode(self, msg: String) -> None:
        with self._lock: self._current_mode = msg.data

    def _cb_bb_red(self, msg: Float32MultiArray) -> None:
        with self._lock: self._bb_red_area = float(msg.data[1]) if len(msg.data) >= 2 else 0.0

    def _cb_bb_blue(self, msg: Float32MultiArray) -> None:
        with self._lock: self._bb_blue_area = float(msg.data[1]) if len(msg.data) >= 2 else 0.0

    def _cb_bb_green(self, msg: Float32MultiArray) -> None:
        with self._lock: self._bb_green_area = float(msg.data[1]) if len(msg.data) >= 2 else 0.0

    def get_sim_time_s(self) -> float:
        with self._lock: return self._sim_time_ns * 1e-9

    def snapshot(self) -> Dict[str, Any]:
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

def _kill_process_group(proc: subprocess.Popen, grace_s: float) -> None:
    try:
        pgid = os.getpgid(proc.pid)
        log.info(f'Enviando SIGINT a PGID={pgid}')
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        pass

    deadline = time.monotonic() + grace_s
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            break
        time.sleep(0.2)

    for pattern in ('gz sim', 'ros2 launch', 'robot_state_publisher', 'ign', 'ruby'):
        subprocess.run(['pkill', '-9', '-f', pattern], capture_output=True, check=False)
    
    time.sleep(2.0)

    if proc.poll() is None:
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.kill()


class TrialEvaluator:
    def __init__(self, suite_name: str, timeout_s: float) -> None:
        self.suite_name = suite_name
        self.timeout_s  = timeout_s
        self._success_hold_start: Optional[float] = None  
        self._mode_switched: bool = False
        self._post_switch_start: Optional[float] = None   
        self._tipover_start_s: Optional[float] = None
        self.NEURONOFFSET = 12

        self._x17_window = deque(maxlen=20) 
        self._success_hold_start = None


        # Flags para la captura de teclado manual
        self._manual_success_requested = False
        self._manual_restart_requested = False

        # Iniciar el escuchador de teclado en segundo plano (non-blocking)
        self._keyboard_listener = keyboard.Listener(on_press=self._on_key_press)
        self._keyboard_listener.start()



    def _on_key_press(self, key):
        try:
            # Captura caracteres en minúscula o mayúscula
            if hasattr(key, 'char') and key.char is not None:
                char = key.char.lower()
                if char == 's':
                    log.info('[KEYBOARD] Tecla "S" presionada -> Solicitando SUCCESS.')
                    self._manual_success_requested = True
                elif char == 'r':
                    log.info('[KEYBOARD] Tecla "R" presionada -> Solicitando RESTART.')
                    self._manual_restart_requested = True
        except Exception as e:
            log.error(f'Error en captura de teclado: {e}')

    def evaluate(self, snap: Dict[str, Any], sim_start_s: float, warmup_done: bool) -> Optional[Verdict]:
        if not warmup_done: return None
        current_sim_s: float = snap['sim_time_s']
        elapsed: float = current_sim_s - sim_start_s

        if elapsed >= self.timeout_s:
            log.warning(f'[TIMEOUT] {elapsed:.1f}s >= {self.timeout_s}s')
            return Verdict.FAILURE_TIMEOUT

        # --- LÓGICA DE TIPOVER CON HISTÉRESIS TEMPORAL ---
        if snap['tr'] >= CRIT_TR:
            if self._tipover_start_s is None:
                self._tipover_start_s = current_sim_s
            elif (current_sim_s - self._tipover_start_s) >= TIPOVER_HOLD_S:
                log.warning(f'[TIPOVER] TR={snap["tr"]:.3f} >= {CRIT_TR} por {TIPOVER_HOLD_S}s continuos')
                return Verdict.FAILURE_TIPOVER
        else:
            self._tipover_start_s = None  # Resetea si el robot recupera el equilibrio
        # -------------------------------------------------

        if self.suite_name in SUITE_APPETITIVE: return self._eval_appetitive(snap, current_sim_s)
        if self.suite_name in SUITE_EVASIVE: return self._eval_evasive(snap, current_sim_s)
        if self.suite_name in SUITE_TERRAIN_C1: return self._eval_terrain_c1(snap, current_sim_s)
        if self.suite_name in SUITE_TERRAIN_C2: return self._eval_terrain_c2(snap, current_sim_s)
        return None

    def _eval_appetitive(self, snap: Dict[str, Any], current_sim_s: float) -> Optional[Verdict]:
        neurons = snap['neurons']
        x17_raw = neurons[17 + self.NEURONOFFSET] if len(neurons) > 17+self.NEURONOFFSET else 0.0
        
        self._x17_window.append(x17_raw)
        x17_avg = sum(self._x17_window) / len(self._x17_window)
        
        if x17_avg > X17_THRESHOLD:
            if self._success_hold_start is None:
                self._success_hold_start = current_sim_s
            elif (current_sim_s - self._success_hold_start) >= SUCCESS_HOLD_S:
                log.info(f'[SUCCESS] Objetivo alcanzado! X17={x17_avg:.3f}')
                return Verdict.SUCCESS
        elif x17_avg < 0.1:
            self._success_hold_start = None  
        return None

    def _eval_evasive(self, snap: Dict[str, Any], current_sim_s: float) -> Optional[Verdict]:
        neurons = snap['neurons']
        x14_active = (neurons[14 + self.NEURONOFFSET] > 0.3) if len(neurons) > 14 + self.NEURONOFFSET else False
        mode_switched = x14_active or (snap['mode'] in ('H', 'C'))

        l4_active = neurons[34] if len(neurons) > 34 else False
        if (l4_active*15) > 0.2:
            log.info("Condicion activada")

        
        # ── CAMBIO: Se ajustan las llaves a los nombres correctos del snapshot ──
        stimulus_gone = (snap['bb_red'] < 1.0) and (snap['bb_green'] < 1.0)
        
        tr_safe       = snap['tr'] < CRIT_TR
        
        # También eliminamos tresponse_ok, por la misma razón que en la apetitiva
        # (es una métrica exclusiva de la Familia C de terreno rugoso).
        if mode_switched and stimulus_gone and tr_safe:
            if self._success_hold_start is None:
                self._success_hold_start = current_sim_s
            elif (current_sim_s - self._success_hold_start) >= SUCCESS_HOLD_S:
                log.info(f'[SUCCESS] Evasión completada exitosamente.')
                return Verdict.SUCCESS
        else:
            self._success_hold_start = None
        return None

    # def _eval_terrain_c1(self, snap: Dict[str, Any], current_sim_s: float) -> Optional[Verdict]:
    #     neurons = snap['neurons']
    #     x0 = neurons[0 + self.NEURONOFFSET] if len(neurons) > 0 + self.NEURONOFFSET else 0.0
    #     x15 = neurons[15 + self.NEURONOFFSET] if len(neurons) > 15 + self.NEURONOFFSET else 0.0
    #     tswitch_done = snap['tswitch'] > 0.0
    #     mode_articulated = (snap['mode'] == 'C') # El tópico /peter_mode pasa a 'C' en terreno irregular

    #     # Evaluamos el cumplimiento de la condición de disparo (Activación de red o modo motor)
    #     if not self._mode_switched and (x0 > 0.1 or x15 > 0.1 or mode_articulated) and tswitch_done:
    #         self._mode_switched     = True
    #         self._post_switch_start = current_sim_s
    #         log.info(f'[C1] Transition to articulated gait detected at {current_sim_s:.2f}s. Evaluating stability window...')

    #     # Verificación de la ventana de estabilidad obligatoria (SAFETY_STABLE_S = 12.0 segundos)
    #     if self._mode_switched and self._post_switch_start is not None:
    #         if (current_sim_s - self._post_switch_start) >= SAFETY_STABLE_S:
    #             log.info(f'[SUCCESS] C1 test successful: 12 seconds of stable articulated locomotion completed.')
    #             return Verdict.SUCCESS
    #     return None

    def _eval_terrain_c1(self, snap: Dict[str, Any], current_sim_s: float) -> Optional[Verdict]:
        if self._manual_restart_requested:
            self._manual_restart_requested = False  # Reset del flag
            log.info(f'[MANUAL RESTART] Solicitud de reinicio manual recibida en {current_sim_s:.2f}s.')
            # Retorna el veredicto correspondiente a reinicio en tu framework
            # (Ej: Verdict.REBOOT, Verdict.RESTART, Verdict.FAILURE según tu Enum)
            return Verdict.FAILURE_TIPOVER

        if self._manual_success_requested:
            self._manual_success_requested = False  # Reset del flag
            log.info(f'[SUCCESS] C1 test marcado exitosamente por entrada manual (Tecla S) en {current_sim_s:.2f}s.')
            return Verdict.SUCCESS

        return None

    def _eval_terrain_c2(self, snap: Dict[str, Any], current_sim_s: float) -> Optional[Verdict]:
        mode = snap['mode']
        
        # 1. Detectar el paso por modo Cuadrúpedo (reacción inicial al estímulo aversivo)
        if not hasattr(self, '_c2_saw_quadruped'):
            self._c2_saw_quadruped = False
            
        if not self._c2_saw_quadruped and mode == 'C':
            self._c2_saw_quadruped = True
            log.info(f'[C2] Modo cuadrúpedo (evasión) detectado a los {current_sim_s:.2f}s.')
            
        # 2. Detectar la transición de vuelta a Híbrido (subiendo la pendiente hacia atrás)
        if self._c2_saw_quadruped and not self._mode_switched and mode == 'H':
            self._mode_switched = True
            self._post_switch_start = current_sim_s
            log.info(f'[C2] Pendiente detectada. Cambio a modo Híbrido a los {current_sim_s:.2f}s. Evaluando ascenso...')
            
        # 3. Validar que mantenga el ascenso en modo Híbrido estable
        if self._mode_switched and self._post_switch_start is not None:
            if mode != 'H':
                # Si el robot resbala y pierde la postura, reiniciamos la ventana de evaluación
                self._mode_switched = False
                self._post_switch_start = None
                log.info(f'[C2] Se perdió el modo Híbrido de ascenso. Reiniciando ventana de estabilidad.')
            elif (current_sim_s - self._post_switch_start) >= SAFETY_STABLE_S//3:  # Requiere la mitad del tiempo de estabilidad que la Familia C1, dado lo desafiante de la tarea
                log.info(f'[SUCCESS] Prueba C2 exitosa: Ascenso en modo Híbrido mantenido por {SAFETY_STABLE_S//3}s.')
                return Verdict.SUCCESS
                
        return None

def _collect_artifacts(trial_dir: Path, suite_name: str, trial_idx: int, verdict: Verdict, seed_info: Dict[str, Any], snap_final: Dict[str, Any]) -> None:
    trial_dir.mkdir(parents=True, exist_ok=True)
    if STABILITY_LOG_SRC.exists(): shutil.move(str(STABILITY_LOG_SRC), str(trial_dir / 'stability_log.csv'))
    
    unified_metrics_src = Path.home() / 'unified_metrics.csv'
    if unified_metrics_src.exists(): shutil.move(str(unified_metrics_src), str(trial_dir / 'unified_metrics.csv'))
    
    exp_base = Path.home() / 'peter_experiments'
    if exp_base.exists():
        candidates = sorted(exp_base.iterdir(), key=lambda p: p.stat().st_mtime, reverse=True)
        for candidate in candidates:
            csv_src = candidate / 'metrics.csv'
            if csv_src.exists():
                shutil.move(str(csv_src), str(trial_dir / 'metrics_raw.csv'))
                try: candidate.rmdir()
                except OSError: pass
                break

    summary = {
        'trial_index': trial_idx,
        'suite_name':  suite_name,
        'verdict':     verdict.value,
        'seed_info':   seed_info,
        'final_metrics': { k: (round(v, 4) if isinstance(v, float) else v) for k, v in snap_final.items() if k not in ('neurons',) }
    }
    with open(trial_dir / 'trial_summary.json', 'w') as f: json.dump(summary, f, indent=2)

def _update_manifest(manifest_path: Path, trial_idx: int, seed_info: Dict[str, Any], verdict: Verdict) -> None:
    data: List[Dict] = []
    if manifest_path.exists():
        with open(manifest_path) as f: data = json.load(f)
    data.append({'trial_index': trial_idx, 'verdict': verdict.value, **seed_info})
    with open(manifest_path, 'w') as f: json.dump(data, f, indent=2)

def _build_launch_args(suite_cfg: Dict[str, Any], trial_idx: int, noise_idx: int, rng: np.random.Generator) -> tuple[List[str], Dict[str, Any]]:
    dp: Dict[str, Any] = suite_cfg.get('dynamic_parameters', {})
    args: List[str] = []
    seed_info: Dict[str, Any] = {'trial_index': trial_idx, 'noise_level_idx': noise_idx, 'perturbations': {}}
    random_perturb: bool = dp.get('random_perturbation', False)
    sigma: float = float(dp.get('variance_sigma', 0.15))

    # ── Lógica para illum_direction (0-7 -> -1, 8-15 -> 1) ──
    # Si las pruebas empiezan en 1 (por ejemplo 1-8 y 9-16), ajusta (trial_idx - 1)
    if trial_idx < 8: illum_dir = -1
    else: illum_dir = 1

    args += [f'illum_direction:={illum_dir}']
    seed_info['illum_direction'] = illum_dir

    def _perturbed(base_key: str) -> tuple[str, float]:
        base_val = float(dp.get(base_key, 0.0))
        if random_perturb:
            delta = float(rng.normal(0.0, sigma))
            seed_info['perturbations'][base_key] = {'base': base_val, 'delta': round(delta, 4)}
            return str(round(base_val + delta, 4)), base_val + delta
        return str(base_val), base_val

    if 'world_name' in dp: args += [f'world_name:={dp["world_name"]}']
    if 'stimulus_type' in dp: args += [f'stimulus_type:={dp["stimulus_type"]}']
    if 'stimulus_x' in dp: args += [f'stimulus_x:={_perturbed("stimulus_x")[0]}']
    if 'stimulus_y' in dp: args += [f'stimulus_y:={_perturbed("stimulus_y")[0]}']
    for flag in ('spawn_green', 'spawn_red', 'spawn_blue'):
        if flag in dp: args += [f'{flag}:={str(dp[flag]).lower()}']
    for coord in ('green_x', 'green_y', 'red_x', 'red_y', 'blue_x', 'blue_y'):
        if coord in dp: args += [f'{coord}:={_perturbed(coord)[0] if random_perturb else str(dp[coord])}']
    
    args += [f'noise_level_idx:={noise_idx}']
    return args, seed_info


class TestManager:
    def __init__(self, config_path: str) -> None:
        with open(config_path) as f: self._cfg: Dict[str, Any] = yaml.safe_load(f)
        gs = self._cfg.get('global_settings', {})
        self._workspace: Path = Path(os.path.expanduser(gs.get('workspace_path', '~/PETER_SIMULATION/ros2_ws')))
        self._output_base: Path = Path(os.path.expanduser(gs.get('output_base_dir', '~/PETER_SIMULATION/Findings')))
        self._grace_period: float = float(gs.get('grace_period_teardown_s', GRACE_PERIOD_S))
        self._ros_setup  = f'source /opt/ros/humble/setup.bash && source {self._workspace}/install/setup.bash'
        self._rng        = np.random.default_rng()
        self._judge_node: Optional[TestJudgeNode] = None
        self._ros_thread: Optional[threading.Thread] = None

    def _start_ros_thread(self) -> None:
        if not rclpy.ok(): rclpy.init()
        self._judge_node = TestJudgeNode()
        self._ros_thread = threading.Thread(target=lambda: rclpy.spin(self._judge_node), daemon=True)
        self._ros_thread.start()

    def _stop_ros_thread(self) -> None:
        if self._judge_node is not None: self._judge_node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        if self._ros_thread is not None: self._ros_thread.join(timeout=5.0)

    def _launch_simulation(self, launch_file: str, extra_args: List[str]) -> subprocess.Popen:
        cmd_str = f'{self._ros_setup} && ros2 launch peter_robot {launch_file} ' + ' '.join(extra_args) + ' > ~/sim_output.log 2>&1'
        return subprocess.Popen(cmd_str, shell=True, executable='/bin/bash', preexec_fn=os.setsid)

    def _run_trial(self, suite_cfg: Dict[str, Any], trial_idx: int, noise_idx: int) -> tuple[Verdict, Dict[str, Any], subprocess.Popen]:
        dp = suite_cfg.get('dynamic_parameters', {})
        timeout_s: float = float(dp.get('timeout_s', 60.0))
        suite_name: str  = suite_cfg['suite_name']
        launch_args, seed_info = _build_launch_args(suite_cfg, trial_idx, noise_idx, self._rng)
        seed_info['noise_level_idx'] = noise_idx

        proc = self._launch_simulation(suite_cfg['launch_file'], launch_args)
        warmup_start_wall = time.monotonic()
        while time.monotonic() - warmup_start_wall < WARMUP_S:
            if proc.poll() is not None: return Verdict.FAILURE_CRASH, seed_info, proc
            time.sleep(0.2)

        evaluator = TrialEvaluator(suite_name=suite_name, timeout_s=timeout_s)
        sim_start_s: float = 0.0
        for _ in range(50):
            candidate = self._judge_node.get_sim_time_s()
            if candidate > 0.0:
                sim_start_s = candidate
                break
            time.sleep(0.1)

        verdict: Optional[Verdict] = None
        while verdict is None:
            if proc.poll() is not None:
                verdict = Verdict.FAILURE_CRASH
                break
            snap = self._judge_node.snapshot()
            verdict = evaluator.evaluate(snap, sim_start_s, warmup_done=True)
            time.sleep(0.05)

        time.sleep(1.0)
        _kill_process_group(proc, self._grace_period)
        return verdict, seed_info, proc

    def _teardown(self, suite_name: str, trial_idx: int, verdict: Verdict, seed_info: Dict[str, Any], snap_final: Dict[str, Any]) -> None:
        trial_label = f'test_{trial_idx:03d}_{verdict.value}'
        trial_dir   = self._output_base / suite_name / trial_label
        _collect_artifacts(trial_dir, suite_name, trial_idx, verdict, seed_info, snap_final)
        manifest_path = self._output_base / suite_name / 'suite_execution_manifest.json'
        _update_manifest(manifest_path, trial_idx, seed_info, verdict)

    def run(self) -> None:
        self._start_ros_thread()
        noise_levels: List[int] = [0, 1, 2, 3, 4] 

        try:
            for suite_cfg in self._cfg.get('experiment_suites', []):
                suite_name  = suite_cfg['suite_name']
                repetitions = int(suite_cfg.get('repetitions', 1))
                (self._output_base / suite_name).mkdir(parents=True, exist_ok=True)

                # --- Lógica de Reanudación Automática ---
                manifest_path = self._output_base / suite_name / 'suite_execution_manifest.json'
                successful_trials = set()
                if manifest_path.exists():
                    try:
                        with open(manifest_path) as f:
                            for entry in json.load(f):
                                if entry.get('verdict') == 'SUCCESS':
                                    successful_trials.add(entry.get('trial_index'))
                    except Exception: pass

                trial_global_idx = 0
                for rep in range(repetitions):
                    noise_idx = noise_levels[rep % len(noise_levels)]
                    trial_global_idx += 1

                    if trial_global_idx in successful_trials:
                        log.info(f'\n[TRIAL {trial_global_idx:03d}/{repetitions}] OMITIDO (Éxito previo registrado)')
                        continue

                    log.info(f'\n[TRIAL {trial_global_idx:03d}/{repetitions}] suite={suite_name} noise_idx={noise_idx}')
                    verdict, seed_info, proc = self._run_trial(suite_cfg, trial_global_idx, noise_idx)

                    if proc.poll() is None:
                        try: proc.wait(timeout=5.0)
                        except subprocess.TimeoutExpired: proc.kill()

                    snap_final = self._judge_node.snapshot()
                    self._teardown(suite_name, trial_global_idx, verdict, seed_info, snap_final)
                    time.sleep(5.0)

        except KeyboardInterrupt:
            pass
        finally:
            self._stop_ros_thread()

def main() -> None:
    config_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.expanduser('/ros2_ws'), 'src', 'peter_robot', 'config', 'experiments_config.yaml')
    TestManager(config_path).run()

if __name__ == '__main__':
    main()
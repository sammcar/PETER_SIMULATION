"""
gait_v2.py — Puerto Python adaptado al patrón "Y Acostada" (Peter Controller)
════════════════════════════════════════════════════════════════════
Replica la lógica de peter_controller.py incluyendo su matemática
exacta para los objetivos de giro (TURN_X, TURN_Y).
"""

import numpy as np
import math
from ver_def.robot_kinematics import SpiderQuadruped

# ── Constantes de dirección ───────────────────────────────────────────
GAIT_STOP      = 'STOP'
GAIT_FORWARD   = 'FORWARD'
GAIT_BACKWARD  = 'BACKWARD'
GAIT_LEFT      = 'LEFT'
GAIT_RIGHT     = 'RIGHT'
GAIT_FWD_LEFT  = 'FWD_LEFT'
GAIT_FWD_RIGHT = 'FWD_RIGHT'
GAIT_BCK_LEFT  = 'BCK_LEFT'
GAIT_BCK_RIGHT = 'BCK_RIGHT'

LEG_NAMES = ['FL', 'FR', 'RL', 'RR']

PHASE_IDLE   = 'IDLE'
PHASE_LIFT   = 'LIFT'
PHASE_MOVE   = 'MOVE'
PHASE_LOWER  = 'LOWER'
PHASE_BODY   = 'BODY'
PHASE_RETURN = 'RETURN'

_STATE_TABLE = [
    (PHASE_LIFT,  'FL', [ 0,  0,  0,  0]),  
    (PHASE_MOVE,  'FL', [ 2,  0,  0,  0]),  
    (PHASE_LOWER, 'FL', [ 2,  0,  0,  0]),  
    (PHASE_BODY,  'RR', [ 1, -1, -1, -1]),  
    (PHASE_LIFT,  'RR', [ 1, -1, -1, -1]),  
    (PHASE_MOVE,  'RR', [ 1, -1, -1,  1]),  
    (PHASE_LOWER, 'RR', [ 1, -1, -1,  1]),  
    (PHASE_LIFT,  'FR', [ 1, -1, -1,  1]),  
    (PHASE_MOVE,  'FR', [ 1,  1, -1,  1]),  
    (PHASE_LOWER, 'FR', [ 1,  1, -1,  1]),  
    (PHASE_BODY,  'RL', [ 0,  0, -2,  0]),  
    (PHASE_LIFT,  'RL', [ 0,  0, -2,  0]),  
    (PHASE_MOVE,  'RL', [ 0,  0,  0,  0]),  
    (PHASE_LOWER, 'RL', [ 0,  0,  0,  0]),  
]

_STATE_TABLE_BWD = [
    (PHASE_LIFT,  'RL', [ 0,  0,  0,  0]),  
    (PHASE_MOVE,  'RL', [ 0,  0,  2,  0]),  
    (PHASE_LOWER, 'RL', [ 0,  0,  2,  0]),  
    (PHASE_BODY,  'FR', [-1, -1,  1, -1]),  
    (PHASE_LIFT,  'FR', [-1, -1,  1, -1]),  
    (PHASE_MOVE,  'FR', [-1,  1,  1, -1]),  
    (PHASE_LOWER, 'FR', [-1,  1,  1, -1]),  
    (PHASE_LIFT,  'RR', [-1,  1,  1, -1]),  
    (PHASE_MOVE,  'RR', [-1,  1,  1,  1]),  
    (PHASE_LOWER, 'RR', [-1,  1,  1,  1]),  
    (PHASE_BODY,  'FL', [-2,  0,  0,  0]),  
    (PHASE_LIFT,  'FL', [-2,  0,  0,  0]),  
    (PHASE_MOVE,  'FL', [ 0,  0,  0,  0]),  
    (PHASE_LOWER, 'FL', [ 0,  0,  0,  0]),  
]

_BWD_DIRS = {GAIT_BACKWARD, GAIT_BCK_LEFT, GAIT_BCK_RIGHT}

GAIT_TURN_LEFT  = 'TURN_LEFT'
GAIT_TURN_RIGHT = 'TURN_RIGHT'

_TURN_DIRS = {GAIT_TURN_LEFT, GAIT_TURN_RIGHT}

# Secuencia de 4 pasos de prueba
_STATE_TABLE_TURN_R = [
    (PHASE_LIFT,  'FL'),  # 0: Levantar FL
    (PHASE_BODY,  'FL'),  # 1: Acomodar cuerpo a coordenadas exactas peter_controller
    (PHASE_MOVE,  'FL'),  # 2: Mover FL a su coordenada target peter_controller
    (PHASE_LOWER, 'FL'),  # 3: Bajar FL y detener
]

_STATE_TABLE_TURN_L = [
    (PHASE_LIFT,  'RL', False),
    (PHASE_MOVE,  'RL', True),
    (PHASE_LOWER, 'RL', True),
    (PHASE_LIFT,  'RR', True),
    (PHASE_MOVE,  'RR', False),
    (PHASE_LOWER, 'RR', False),
    (PHASE_LIFT,  'FR', False),
    (PHASE_MOVE,  'FR', True),
    (PHASE_LOWER, 'FR', True),
    (PHASE_LIFT,  'FL', True),
    (PHASE_MOVE,  'FL', False),
    (PHASE_LOWER, 'FL', False),
]

class GaitV2:
    def __init__(self, robot: SpiderQuadruped,
                 step_len:     float = 0.040,
                 step_len_lat: float = 0.020,
                 step_h:       float = 0.035,
                 leg_speed:    float = 0.006,
                 body_speed:   float = 0.003,
                 reach_tol:    float = 0.002,
                 turn_step:    float = 0.20):
        self.robot        = robot
        self.step_len     = step_len
        self.step_len_lat = step_len_lat
        self.step_h       = step_h
        self.leg_speed    = leg_speed
        self.body_speed   = body_speed
        self.reach_tol    = reach_tol
        self.turn_step    = turn_step

        self._site_now    = np.zeros((4, 3))
        self._site_expect = np.zeros((4, 3))
        self._site_rest   = np.zeros((4, 3))

        self._phase         = PHASE_IDLE
        self._current_swing = 'FL'

        self._dir           = GAIT_STOP
        self._next_dir      = GAIT_STOP

        self._step_dx       = 0.0
        self._step_dy       = 0.0
        self._state_idx     = 13  

        self.body_pos       = np.zeros(3)
        self.body_yaw       = 0.0        
        self._stable_pose   = 'LEFT_Y'  
        self._last_was_turn = False      
        
        # Calcula constantes geométricas de Peter Controller
        self._calc_peter_turn_constants()
        self._gait_init()

    def _calc_peter_turn_constants(self):
        """Replica exacta de la matemática de peter_controller.py pero en metros."""
        L_A = 0.045
        L_C = 0.0685
        L_SIDE = 0.120  # Distancia lateral base_link
        DIAG = math.sqrt(((L_A + L_C) ** 2) / 2)

        T_A = math.sqrt((2 * DIAG + L_SIDE) ** 2 + DIAG ** 2)
        T_B = 2 * DIAG + L_SIDE
        T_C = math.sqrt((2 * DIAG + L_SIDE) ** 2 + (DIAG + L_SIDE) ** 2)
        T_ALPHA = math.acos((T_A ** 2 + T_B ** 2 - T_C ** 2) / (2 * T_A * T_B))

        self.TURN_X1 = (T_A - L_SIDE) / 2
        self.TURN_Y1 = DIAG / 2
        self.TURN_X0 = self.TURN_X1 - T_B * math.cos(T_ALPHA)
        self.TURN_Y0 = T_B * math.sin(T_ALPHA) - self.TURN_Y1 - L_SIDE

    def _get_peter_target(self, name, local_x, local_y, z):
        """Convierte coordenadas locales de Peter a Frame del Cuerpo según _foot_body_xy()"""
        hx, hy, _ = self.robot.legs[name].body_offset
        # Según peter_controller: Y es "Forward" y X es "Outward"
        if name == 'FR':
            return np.array([hx + local_y, hy - local_x, z])
        elif name == 'FL':
            return np.array([hx + local_y, hy + local_x, z])
        elif name == 'RR':
            return np.array([hx - local_y, hy - local_x, z])
        elif name == 'RL':
            return np.array([hx - local_y, hy + local_x, z])

    def _gait_init(self):
        for i, name in enumerate(LEG_NAMES):
            rest = self.robot.legs[name].get_rest_foot_body()
            self._site_rest[i]   = rest.copy()
            self._site_now[i]    = rest.copy()
            self._site_expect[i] = rest.copy()

        distancia_x_geometrica = abs(self._site_rest[1][0] - self._site_rest[0][0])
        self.step_len = distancia_x_geometrica

        self._state_idx = 13  
        self._apply_ik_all()
        self._phase = PHASE_IDLE

    def reset(self):
        self._phase         = PHASE_IDLE
        self._current_swing = 'FL'
        self._dir           = GAIT_STOP
        self._next_dir      = GAIT_STOP
        self._state_idx     = 13
        self.body_pos       = np.zeros(3)
        self.body_yaw       = 0.0
        self._stable_pose   = 'LEFT_Y'
        self._last_was_turn = False
        self._gait_init()

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def current_swing(self) -> str:
        return self._current_swing

    @property
    def direction(self) -> str:
        return self._next_dir

    def set_dir(self, direction: str):
        self._next_dir = direction

        if self._phase == PHASE_IDLE and direction != GAIT_STOP:
            self._dir    = direction
            if direction == GAIT_TURN_RIGHT:
                self._state_idx = 0
            else:
                is_turn      = direction in _TURN_DIRS
                at_right_y   = (self._stable_pose == 'RIGHT_Y')

                if at_right_y and self._last_was_turn and is_turn:
                    self._state_idx = 6   
                elif at_right_y and not self._last_was_turn and not is_turn:
                    self._state_idx = 7   
                else:
                    self._state_idx = 0
            self._update_state_machine()

    def update(self) -> dict:
        if self._phase == PHASE_IDLE:
            return self._foot_dict()

        if self._phase == PHASE_RETURN:
            prev = self._site_now.copy()
            done = self._tick_toward_expect(self.body_speed)
            self._accumulate_body(prev)
            if done:
                self._phase = PHASE_IDLE
            self._apply_ik_all()
            return self._foot_dict()

        is_turn = self._dir in _TURN_DIRS

        if is_turn:
            speed = self.body_speed if self._phase in (PHASE_BODY, PHASE_MOVE) else self.leg_speed
        else:
            speed = self.body_speed if self._phase == PHASE_BODY else self.leg_speed

        prev = self._site_now.copy()
        done = self._tick_toward_expect(speed)

        if not is_turn and self._phase == PHASE_BODY:
            self._accumulate_body(prev)
        elif is_turn and self._phase in (PHASE_BODY, PHASE_MOVE):
            self._accumulate_turn(prev)

        if done:
            # Forzar parada inmediata al completar la secuencia de prueba de giro derecho
            if self._dir == GAIT_TURN_RIGHT and self._state_idx == 3:
                self._stable_pose   = 'RIGHT_Y'
                self._last_was_turn = True
                self._dir           = GAIT_STOP
                self._phase         = PHASE_IDLE
                self._apply_ik_all()
                return self._foot_dict()

            right_cp = 5 if is_turn else 6
            left_cp  = 11 if is_turn else 13

            if self._state_idx == right_cp:
                self._stable_pose   = 'RIGHT_Y'
                self._last_was_turn = is_turn
                new_dir = self._next_dir

                if new_dir in _TURN_DIRS and not is_turn:
                    self._state_idx = right_cp + 1
                    self._update_state_machine()
                elif is_turn and new_dir not in _TURN_DIRS:
                    self._dir = new_dir
                    if self._dir == GAIT_STOP:
                        self._phase = PHASE_IDLE
                    else:
                        self._state_idx = 0
                        self._update_state_machine()
                else:
                    self._dir = new_dir
                    if self._dir == GAIT_STOP:
                        self._phase = PHASE_IDLE
                    else:
                        self._state_idx = right_cp + 1
                        self._update_state_machine()

            elif self._state_idx == left_cp:
                self._stable_pose   = 'LEFT_Y'
                self._last_was_turn = is_turn
                self._dir = self._next_dir
                if self._dir == GAIT_STOP:
                    self._phase = PHASE_IDLE
                else:
                    self._state_idx = 0
                    self._update_state_machine()

            else:
                self._state_idx += 1
                self._update_state_machine()

        self._apply_ik_all()
        return self._foot_dict()

    def _update_state_machine(self):
        is_turn = self._dir in _TURN_DIRS

        if is_turn:
            if self._dir == GAIT_TURN_RIGHT:
                phase, swing_leg = _STATE_TABLE_TURN_R[self._state_idx]
                self._phase         = phase
                self._current_swing = swing_leg

                rz = {l_idx: self._site_rest[l_idx][2] for l_idx in range(4)}

                # Calcular los objetivos exactos de peter_controller para este giro
                peter_targets = {}
                for l_idx, name in enumerate(LEG_NAMES):
                    if name in ('FL', 'FR'): # Patas delanteras
                        peter_targets[name] = self._get_peter_target(name, self.TURN_X0, self.TURN_Y0, rz[l_idx])
                    else:                    # Patas traseras
                        peter_targets[name] = self._get_peter_target(name, self.TURN_X1, self.TURN_Y1, rz[l_idx])

                for l_idx, name in enumerate(LEG_NAMES):
                    if self._state_idx == 0:    # PHASE_LIFT FL
                        tgt = self._site_now[l_idx].copy()
                        if name == 'FL':
                            tgt[2] += self.step_h

                    elif self._state_idx == 1:  # PHASE_BODY
                        if name == 'FL':        # FL se queda quieta en el aire
                            tgt = self._site_now[l_idx].copy()
                        else:                   # FR, RL, RR se mueven a coordenadas Peter
                            tgt = peter_targets[name].copy()

                    elif self._state_idx == 2:  # PHASE_MOVE
                        if name == 'FL':        # FL viaja en el aire a coordenada Peter
                            tgt = peter_targets[name].copy()
                            tgt[2] = rz[l_idx] + self.step_h
                        else:                   # El resto se queda quieto anclado
                            tgt = self._site_now[l_idx].copy()

                    elif self._state_idx == 3:  # PHASE_LOWER
                        if name == 'FL':        # FL baja al suelo
                            tgt = peter_targets[name].copy()
                        else:
                            tgt = self._site_now[l_idx].copy()

                    self._site_expect[l_idx] = tgt
            else:
                # GAIT_TURN_LEFT (Mantiene comportamiento original/placeholder)
                rot = -self.turn_step
                table = _STATE_TABLE_TURN_L

                phase, swing_leg, use_rotated = table[self._state_idx]
                self._phase         = phase
                self._current_swing = swing_leg

                rz = {l_idx: self._site_rest[l_idx][2] for l_idx in range(4)}

                for l_idx, name in enumerate(LEG_NAMES):
                    if phase == PHASE_MOVE:
                        if use_rotated:
                            tgt = self._rotate_xy(self._site_rest[l_idx], rot)
                        else:
                            tgt = self._site_rest[l_idx].copy()
                        tgt[2] = rz[l_idx] + self.step_h if name == swing_leg else rz[l_idx]
                    else:
                        tgt = self._site_now[l_idx].copy()
                        if name == swing_leg:
                            tgt[2] = rz[l_idx] + self.step_h if phase == PHASE_LIFT else rz[l_idx]
                    self._site_expect[l_idx] = tgt

        else:
            # Marcha lineal (adelante / atrás / lateral) sin cambios
            if self._state_idx in (0, 7):
                self._step_dx, self._step_dy = self._dir_to_step(self._dir)

            table = _STATE_TABLE_BWD if self._dir in _BWD_DIRS else _STATE_TABLE
            phase, swing_leg, offsets = table[self._state_idx]
            self._phase         = phase
            self._current_swing = swing_leg

            for l_idx, name in enumerate(LEG_NAMES):
                rx, ry, rz = self._site_rest[l_idx]
                mult     = offsets[l_idx]
                target_x = rx + mult * self._step_dx
                target_y = ry + mult * self._step_dy
                target_z = rz
                if name == swing_leg and phase in (PHASE_LIFT, PHASE_MOVE):
                    target_z += self.step_h
                self._site_expect[l_idx] = [target_x, target_y, target_z]

    def _rotate_xy(self, pos: np.ndarray, angle: float) -> np.ndarray:
        c, s = np.cos(angle), np.sin(angle)
        x, y, z = pos[0], pos[1], pos[2]
        return np.array([c*x - s*y, s*x + c*y, z])

    def _dir_to_step(self, direction: str):
        sl  = self.step_len
        sll = self.step_len_lat
        d   = sl * 0.707
        return {
            GAIT_FORWARD:   ( sl,  0.0),
            GAIT_BACKWARD:  (-sl,  0.0),
            GAIT_LEFT:      (0.0,   sll),
            GAIT_RIGHT:     (0.0,  -sll),
            GAIT_FWD_LEFT:  ( d,    d),
            GAIT_FWD_RIGHT: ( d,   -d),
            GAIT_BCK_LEFT:  (-d,    d),
            GAIT_BCK_RIGHT: (-d,   -d),
        }.get(direction, (0.0, 0.0))

    def _tick_toward_expect(self, speed: float) -> bool:
        all_done = True
        for l in range(4):
            for j in range(3):
                diff = self._site_expect[l, j] - self._site_now[l, j]
                if abs(diff) > self.reach_tol:
                    all_done = False
                    step = min(abs(diff), speed)
                    self._site_now[l, j] += step if diff > 0 else -step
                else:
                    self._site_now[l, j] = self._site_expect[l, j]
        return all_done

    def _apply_ik_all(self):
        for i, name in enumerate(LEG_NAMES):
            self.robot.legs[name].ik(self._site_now[i])

    def _foot_dict(self) -> dict:
        return {LEG_NAMES[i]: self._site_now[i].copy() for i in range(4)}

    def _accumulate_body(self, prev: np.ndarray):
        delta = self._site_now - prev
        mean_delta = delta.mean(axis=0)
        self.body_pos -= mean_delta
        self.body_pos[2] = 0.0

    def _accumulate_turn(self, prev: np.ndarray):
        angles = []
        for i, name in enumerate(LEG_NAMES):
            if name == self._current_swing:
                continue
            px, py = prev[i, 0], prev[i, 1]
            cx, cy = self._site_now[i, 0], self._site_now[i, 1]
            if px * px + py * py < 1e-8:
                continue
            angles.append(np.arctan2(px * cy - py * cx,
                                     px * cx + py * cy))
        if angles:
            self.body_yaw -= np.mean(angles)

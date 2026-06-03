"""
gait_v2.py — Puerto Python adaptado al patrón "Y Acostada" (Peter Controller)
════════════════════════════════════════════════════════════════════
Replica la lógica de peter_controller.py mediante una máquina de
estados de offset (14 pasos) que cierra el ciclo matemáticamente y
es 100% compatible con la interfaz de peter_simu_v7.py.
"""

import numpy as np
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

# ── Fases Originales (Para compatibilidad con el simulador) ───────────
PHASE_IDLE   = 'IDLE'
PHASE_LIFT   = 'LIFT'
PHASE_MOVE   = 'MOVE'
PHASE_LOWER  = 'LOWER'
PHASE_BODY   = 'BODY'
PHASE_RETURN = 'RETURN'

# ── Tabla de Estados: (Fase, Pata_Activa, Offsets_XY[FL, FR, RL, RR]) ──
# Los offsets son multiplicadores relativos a (dx, dy). Esto garantiza
# que al finalizar el ciclo las 4 patas regresen a un offset relativo 0.
_STATE_TABLE = [
    (PHASE_LIFT,  'FL', [ 0,  0,  0,  0]),  # 0: Inicia FL
    (PHASE_MOVE,  'FL', [ 2,  0,  0,  0]),  # 1: Avanza FL (+2)
    (PHASE_LOWER, 'FL', [ 2,  0,  0,  0]),  # 2: Baja FL
    (PHASE_BODY,  'RR', [ 1, -1, -1, -1]),  # 3: Chasis avanza (patas -1). Cambio a RR para contar paso
    (PHASE_LIFT,  'RR', [ 1, -1, -1, -1]),  # 4: Inicia RR
    (PHASE_MOVE,  'RR', [ 1, -1, -1,  1]),  # 5: Avanza RR (+2)
    (PHASE_LOWER, 'RR', [ 1, -1, -1,  1]),  # 6: Baja RR
    (PHASE_LIFT,  'FR', [ 1, -1, -1,  1]),  # 7: Inicia FR
    (PHASE_MOVE,  'FR', [ 1,  1, -1,  1]),  # 8: Avanza FR (+2)
    (PHASE_LOWER, 'FR', [ 1,  1, -1,  1]),  # 9: Baja FR
    (PHASE_BODY,  'RL', [ 0,  0, -2,  0]),  # 10: Chasis avanza (patas -1). Cambio a RL para contar paso
    (PHASE_LIFT,  'RL', [ 0,  0, -2,  0]),  # 11: Inicia RL
    (PHASE_MOVE,  'RL', [ 0,  0,  0,  0]),  # 12: Avanza RL (+2)
    (PHASE_LOWER, 'RL', [ 0,  0,  0,  0]),  # 13: Baja RL (Ciclo cerrado perfecto a 0)
]


class GaitV2:
    def __init__(self, robot: SpiderQuadruped,
                 step_len:     float = 0.040,
                 step_len_lat: float = 0.020,
                 step_h:       float = 0.035,
                 leg_speed:    float = 0.006,
                 body_speed:   float = 0.003,
                 reach_tol:    float = 0.002):
        self.robot        = robot
        self.step_len     = step_len
        self.step_len_lat = step_len_lat
        self.step_h       = step_h
        self.leg_speed    = leg_speed
        self.body_speed   = body_speed
        self.reach_tol    = reach_tol

        self._site_now    = np.zeros((4, 3))
        self._site_expect = np.zeros((4, 3))
        self._site_rest   = np.zeros((4, 3))

        self._phase         = PHASE_IDLE
        self._current_swing = 'FL'
        self._dir           = GAIT_STOP
        self._step_dx       = 0.0
        self._step_dy       = 0.0
        self._state_idx     = 0

        self.body_pos = np.zeros(3)
        self._gait_init()

    def _gait_init(self):
        """Captura la pose inicial definida (Y acostada) en site_rest."""
        for i, name in enumerate(LEG_NAMES):
            rest = self.robot.legs[name].get_rest_foot_body()
            self._site_rest[i]   = rest.copy()
            self._site_now[i]    = rest.copy()
            self._site_expect[i] = rest.copy()

        self._apply_ik_all()
        self._phase = PHASE_IDLE

    def reset(self):
        self._phase         = PHASE_IDLE
        self._current_swing = 'FL'
        self._dir           = GAIT_STOP
        self._step_dx       = 0.0
        self._step_dy       = 0.0
        self._state_idx     = 0
        self.body_pos       = np.zeros(3)
        self._gait_init()

    # ── Interfaces expuestas para peter_simu_v7.py ──────────────────

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def current_swing(self) -> str:
        return self._current_swing

    @property
    def direction(self) -> str:
        return self._dir

    # ── Actualización de comandos ─────────────────────────────────────

    def set_dir(self, direction: str):
        if direction == self._dir:
            return
        self._dir = direction

        if direction == GAIT_STOP:
            if self._phase != PHASE_IDLE:
                self._phase = PHASE_RETURN
                self._site_expect[:] = self._site_rest[:]
            return

        # Si estaba en reposo, inicia la tabla de secuencia en el paso 0
        if self._phase in (PHASE_IDLE, PHASE_RETURN):
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

        # Ejecución activa de las fases
        prev = self._site_now.copy()
        speed = self.body_speed if self._phase == PHASE_BODY else self.leg_speed
        done = self._tick_toward_expect(speed)

        if self._phase == PHASE_BODY:
            self._accumulate_body(prev)

        if done:
            if self._dir == GAIT_STOP:
                self._phase = PHASE_RETURN
                self._site_expect[:] = self._site_rest[:]
            else:
                self._state_idx = (self._state_idx + 1) % len(_STATE_TABLE)
                self._update_state_machine()

        self._apply_ik_all()
        return self._foot_dict()

    # ── Máquina de estados interna y geometría ────────────────────────

    def _update_state_machine(self):
        """Calcula los targets absolutos basándose en la tabla lógica."""
        
        # Solo actualizar (dx, dy) al inicio de un medio ciclo (evita saltos bruscos
        # si cambias de dirección a mitad de la caminata)
        if self._state_idx in (0, 7):
            self._step_dx, self._step_dy = self._dir_to_step(self._dir)

        phase, swing_leg, offsets = _STATE_TABLE[self._state_idx]
        self._phase = phase
        self._current_swing = swing_leg

        for l_idx, name in enumerate(LEG_NAMES):
            rx, ry, rz = self._site_rest[l_idx]
            
            # Aplicar el multiplicador de offset correspondiente a esta fase
            mult = offsets[l_idx]
            target_x = rx + mult * self._step_dx
            target_y = ry + mult * self._step_dy
            target_z = rz
            
            # Subir el Z solo si es la pata activa y está levantada o moviéndose
            if name == swing_leg and phase in (PHASE_LIFT, PHASE_MOVE):
                target_z += self.step_h

            self._site_expect[l_idx] = [target_x, target_y, target_z]

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
        """
        En la fase BODY o RETURN todas las patas actúan como apoyo.
        El desplazamiento del cuerpo relativo a tierra es el inverso al de las patas.
        """
        delta = self._site_now - prev
        mean_delta = delta.mean(axis=0)
        self.body_pos -= mean_delta
        self.body_pos[2] = 0.0

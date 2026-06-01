"""
gait_v2.py — Puerto Python de gait.cpp (versión máquina de estados)
════════════════════════════════════════════════════════════════════
Replica la lógica de gait.cpp iteración por iteración:
  • Misma máquina de estados: IDLE → LIFT → MOVE → LOWER → BODY → RETURN
  • tick_toward_expect() a velocidad constante por iteración (sin dt real)
  • Parámetros por defecto = config.h

Diferencias con robot_gait.py (viejo simulador):
  • Sin interpolación temporal (no usa step_duration ni dt)
  • Sin body_shift de equilibrio (eliminado en la nueva versión del Arduino)
  • Añade strafe LEFT/RIGHT y diagonales (igual que el Arduino)

Cada llamada a update() equivale a una iteración de loop() en el Arduino.
"""

import numpy as np
from ver_def.robot_kinematics import SpiderQuadruped

# ── Constantes de dirección (= GaitDir en gait.h) ─────────────────────
GAIT_STOP      = 'STOP'
GAIT_FORWARD   = 'FORWARD'
GAIT_BACKWARD  = 'BACKWARD'
GAIT_LEFT      = 'LEFT'
GAIT_RIGHT     = 'RIGHT'
GAIT_FWD_LEFT  = 'FWD_LEFT'
GAIT_FWD_RIGHT = 'FWD_RIGHT'
GAIT_BCK_LEFT  = 'BCK_LEFT'
GAIT_BCK_RIGHT = 'BCK_RIGHT'

# Orden diagonal-wave: FL → RR → FR → RL  (= SWING_ORDER en gait.cpp)
LEG_NAMES   = ['FL', 'FR', 'RL', 'RR']   # índice 0-3
SWING_ORDER = [0, 3, 1, 2]               # FL=0  RR=3  FR=1  RL=2

# Fases (= Phase enum en gait.cpp)
PHASE_IDLE   = 'IDLE'
PHASE_LIFT   = 'LIFT'
PHASE_MOVE   = 'MOVE'
PHASE_LOWER  = 'LOWER'
PHASE_BODY   = 'BODY'
PHASE_RETURN = 'RETURN'


class GaitV2:
    """
    Marcha crawl diagonal-wave — puerto de gait.cpp.

    Parámetros por defecto coinciden con config.h del Arduino.
    La velocidad visual depende del frame-rate de la animación:
    con 30 fps y leg_speed=0.006 un paso completo dura ~22 frames (~0.73 s).
    """

    def __init__(self, robot: SpiderQuadruped,
                 step_len:     float = 0.040,   # STEP_LEN
                 step_len_lat: float = 0.020,   # STEP_LEN_LAT
                 step_h:       float = 0.035,   # STEP_H
                 leg_speed:    float = 0.006,   # LEG_SPEED
                 body_speed:   float = 0.003,   # BODY_SPEED
                 reach_tol:    float = 0.002):  # REACH_TOL
        self.robot        = robot
        self.step_len     = step_len
        self.step_len_lat = step_len_lat
        self.step_h       = step_h
        self.leg_speed    = leg_speed
        self.body_speed   = body_speed
        self.reach_tol    = reach_tol

        # site_now / site_expect / site_rest — shape (4, 3), frame cuerpo
        self._site_now    = np.zeros((4, 3))
        self._site_expect = np.zeros((4, 3))
        self._site_rest   = np.zeros((4, 3))

        self._phase     = PHASE_IDLE
        self._swing_idx = 0           # índice 0-3 en SWING_ORDER
        self._dir       = GAIT_STOP
        self._step_dx   = 0.0
        self._step_dy   = 0.0

        # Posición acumulada del cuerpo en frame mundo (solo para visualización).
        # Se actualiza durante BODY y RETURN usando el desplazamiento de las patas.
        self.body_pos = np.zeros(3)

        self._gait_init()

    # ── gait_init() ───────────────────────────────────────────────────────

    def _gait_init(self):
        """Lleva todas las patas a posición de reposo y aplica IK."""
        for i, name in enumerate(LEG_NAMES):
            rest = self.robot.legs[name].get_rest_foot_body()
            self._site_rest[i]   = rest
            self._site_now[i]    = rest.copy()
            self._site_expect[i] = rest.copy()
        self._apply_ik_all()
        self._phase = PHASE_IDLE

    def reset(self):
        """Reinicia el estado completo a reposo."""
        self._phase     = PHASE_IDLE
        self._swing_idx = 0
        self._dir       = GAIT_STOP
        self._step_dx   = 0.0
        self._step_dy   = 0.0
        self.body_pos   = np.zeros(3)
        self._gait_init()

    # ── Propiedades de estado (para visualización) ─────────────────────

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def current_swing(self) -> str:
        return LEG_NAMES[SWING_ORDER[self._swing_idx]]

    @property
    def direction(self) -> str:
        return self._dir

    # ── gait_set_dir() ────────────────────────────────────────────────

    def set_dir(self, direction: str):
        if direction == self._dir:
            return
        self._dir = direction

        if direction == GAIT_STOP:
            if self._phase != PHASE_IDLE:
                self._phase = PHASE_RETURN
                self._expect_all_rest()
            return

        # Arrancar desde reposo o cancelar retorno en curso
        if self._phase in (PHASE_IDLE, PHASE_RETURN):
            self._swing_idx = 0
            self._freeze_stance_legs()
            self._phase = PHASE_LIFT
            self._start_lift()
        # Si ya marcha, la nueva dirección toma efecto en el próximo start_lift()

    # ── gait_update() ────────────────────────────────────────────────

    def update(self) -> dict:
        """
        Una iteración del loop() del Arduino.
        Aplica IK al robot y retorna {name: np.array([x,y,z])} en frame cuerpo.
        """
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

        # ── Fases de balanceo ───────────────────────────────────────
        if self._phase == PHASE_LIFT:
            if self._tick_toward_expect(self.leg_speed):
                self._phase = PHASE_MOVE
                self._start_move()

        elif self._phase == PHASE_MOVE:
            if self._tick_toward_expect(self.leg_speed):
                self._phase = PHASE_LOWER
                self._start_lower()

        elif self._phase == PHASE_LOWER:
            if self._tick_toward_expect(self.leg_speed):
                self._phase = PHASE_BODY
                self._expect_all_stride_back(self._step_dx, self._step_dy)

        elif self._phase == PHASE_BODY:
            prev = self._site_now.copy()
            done = self._tick_toward_expect(self.body_speed)
            self._accumulate_body(prev)
            if done:
                if self._dir == GAIT_STOP:
                    self._phase = PHASE_RETURN
                    self._expect_all_rest()
                else:
                    self._swing_idx = (self._swing_idx + 1) % 4
                    self._phase = PHASE_LIFT
                    self._start_lift()

        self._apply_ik_all()
        return self._foot_dict()

    # ── Helpers — traducción directa de gait.cpp ──────────────────────

    def _dir_to_step(self, direction: str):
        """dir_to_step() — retorna (dx, dy) en metros."""
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
        """tick_toward_expect() — retorna True cuando todas las patas llegaron."""
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

    def _set_site_one(self, leg: int, x: float, y: float, z: float):
        self._site_expect[leg] = [x, y, z]

    def _expect_all_rest(self):
        self._site_expect[:] = self._site_rest

    def _expect_all_stride_back(self, dx: float, dy: float):
        """Todas las patas al apoyo posterior: rest - step/2."""
        for l in range(4):
            self._site_expect[l, 0] = self._site_rest[l, 0] - dx * 0.5
            self._site_expect[l, 1] = self._site_rest[l, 1] - dy * 0.5
            self._site_expect[l, 2] = self._site_rest[l, 2]

    def _freeze_stance_legs(self):
        """Congela los expects de las patas en apoyo a su posición actual."""
        for l in range(4):
            if l != SWING_ORDER[self._swing_idx]:
                self._site_expect[l] = self._site_now[l].copy()

    def _start_lift(self):
        sl = SWING_ORDER[self._swing_idx]
        self._step_dx, self._step_dy = self._dir_to_step(self._dir)
        # Solo sube Z; XY quedan donde está
        self._set_site_one(sl,
            self._site_now[sl, 0],
            self._site_now[sl, 1],
            self._site_rest[sl, 2] + self.step_h)

    def _start_move(self):
        sl = SWING_ORDER[self._swing_idx]
        # Avanza a rest + medio paso, a altura levantada
        self._set_site_one(sl,
            self._site_rest[sl, 0] + self._step_dx * 0.5,
            self._site_rest[sl, 1] + self._step_dy * 0.5,
            self._site_rest[sl, 2] + self.step_h)

    def _start_lower(self):
        sl = SWING_ORDER[self._swing_idx]
        # Baja al suelo en la posición delantera
        self._set_site_one(sl,
            self._site_rest[sl, 0] + self._step_dx * 0.5,
            self._site_rest[sl, 1] + self._step_dy * 0.5,
            self._site_rest[sl, 2])

    def _apply_ik_all(self):
        """apply_ik_all() — llama a IK de Python en lugar de mover()."""
        for i, name in enumerate(LEG_NAMES):
            self.robot.legs[name].ik(self._site_now[i])

    def _foot_dict(self) -> dict:
        return {LEG_NAMES[i]: self._site_now[i].copy() for i in range(4)}

    def _accumulate_body(self, prev: np.ndarray):
        """
        Aproxima la posición del cuerpo en frame mundo para visualización.
        El cuerpo avanza cuando los pies en apoyo retroceden.
        Solo actualiza durante BODY y RETURN (cuando pies tocan el suelo).
        """
        delta = self._site_now - prev          # desplazamiento de cada pata
        sw    = SWING_ORDER[self._swing_idx]
        # Solo las patas en apoyo (no la que acaba de aterrizar en LIFT/MOVE/LOWER)
        stance = [i for i in range(4) if i != sw]
        if stance:
            mean_delta        = delta[stance].mean(axis=0)
            self.body_pos    -= mean_delta       # cuerpo va en sentido opuesto
            self.body_pos[2]  = 0.0             # mantener altura 0 (suelo plano)

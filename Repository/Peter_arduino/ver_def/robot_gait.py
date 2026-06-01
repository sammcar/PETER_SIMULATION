"""
robot_gait.py — Controlador de marcha crawl con dirección
══════════════════════════════════════════════════════════
PORTABLE A ESP32 / C++

Marcha crawl estática: un pie en swing, tres en stance.
Secuencia fija: FL → RR → FR → RL → (repite)

Pipeline de control por ciclo (dt = tiempo entre llamadas):
    foot_body = gait.update(dt)          # posiciones en frame cuerpo
    for each leg: leg.ik(foot_body[leg]) # cinemática inversa
    send_to_servos(leg.q)                # ángulos → PWM

En C++ / ESP32:
    CrawlGait se convierte en variables globales + función update(float dt).
    foot_body[4][3] = array de floats por pata.
    Todas las matrices rot_z son operaciones inline (no heap).

Constantes de dirección:
    DIR_FORWARD / DIR_BACKWARD / DIR_TURN_LEFT / DIR_TURN_RIGHT / DIR_STOP
    En C++: typedef enum { DIR_FWD=0, DIR_BWD, DIR_LEFT, DIR_RIGHT, DIR_STOP } Direction;
"""

import numpy as np

from ver_def.robot_math import rot_z
from ver_def.robot_kinematics import SpiderQuadruped


# ── Constantes ────────────────────────────────────────────────────────

CRAWL_SEQUENCE = ['FL', 'RR', 'FR', 'RL']   # orden de swing

DIR_FORWARD    = 'ADELANTE'
DIR_BACKWARD   = 'ATRAS'
DIR_TURN_LEFT  = 'GIRAR_IZQ'
DIR_TURN_RIGHT = 'GIRAR_DER'
DIR_STOP       = 'PARAR'


# ──────────────────────────────────────────────────────────────────────
#  TRAYECTORIA DE SWING
# ──────────────────────────────────────────────────────────────────────

class FootTrajectory:
    """
    Trayectoria parabólica de un pie durante el swing.

    XY : interpolación lineal start → end.
    Z  : parábola aditiva (pico = height en t = 0.5).

    C++ equivalente (inline, sin heap):
        // t normalizado [0..1]
        float pos[3];
        for (int i=0; i<3; i++) pos[i] = start[i] + t*(end[i]-start[i]);
        pos[2] += 4.0f * height * t * (1.0f - t);
    """

    def __init__(self, start: np.ndarray, end: np.ndarray, height: float):
        self.start  = np.array(start, dtype=float)
        self.end    = np.array(end,   dtype=float)
        self.height = float(height)

    def eval(self, t: float) -> np.ndarray:
        t   = float(np.clip(t, 0.0, 1.0))
        pos = self.start + t * (self.end - self.start)
        pos[2] += 4.0 * self.height * t * (1.0 - t)
        return pos


# ──────────────────────────────────────────────────────────────────────
#  CONTROLADOR DE MARCHA CRAWL
# ──────────────────────────────────────────────────────────────────────

class CrawlGait:
    """
    Marcha crawl con soporte de dirección y regreso seguro a posición home.

    Cambio de dirección seguro
    ──────────────────────────
    set_direction() encola el cambio en pending_direction.
    Se aplica únicamente en el límite entre pasos (zona estable),
    cuando los 3 pies de stance están plantados y el swing ha terminado.
    Esto evita pérdidas de equilibrio a mitad de un paso.

    Secuencia de parada (DIR_STOP)
    ──────────────────────────────
    Al recibir STOP se activan _homing_steps = 4. Cada una de las 4 patas
    hace su swing de vuelta a la posición nominal de reposo. Solo entonces
    _stopped = True y el robot queda congelado en postura simétrica.

    API pública (idéntica en Python y C++)
    ──────────────────────────────────────
        gait.set_direction(DIR_*)      # encolar cambio de dirección
        foot_body = gait.update(dt)    # avanzar simulación, obtener posiciones
        gait.reset()                   # volver al estado inicial
    """

    def __init__(self, robot: SpiderQuadruped,
                 step_length:   float = 0.06,
                 step_height:   float = 0.03,
                 step_duration: float = 0.50,
                 yaw_per_step:  float = 0.15):
        """
        Args:
            step_length   : distancia de avance por paso [m]
            step_height   : altura máxima del pie en swing [m]
            step_duration : duración de cada paso [s]
            yaw_per_step  : ángulo de giro por paso [rad] (~8.6° = 0.15 rad)
        """
        self.robot         = robot
        self.step_length   = step_length
        self.step_height   = step_height
        self.step_duration = step_duration
        self.yaw_per_step  = yaw_per_step
        self.body_vel      = step_length / step_duration       # m/s en traslación
        self.angular_vel   = yaw_per_step / step_duration      # rad/s en giro

        # Estado del cuerpo en frame mundo
        self.body_pos = np.zeros(3)
        self.body_yaw = 0.0

        # Dirección activa y cambio pendiente
        self.direction         = DIR_FORWARD
        self.pending_direction = None
        self._stopped          = False
        self._homing_steps     = 0

        # Posiciones iniciales de los pies (frame mundo)
        robot.rest_pose()
        all_fk = robot.get_all_fk()
        self.foot_world = {n: fk['foot'].copy() for n, fk in all_fk.items()}

        # Estado del ciclo
        self.swing_idx = 0
        self.t_phase   = 0.0
        self._start_new_swing()

    # ── Propiedad de conveniencia ──────────────────────────────────────

    @property
    def current_swing(self) -> str:
        return CRAWL_SEQUENCE[self.swing_idx]

    # ── API pública ────────────────────────────────────────────────────

    def set_direction(self, direction: str):
        """
        Encola un cambio de dirección.
        Se aplica en la próxima zona estable para no interrumpir un swing.

        C++:  gait.pending_dir = direction;
        """
        self.pending_direction = direction

    def update(self, dt: float) -> dict:
        """
        Avanza la simulación dt segundos.

        Retorna foot_body: {name: np.array([x,y,z])} en FRAME CUERPO,
        listo para pasar directamente a SpiderLeg.ik().

        C++:
            void gait_update(float dt, float foot_body[4][3]) { ... }
        """
        R_inv = rot_z(-self.body_yaw)

        # ── Detenido: esperar nueva dirección ────────────────────────
        if self._stopped:
            if self.pending_direction is not None:
                self.direction         = self.pending_direction
                self.pending_direction = None
                self._stopped          = False
                self._start_new_swing()
            else:
                return {n: R_inv @ (self.foot_world[n] - self.body_pos)
                        for n in self.robot.legs}

        # ── Mover cuerpo según dirección activa ──────────────────────
        R = rot_z(self.body_yaw)
        if self.direction == DIR_FORWARD:
            self.body_pos += R @ np.array([self.body_vel * dt, 0.0, 0.0])
        elif self.direction == DIR_BACKWARD:
            self.body_pos += R @ np.array([-self.body_vel * dt, 0.0, 0.0])
        elif self.direction == DIR_TURN_LEFT:
            self.body_yaw += self.angular_vel * dt
        elif self.direction == DIR_TURN_RIGHT:
            self.body_yaw -= self.angular_vel * dt

        self.t_phase += dt

        # ── Zona estable: límite entre pasos ─────────────────────────
        while self.t_phase >= self.step_duration:
            # Aterrizar el pie actual en su posición final exacta
            self.foot_world[self.current_swing] = self.trajectory.eval(1.0).copy()
            self.t_phase  -= self.step_duration
            self.swing_idx = (self.swing_idx + 1) % len(CRAWL_SEQUENCE)

            # Aplicar cambio de dirección pendiente
            if self.pending_direction is not None:
                self.direction         = self.pending_direction
                self.pending_direction = None
                if self.direction == DIR_STOP:
                    # Activar regreso a home: 4 pasos para llevar cada pata a reposo
                    self._homing_steps = len(CRAWL_SEQUENCE)

            # Decidir siguiente swing o detener
            if self.direction == DIR_STOP:
                if self._homing_steps > 0:
                    self._homing_steps -= 1
                    self._start_new_swing()   # pata va a posición nominal (reposo)
                else:
                    self._stopped = True
                    self.t_phase  = 0.0
                    break
            else:
                self._start_new_swing()

        # Si acaba de detenerse, devolver posiciones fijas
        if self._stopped:
            R_inv = rot_z(-self.body_yaw)
            return {n: R_inv @ (self.foot_world[n] - self.body_pos)
                    for n in self.robot.legs}

        # ── Posiciones de los 4 pies en frame cuerpo ─────────────────
        t_norm      = self.t_phase / self.step_duration
        swing_pos_w = self.trajectory.eval(t_norm)

        R_inv     = rot_z(-self.body_yaw)
        foot_body = {}
        for name in self.robot.legs:
            fw = swing_pos_w if name == self.current_swing else self.foot_world[name]
            foot_body[name] = R_inv @ (fw - self.body_pos)
        return foot_body

    def reset(self):
        """Reinicia la marcha a la postura de reposo y posición cero."""
        self.body_pos          = np.zeros(3)
        self.body_yaw          = 0.0
        self.swing_idx         = 0
        self.t_phase           = 0.0
        self.direction         = DIR_FORWARD
        self.pending_direction = None
        self._stopped          = False
        self._homing_steps     = 0
        self.robot.rest_pose()
        all_fk = self.robot.get_all_fk()
        self.foot_world = {n: fk['foot'].copy() for n, fk in all_fk.items()}
        self._start_new_swing()

    # ── Helpers internos ──────────────────────────────────────────────

    def _nominal_foot_world(self, name: str, bp: np.ndarray,
                             yaw: float) -> np.ndarray:
        """Posición nominal del pie en frame mundo dado body_pos=bp y body_yaw=yaw."""
        return rot_z(yaw) @ self.robot.legs[name].get_rest_foot_body() + bp

    def _start_new_swing(self):
        """Inicializa la trayectoria para el pie que entra en swing."""
        name  = self.current_swing
        start = self.foot_world[name].copy()
        R     = rot_z(self.body_yaw)

        if self.direction == DIR_FORWARD:
            body_at_land = self.body_pos + R @ np.array([self.step_length, 0., 0.])
            yaw_at_land  = self.body_yaw
        elif self.direction == DIR_BACKWARD:
            body_at_land = self.body_pos + R @ np.array([-self.step_length, 0., 0.])
            yaw_at_land  = self.body_yaw
        elif self.direction == DIR_TURN_LEFT:
            body_at_land = self.body_pos.copy()
            yaw_at_land  = self.body_yaw + self.yaw_per_step
        elif self.direction == DIR_TURN_RIGHT:
            body_at_land = self.body_pos.copy()
            yaw_at_land  = self.body_yaw - self.yaw_per_step
        else:   # DIR_STOP — pie va a posición nominal actual (reposo)
            body_at_land = self.body_pos.copy()
            yaw_at_land  = self.body_yaw

        end    = self._nominal_foot_world(name, body_at_land, yaw_at_land)
        end[2] = start[2]   # suelo plano

        self.trajectory = FootTrajectory(start, end, self.step_height)

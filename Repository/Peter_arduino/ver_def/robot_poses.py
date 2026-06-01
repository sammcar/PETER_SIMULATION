"""
robot_poses.py — Poses y rutinas de transición del robot
═════════════════════════════════════════════════════════
PORTABLE A ESP32 / C++

Define poses nominales como keyframes de posición de pies (frame cuerpo)
y un reproductor de rutinas que interpola entre ellas con smooth-step.

Pipeline de uso (idéntico en Python y C++):
    steps  = routine_lie_down(robot)
    player.start(current_foot_pose(robot), steps)
    while not player.done:
        player.update(dt)
        foot_body = player.current_pose()
        for name, pos in foot_body.items():
            robot.legs[name].ik(pos)

C++ / ESP32:
    PoseTransition_t: from[4][3], to[4][3], duration, elapsed, done
    RoutinePlayer_t:  array de pasos, índice actual, transición en curso
    Smooth-step inline: float ts = t*t*(3-2*t);
"""

import numpy as np
from typing import Optional

from ver_def.robot_kinematics import SpiderQuadruped


# ── Captura de pose actual ─────────────────────────────────────────────

def current_foot_pose(robot: SpiderQuadruped) -> dict:
    """Captura las posiciones FK actuales de los cuatro pies (frame cuerpo)."""
    return {name: fk['foot'].copy() for name, fk in robot.get_all_fk().items()}


# ── Constructor de poses ───────────────────────────────────────────────

def _build_pose(robot: SpiderQuadruped,
                dz: float, xy_scale: float = 1.0) -> dict:
    """
    Construye una pose desplazando cada pie desde su posición de reposo.

    dz        : delta Z en frame cuerpo (+dz = pies más arriba = cuerpo más bajo)
    xy_scale  : escala del radio XY respecto al origen de la pata (>1 = más abierto)

    C++: iterar sobre las 4 patas; rest = leg_get_rest_foot_body(leg);
         rest[2] += dz;
         float rel[2] = {rest[0]-offset[0], rest[1]-offset[1]};
         rest[0] = offset[0] + rel[0]*xy_scale;
         rest[1] = offset[1] + rel[1]*xy_scale;
    """
    pose = {}
    for name, leg in robot.legs.items():
        p = leg.get_rest_foot_body().copy()
        p[2] += dz
        if xy_scale != 1.0:
            rel    = p[:2] - leg.body_offset[:2]
            p[:2]  = leg.body_offset[:2] + rel * xy_scale
        pose[name] = p
    return pose


# ── Poses estándar ─────────────────────────────────────────────────────

def pose_stand(robot: SpiderQuadruped) -> dict:
    """Postura normal erguida (posición de reposo nominal)."""
    return _build_pose(robot, dz=0.0, xy_scale=1.0)

def pose_crouch(robot: SpiderQuadruped) -> dict:
    """Postura agachada intermedia (cuerpo semibajo, patas semidobladas)."""
    return _build_pose(robot, dz=+0.055, xy_scale=1.10)

def pose_lie(robot: SpiderQuadruped) -> dict:
    """Postura acostada (cuerpo casi a nivel del suelo, patas abiertas)."""
    return _build_pose(robot, dz=+0.088, xy_scale=1.22)


# ──────────────────────────────────────────────────────────────────────
#  TRANSICIÓN ENTRE DOS POSES
# ──────────────────────────────────────────────────────────────────────

class PoseTransition:
    """
    Interpola suavemente entre dos poses con función smooth-step (3t²-2t³).

    Proporciona aceleración/desaceleración natural sin parámetros extra.

    C++ equivalente:
        float t  = elapsed / duration;
        float ts = t*t*(3.0f - 2.0f*t);
        for (int i=0; i<4; i++)
            for (int j=0; j<3; j++)
                foot[i][j] = from[i][j] + ts*(to[i][j]-from[i][j]);
    """

    def __init__(self, from_pose: dict, to_pose: dict, duration: float):
        self.from_pose = {n: np.array(p, dtype=float) for n, p in from_pose.items()}
        self.to_pose   = {n: np.array(p, dtype=float) for n, p in to_pose.items()}
        self.duration  = max(float(duration), 1e-6)
        self.elapsed   = 0.0
        self.done      = False

    def update(self, dt: float):
        if self.done:
            return
        self.elapsed = min(self.elapsed + dt, self.duration)
        if self.elapsed >= self.duration:
            self.done = True

    def eval(self) -> dict:
        t  = self.elapsed / self.duration
        ts = t * t * (3.0 - 2.0 * t)          # smooth-step
        return {
            name: self.from_pose[name] + ts * (self.to_pose[name] - self.from_pose[name])
            for name in self.from_pose
        }


# ──────────────────────────────────────────────────────────────────────
#  REPRODUCTOR DE RUTINAS
# ──────────────────────────────────────────────────────────────────────

class RoutinePlayer:
    """
    Ejecuta una secuencia de keyframes (pose, duración) como rutina continua.

    Cada par (pose_destino, duración_s) se convierte en una PoseTransition.
    El player avanza automáticamente de un keyframe al siguiente.

    API:
        player.start(from_pose, steps)  — inicia la rutina
        player.update(dt)               — avanza la simulación dt segundos
        player.current_pose()           — posiciones interpoladas actuales
        player.active                   — True mientras ejecuta
        player.done                     — True al terminar el último paso

    C++:
        State machine con índice de paso. Al avanzar de paso, crear nueva
        PoseTransition_t con to_pose del paso anterior como from_pose.
    """

    def __init__(self):
        self._steps: list                     = []
        self._idx:   int                      = 0
        self._trans: Optional[PoseTransition] = None
        self.active: bool                     = False
        self.done:   bool                     = False

    def start(self, from_pose: dict, steps: list):
        """
        Inicia la rutina.

        from_pose : pose de partida (dict {name: np.array([x,y,z])})
        steps     : lista de (target_pose: dict, duration_s: float)
        """
        if not steps:
            return
        self._steps = list(steps)
        self._idx   = 0
        self.active = True
        self.done   = False
        target, dur = self._steps[0]
        self._trans = PoseTransition(from_pose, target, dur)

    def update(self, dt: float):
        if not self.active or self.done:
            return
        self._trans.update(dt)
        if self._trans.done:
            self._idx += 1
            if self._idx >= len(self._steps):
                self.done   = True
                self.active = False
            else:
                prev_pose    = self._trans.to_pose
                target, dur  = self._steps[self._idx]
                self._trans  = PoseTransition(prev_pose, target, dur)

    def current_pose(self) -> Optional[dict]:
        """Devuelve las posiciones interpoladas actuales, o None si no hay rutina."""
        return self._trans.eval() if self._trans is not None else None

    def reset(self):
        """Cancela cualquier rutina en curso."""
        self._steps = []
        self._idx   = 0
        self._trans = None
        self.active = False
        self.done   = False


# ── Rutinas predefinidas ────────────────────────────────────────────────

def routine_lie_down(robot: SpiderQuadruped) -> list:
    """
    Rutina acostarse: de pie → agachado → acostado.
    Retorna lista de (pose, duración) lista para pasar a RoutinePlayer.start().
    """
    return [
        (pose_crouch(robot), 0.70),
        (pose_lie(robot),    0.85),
    ]

def routine_stand_up(robot: SpiderQuadruped) -> list:
    """
    Rutina levantarse: acostado → agachado → de pie.
    Retorna lista de (pose, duración) lista para pasar a RoutinePlayer.start().
    """
    return [
        (pose_crouch(robot), 0.85),
        (pose_stand(robot),  0.70),
    ]

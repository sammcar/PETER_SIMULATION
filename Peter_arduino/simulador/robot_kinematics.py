"""
robot_kinematics.py — Cinemática inversa y directa del robot cuadrúpedo
════════════════════════════════════════════════════════════════════════
PORTABLE A ESP32 / C++

Estructura de cada pata (3 GDL):
    [CUERPO] → Coxa (yaw) → Fémur (pitch) → Tibia (pitch) → [PIE]

Convención ejes (frame del cuerpo):
    +X → Frente   +Y → Izquierda   +Z → Arriba

Disposición patas (vista superior):
    FL ──────── FR
    │  CUERPO   │
    RL ──────── RR

Para portar ik() a C++, reemplazar:
    np.arctan2(y, x)    →  atan2f(y, x)
    np.sqrt(v)          →  sqrtf(v)
    np.arccos(v)        →  acosf(v)
    np.clip(v, -1, 1)   →  fmaxf(-1.f, fminf(1.f, v))
    np.radians(d)       →  (d * M_PI / 180.f)
"""

import numpy as np
from typing import Optional

from simulador.robot_params import RobotParams


# ──────────────────────────────────────────────────────────────────────
#  CINEMÁTICA DE UNA PATA
# ──────────────────────────────────────────────────────────────────────

class SpiderLeg:
    """
    Cinemática de una pata en configuración araña.

    El frame de la pata tiene su eje X apuntando en la dirección de
    extensión radial del cuerpo; Z es vertical. Todos los ángulos se
    calculan en este frame local y luego se transforman al frame del
    cuerpo para visualización o envío a servos.

    ESP32 / C++:
        Mantener R_body2leg y body_offset como arrays globales por pata.
        ik() y fk() son funciones puras sin efectos secundarios salvo
        actualizar q[3] (ángulos articulares).
    """

    def __init__(self, name: str, body_offset: np.ndarray,
                 coxa_angle_body: float, params: RobotParams):
        self.name            = name
        self.body_offset     = np.array(body_offset, dtype=float)
        self.coxa_angle_body = coxa_angle_body
        self.p               = params

        c, s = np.cos(coxa_angle_body), np.sin(coxa_angle_body)
        self.R_body2leg = np.array([[ c,  s, 0],
                                    [-s,  c, 0],
                                    [ 0,  0, 1]], dtype=float)
        self.R_leg2body = self.R_body2leg.T

        self.q = np.zeros(3)   # [theta_coxa, theta_femur, theta_tibia] rad

        # Posición de reposo individual (frame pata). Copiados de params por
        # defecto; sobreescribir por pata para poses asimétricas (ej. modo C).
        self.rest_x: float = params.rest_x
        self.rest_y: float = params.rest_y
        self.rest_z: float = params.rest_z

    # ── Cinemática Inversa ────────────────────────────────────────────

    def ik(self, foot_pos_body: np.ndarray) -> Optional[np.ndarray]:
        """
        Calcula ángulos articulares dado el objetivo del pie en FRAME CUERPO.

        Retorna [theta_coxa, theta_femur, theta_tibia] en rad, o None si
        el punto está fuera de alcance o viola límites articulares.

        C++ equivalente — ver robot_kinematics.cpp::leg_ik().
        """
        # 1. Transformar al frame de la pata
        p_leg  = self.R_body2leg @ (foot_pos_body - self.body_offset)
        px, py, pz = p_leg

        # 2. Coxa (yaw en plano XY)
        theta_coxa   = np.arctan2(py, px)
        r_horizontal = np.sqrt(px**2 + py**2)
        reach        = r_horizontal - self.p.L_coxa
        D            = np.sqrt(reach**2 + pz**2)

        # 3. Verificar alcanzabilidad
        L1, L2 = self.p.L_femur, self.p.L_tibia
        if D > (L1 + L2) or D < abs(L1 - L2) or D < 1e-6:
            return None

        # 4. Tibia (ley del coseno)
        cos_tibia   = np.clip((D**2 - L1**2 - L2**2) / (2*L1*L2), -1.0, 1.0)
        theta_tibia = -np.arccos(cos_tibia)   # rodilla abajo

        # 5. Fémur
        alpha       = np.arctan2(pz, reach)
        beta        = np.arctan2(L2 * np.sin(theta_tibia),
                                 L1 + L2 * np.cos(theta_tibia))
        theta_femur = alpha - beta

        angles = np.array([theta_coxa, theta_femur, theta_tibia])

        # 6. Verificar límites articulares
        for i, (lo, hi) in enumerate([self.p.coxa_lim,
                                       self.p.femur_lim,
                                       self.p.tibia_lim]):
            if not (np.radians(lo) <= angles[i] <= np.radians(hi)):
                return None

        self.q = angles
        return angles

    # ── Cinemática Directa ────────────────────────────────────────────

    def fk(self, q: Optional[np.ndarray] = None) -> dict:
        """
        Posiciones de todas las articulaciones en FRAME CUERPO.

        Retorna {'coxa', 'femur', 'tibia', 'foot'} como np.array [x,y,z].
        En ESP32 se usa principalmente para debug; en operación solo ik().
        """
        if q is None:
            q = self.q
        theta_coxa, theta_femur, theta_tibia = q

        radial   = np.array([np.cos(theta_coxa), np.sin(theta_coxa), 0.0])
        vertical = np.array([0.0, 0.0, 1.0])

        p_coxa  = np.zeros(3)
        p_femur = self.p.L_coxa * radial

        femur_dir = (np.cos(theta_femur) * radial +
                     np.sin(theta_femur) * vertical)
        p_tibia   = p_femur + self.p.L_femur * femur_dir

        total_angle = theta_femur + theta_tibia
        tibia_dir   = (np.cos(total_angle) * radial +
                       np.sin(total_angle) * vertical)
        p_foot      = p_tibia + self.p.L_tibia * tibia_dir

        def to_body(p): return self.R_leg2body @ p + self.body_offset
        return {'coxa':  to_body(p_coxa),
                'femur': to_body(p_femur),
                'tibia': to_body(p_tibia),
                'foot':  to_body(p_foot)}

    def get_rest_foot_body(self) -> np.ndarray:
        """Posición del pie en postura de reposo, en frame del cuerpo."""
        p_rest = np.array([self.p.L_coxa + self.rest_x,
                           self.rest_y,
                           self.rest_z])
        return self.R_leg2body @ p_rest + self.body_offset


# ──────────────────────────────────────────────────────────────────────
#  ROBOT COMPLETO — 4 PATAS
# ──────────────────────────────────────────────────────────────────────

class SpiderQuadruped:
    """
    Robot cuadrúpedo araña: 4 patas × 3 GDL = 12 DOF.

    En C++: struct con 4 instancias de Leg_t indexadas por
    {FL=0, FR=1, RL=2, RR=3}.
    """

    LEG_ORDER = ['FL', 'FR', 'RL', 'RR']

    def __init__(self, params: Optional[RobotParams] = None):
        self.p = params or RobotParams()
        self._build_legs()

    def _build_legs(self):
        bx = self.p.body_length / 2
        by = self.p.body_width  / 2

        leg_defs = [
            ('FL', [ bx,  by, 0], np.radians( 45)),
            ('FR', [ bx, -by, 0], np.radians(-45)),
            ('RL', [-bx,  by, 0], np.radians(135)),
            ('RR', [-bx, -by, 0], np.radians(225)),
        ]

        self.legs = {}
        for name, offset, angle in leg_defs:
            self.legs[name] = SpiderLeg(name, offset, angle, self.p)

    def rest_pose(self):
        """Lleva todas las patas a su postura de reposo."""
        for leg in self.legs.values():
            leg.ik(leg.get_rest_foot_body())

    def set_pose_ik(self, foot_targets: dict) -> dict:
        """IK simultánea para las 4 patas. Devuelve ángulos (None si falla)."""
        results = {}
        for name, pos in foot_targets.items():
            q = self.legs[name].ik(np.array(pos))
            results[name] = q
            if q is None:
                print(f"  ⚠  Pata {name}: inalcanzable {np.round(pos, 3)}")
        return results

    def get_all_fk(self) -> dict:
        """FK de las 4 patas con los ángulos actuales."""
        return {name: leg.fk() for name, leg in self.legs.items()}

    def print_joint_angles(self):
        """Imprime ángulos articulares actuales [°]."""
        print(f"\n{'='*44}")
        print(f"  {'Pata':<6} {'Coxa':>8} {'Fémur':>8} {'Tibia':>8}")
        print(f"  {'-'*36}")
        for name, leg in self.legs.items():
            q = np.degrees(leg.q)
            print(f"  {name:<6} {q[0]:>8.2f} {q[1]:>8.2f} {q[2]:>8.2f}")
        print(f"{'='*44}\n")

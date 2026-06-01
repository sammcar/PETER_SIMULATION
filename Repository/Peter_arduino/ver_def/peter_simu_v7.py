"""
peter_simu_v7.py — Simulación fiel a gait.cpp (nueva máquina de estados)
═════════════════════════════════════════════════════════════════════════
Cada frame de animación = una iteración del loop() del Arduino.
Parámetros físicos y de marcha coinciden con config.h.

Controles de teclado — mismas teclas que el teleop serial del Arduino:
    i / ↑        Adelante          , / ↓        Atrás
    j / ←        Strafe izquierda  l / →        Strafe derecha
    u            Adelante + izq    o            Adelante + der
    m            Atrás   + izq     . (punto)    Atrás   + der
    k / Espacio  STOP (retorno suave a reposo)

Ejecutar desde la raíz del proyecto:
    uv run ver_def/peter_simu_v7.py
"""

import sys, os
_here   = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_here)
for _p in (_parent, _here):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import itertools
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.widgets import Slider, Button
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from robot_params     import RobotParams
from robot_kinematics import SpiderQuadruped
from ver_def.gait_v2  import (
    GaitV2,
    GAIT_STOP, GAIT_FORWARD, GAIT_BACKWARD,
    GAIT_LEFT, GAIT_RIGHT,
    GAIT_FWD_LEFT, GAIT_FWD_RIGHT,
    GAIT_BCK_LEFT, GAIT_BCK_RIGHT,
    PHASE_IDLE, PHASE_BODY, PHASE_RETURN,
    LEG_NAMES,
)

# ── Parámetros físicos — deben coincidir con config.h ─────────────────
params = RobotParams(
    body_length = 0.145,   # BODY_LEN
    body_width  = 0.105,   # BODY_WID
    L_coxa      = 0.042,   # L_COXA
    L_femur     = 0.068,   # L_FEMUR
    L_tibia     = 0.123,   # L_TIBIA
    rest_x      = 0.11,    # REST_X (corregido de 0.00 → 0.11; ver config.h)
    rest_z      = -0.100,  # REST_Z
    tibia_lim   = (-150.0, 0.0),
)

# ── Paleta visual ──────────────────────────────────────────────────────
LEG_COLORS = {'FL': '#00BFFF', 'FR': '#FF6B35', 'RL': '#7CFC00', 'RR': '#FF1493'}
PHASE_COLORS = {
    PHASE_IDLE:   '#555577',
    'LIFT':       '#FFD700',
    'MOVE':       '#FF8C00',
    'LOWER':      '#FF4444',
    PHASE_BODY:   '#44EE88',
    PHASE_RETURN: '#AA88FF',
}

# ── Mapeo teclado → dirección (igual que teleop del Arduino) ──────────
KEY_DIR = {
    'i': GAIT_FORWARD,    'up':    GAIT_FORWARD,
    ',': GAIT_BACKWARD,   'down':  GAIT_BACKWARD,
    'j': GAIT_LEFT,       'left':  GAIT_LEFT,
    'l': GAIT_RIGHT,      'right': GAIT_RIGHT,
    'u': GAIT_FWD_LEFT,
    'o': GAIT_FWD_RIGHT,
    'm': GAIT_BCK_LEFT,
    '.': GAIT_BCK_RIGHT,
    'k': GAIT_STOP,       ' ':     GAIT_STOP,
}


# ──────────────────────────────────────────────────────────────────────
#  PRIMITIVAS DE DIBUJO
# ──────────────────────────────────────────────────────────────────────

def _draw_body(ax, params: RobotParams, R=None, t=None):
    if R is None: R = np.eye(3)
    if t is None: t = np.zeros(3)
    bx, by, bz = params.body_length / 2, params.body_width / 2, 0.015
    corners = np.array([
        [-bx,-by, bz],[ bx,-by, bz],[ bx, by, bz],[-bx, by, bz],
        [-bx,-by,-bz],[ bx,-by,-bz],[ bx, by,-bz],[-bx, by,-bz],
    ])
    rc = np.array([R @ c + t for c in corners])
    faces = [
        [rc[0],rc[1],rc[2],rc[3]], [rc[4],rc[5],rc[6],rc[7]],
        [rc[0],rc[1],rc[5],rc[4]], [rc[2],rc[3],rc[7],rc[6]],
        [rc[0],rc[3],rc[7],rc[4]], [rc[1],rc[2],rc[6],rc[5]],
    ]
    ax.add_collection3d(Poly3DCollection(
        faces, facecolors='#2E4057', edgecolors='#4A6FA5',
        alpha=0.35, linewidths=1.2))
    arrow_start = R @ np.array([0., 0., bz + 0.005]) + t
    arrow_dir   = R @ np.array([0.04, 0., 0.])
    ax.quiver(*arrow_start, *arrow_dir, color='#FFD700',
              linewidth=2, arrow_length_ratio=0.4)


def _draw_leg(ax, fk_data: dict, name: str):
    color = LEG_COLORS[name]
    pts = [fk_data['coxa'], fk_data['femur'], fk_data['tibia'], fk_data['foot']]
    for p0, p1 in zip(pts, pts[1:]):
        ax.plot([p0[0],p1[0]], [p0[1],p1[1]], [p0[2],p1[2]],
                color=color, linewidth=3.5, solid_capstyle='round')
    for pt, sz, mk in zip(pts, [60, 45, 35, 80], ['o','o','o','^']):
        ax.scatter(*pt, s=sz, c='#FFFFFF', marker=mk,
                   edgecolors=color, linewidths=1.5, zorder=5)
    ax.text(pts[-1][0], pts[-1][1], pts[-1][2]-0.015, name,
            fontsize=7, color=color, ha='center', fontweight='bold')


def _style_ax(ax, cx, half=0.35):
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(-half, half)
    ax.set_zlim(-0.22, 0.18)
    ax.set_xlabel('X (frente →)', color='#8899BB', fontsize=8, labelpad=5)
    ax.set_ylabel('Y (← izq)',    color='#8899BB', fontsize=8, labelpad=5)
    ax.set_zlabel('Z (↑ arriba)', color='#8899BB', fontsize=8, labelpad=5)
    for spine in ['x', 'y', 'z']:
        ax.tick_params(axis=spine, colors='#555577', labelsize=7)
    ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
    for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
        pane.set_edgecolor('#1E2233')
    ax.grid(color='#1E2233', linestyle='--', linewidth=0.5, alpha=0.6)
    patches = [mpatches.Patch(color=c, label=n) for n, c in LEG_COLORS.items()]
    ax.legend(handles=patches, loc='upper left',
              facecolor='#1A1E2E', edgecolor='#4A6FA5',
              labelcolor='white', fontsize=8)


# ──────────────────────────────────────────────────────────────────────
#  ANIMACIÓN
# ──────────────────────────────────────────────────────────────────────

def animate_v2(robot: SpiderQuadruped, gait: GaitV2) -> FuncAnimation:
    fps      = 30
    _paused  = [False]
    _steps   = [0]

    # ── Figura ────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 10), facecolor='#0D1117')
    ax  = fig.add_axes([0.04, 0.36, 0.92, 0.60], projection='3d')
    ax.set_facecolor('#0D1117')
    ax.view_init(elev=26, azim=-55)

    z_floor = float(gait._site_rest[:, 2].min()) - 0.01

    sl_kw = dict(facecolor='#1A1E2E')

    # ── Sliders ───────────────────────────────────────────────────────
    fig.text(0.13, 0.335, '── Marcha ──',
             ha='center', color='#AAC8FF', fontsize=9, fontweight='bold')
    sl_slen  = Slider(fig.add_axes([0.04, 0.270, 0.18, 0.023], **sl_kw),
                      'Paso [cm]', 1., 8., valinit=gait.step_len*100,  color='#5599DD')
    sl_sh    = Slider(fig.add_axes([0.04, 0.220, 0.18, 0.023], **sl_kw),
                      'Alt  [cm]', 1., 7., valinit=gait.step_h*100,    color='#5599DD')
    sl_lspd  = Slider(fig.add_axes([0.04, 0.170, 0.18, 0.023], **sl_kw),
                      'VelPata',   1., 20., valinit=gait.leg_speed*1000, color='#4A6FA5')
    sl_bspd  = Slider(fig.add_axes([0.04, 0.120, 0.18, 0.023], **sl_kw),
                      'VelCuerpo', 1., 15., valinit=gait.body_speed*1000, color='#4A6FA5')
    for sl in (sl_slen, sl_sh, sl_lspd, sl_bspd):
        sl.label.set_color('#C8D8FF'); sl.valtext.set_color('#C8D8FF')

    fig.text(0.13, 0.095, '(VelPata/Cuerpo en mm/iter)',
             ha='center', color='#556677', fontsize=7)

    # ── Botones pausa / reset ─────────────────────────────────────────
    btn_kw    = dict(color='#2E4057', hovercolor='#4A6FA5')
    btn_pause = Button(fig.add_axes([0.04, 0.055, 0.085, 0.035], facecolor='#2E4057'),
                       'Pausar', **btn_kw)
    btn_reset = Button(fig.add_axes([0.135, 0.055, 0.085, 0.035], facecolor='#2E4057'),
                       'Reset',  **btn_kw)
    for btn in (btn_pause, btn_reset):
        btn.label.set_color('#C8D8FF'); btn.label.set_fontsize(9)

    # ── D-pad de dirección — mismas teclas que el Arduino ────────────
    fig.text(0.50, 0.340, '── Dirección  (i/j/k/l/u/o/m/.) ──',
             ha='center', color='#88AACC', fontsize=9, fontweight='bold')

    _dir_kw = dict(color='#1A3050', hovercolor='#3A6090')
    _stp_kw = dict(color='#3A2020', hovercolor='#6A4040')
    _dia_kw = dict(color='#0E2030', hovercolor='#2A5060')

    # Diagonal izquierda / derecha (fila superior)
    btn_fu = Button(fig.add_axes([0.32, 0.280, 0.09, 0.030], facecolor='#0E2030'),
                    'u: ↖ Del+Izq', **_dia_kw)
    btn_fo = Button(fig.add_axes([0.55, 0.280, 0.09, 0.030], facecolor='#0E2030'),
                    'o: ↗ Del+Der', **_dia_kw)
    # Fila central
    btn_fwd = Button(fig.add_axes([0.42, 0.280, 0.10, 0.030], facecolor='#1A3050'),
                     'i: ▲ Adelante', **_dir_kw)
    btn_lft = Button(fig.add_axes([0.32, 0.240, 0.09, 0.030], facecolor='#1A3050'),
                     'j: ◄ Izq',     **_dir_kw)
    btn_stp = Button(fig.add_axes([0.42, 0.240, 0.10, 0.030], facecolor='#3A2020'),
                     'k: ■ STOP',    **_stp_kw)
    btn_rgt = Button(fig.add_axes([0.53, 0.240, 0.09, 0.030], facecolor='#1A3050'),
                     'l: ► Der',     **_dir_kw)
    # Diagonal izquierda / derecha (fila inferior)
    btn_bm = Button(fig.add_axes([0.32, 0.200, 0.09, 0.030], facecolor='#0E2030'),
                    'm: ↙ At+Izq', **_dia_kw)
    btn_bwd = Button(fig.add_axes([0.42, 0.200, 0.10, 0.030], facecolor='#1A3050'),
                     ',: ▼ Atrás',  **_dir_kw)
    btn_bp = Button(fig.add_axes([0.55, 0.200, 0.09, 0.030], facecolor='#0E2030'),
                    '.: ↘ At+Der', **_dia_kw)

    for btn in (btn_fwd, btn_bwd, btn_lft, btn_rgt, btn_stp,
                btn_fu, btn_fo, btn_bm, btn_bp):
        btn.label.set_color('#C8D8FF'); btn.label.set_fontsize(8)

    # ── Frame de animación ────────────────────────────────────────────
    def update_frame(_frame):
        elev, azim = ax.elev, ax.azim
        ax.cla(); ax.set_facecolor('#0D1117')

        if not _paused[0]:
            prev_swing = gait.current_swing
            gait.update()
            if gait.current_swing != prev_swing and gait.phase in (PHASE_BODY,):
                _steps[0] += 1

        bp     = gait.body_pos.copy()
        cx     = bp[0]
        R_body = np.eye(3)

        # Cuerpo
        _draw_body(ax, robot.p, R=R_body, t=bp)

        # Patas
        all_fk = robot.get_all_fk()
        sw_name = gait.current_swing
        for name, fk_data in all_fk.items():
            world_fk = {k: R_body @ v + bp for k, v in fk_data.items()}
            _draw_leg(ax, world_fk, name)

        # Marcador dorado en la pata de balanceo
        sw_foot = R_body @ all_fk[sw_name]['foot'] + bp
        ax.scatter(*sw_foot, s=140, c='#FFD700', marker='o',
                   edgecolors='#FF8C00', linewidths=2, zorder=10)

        # Suelo y proyecciones
        hv = 0.35
        xx, yy = np.meshgrid([cx-hv, cx+hv], [-hv, hv])
        ax.plot_surface(xx, yy, np.full_like(xx, z_floor),
                        alpha=0.07, color='#4A6FA5')
        for name, fk_data in all_fk.items():
            fw = R_body @ fk_data['foot'] + bp
            ax.plot([fw[0]]*2, [fw[1]]*2, [fw[2], z_floor],
                    '--', color=LEG_COLORS[name], alpha=0.4, linewidth=1)
            ax.scatter(fw[0], fw[1], z_floor,
                       s=28, c=LEG_COLORS[name], alpha=0.55, marker='x')

        _style_ax(ax, cx, hv)

        # ── Título / estado ───────────────────────────────────────────
        ph  = gait.phase
        col = PHASE_COLORS.get(ph, '#FFFFFF')
        estado = '⏸ PAUSA' if _paused[0] else f'Fase: {ph}'
        fig.suptitle(
            f"gait_v2  │  {estado}  │  Swing: {sw_name}  │  "
            f"Dir: {gait.direction}  │  Pasos: {_steps[0]}  │  "
            f"X: {bp[0]*100:+.1f} cm",
            color=col, fontsize=12, fontweight='bold', y=0.97)

        # Tabla de ángulos articulares
        rows = ['  Pata   Coxa    Fémur   Tibia']
        for n in LEG_NAMES:
            q = np.degrees(robot.legs[n].q)
            rows.append(f'  {n}   {q[0]:>+6.1f}°  {q[1]:>+6.1f}°  {q[2]:>+6.1f}°')
        ax.text2D(0.01, 0.01, '\n'.join(rows),
                  transform=ax.transAxes, ha='left', va='bottom',
                  color='#88CCAA', fontsize=7, fontfamily='monospace',
                  bbox=dict(boxstyle='round,pad=0.3', facecolor='#0A0E18',
                            alpha=0.85, edgecolor='#1E2A3A'))

        # Info de velocidad
        ax.text2D(0.99, 0.99,
                  f"Paso:  {gait.step_len*100:.1f} cm\n"
                  f"Alt:   {gait.step_h*100:.1f} cm\n"
                  f"VPata: {gait.leg_speed*1000:.1f} mm/it\n"
                  f"VCuerp:{gait.body_speed*1000:.1f} mm/it",
                  transform=ax.transAxes, ha='right', va='top',
                  color='#7799BB', fontsize=7.5, fontfamily='monospace',
                  bbox=dict(boxstyle='round,pad=0.3', facecolor='#0D1117',
                            alpha=0.75, edgecolor='none'))

        ax.view_init(elev=elev, azim=azim)

    # ── Callbacks ─────────────────────────────────────────────────────
    def on_slider(_):
        gait.step_len     = sl_slen.val / 100.
        gait.step_h       = sl_sh.val   / 100.
        gait.leg_speed    = sl_lspd.val / 1000.
        gait.body_speed   = sl_bspd.val / 1000.

    def on_pause(_):
        _paused[0] = not _paused[0]
        btn_pause.label.set_text('Reanudar' if _paused[0] else 'Pausar')
        fig.canvas.draw_idle()

    def on_reset(_):
        gait.reset(); _steps[0] = 0
        _paused[0] = False
        btn_pause.label.set_text('Pausar')

    def on_key(event):
        d = KEY_DIR.get(event.key)
        if d is not None:
            gait.set_dir(d)

    for sl in (sl_slen, sl_sh, sl_lspd, sl_bspd):
        sl.on_changed(on_slider)
    btn_pause.on_clicked(on_pause)
    btn_reset.on_clicked(on_reset)

    btn_fwd.on_clicked(lambda _: gait.set_dir(GAIT_FORWARD))
    btn_bwd.on_clicked(lambda _: gait.set_dir(GAIT_BACKWARD))
    btn_lft.on_clicked(lambda _: gait.set_dir(GAIT_LEFT))
    btn_rgt.on_clicked(lambda _: gait.set_dir(GAIT_RIGHT))
    btn_stp.on_clicked(lambda _: gait.set_dir(GAIT_STOP))
    btn_fu.on_clicked(lambda _: gait.set_dir(GAIT_FWD_LEFT))
    btn_fo.on_clicked(lambda _: gait.set_dir(GAIT_FWD_RIGHT))
    btn_bm.on_clicked(lambda _: gait.set_dir(GAIT_BCK_LEFT))
    btn_bp.on_clicked(lambda _: gait.set_dir(GAIT_BCK_RIGHT))

    fig.canvas.mpl_connect('key_press_event', on_key)

    anim = FuncAnimation(fig, update_frame, frames=itertools.count(),
                         interval=int(1000 / fps), blit=False, repeat=False,
                         cache_frame_data=False)
    plt.show()
    return anim


# ── Punto de entrada ───────────────────────────────────────────────────

if __name__ == '__main__':
    robot = SpiderQuadruped(params)
    gait  = GaitV2(robot,
                   step_len     = 0.040,   # STEP_LEN
                   step_len_lat = 0.020,   # STEP_LEN_LAT
                   step_h       = 0.035,   # STEP_H
                   leg_speed    = 0.006,   # LEG_SPEED
                   body_speed   = 0.003,   # BODY_SPEED
                   reach_tol    = 0.002)   # REACH_TOL

    animate_v2(robot, gait)

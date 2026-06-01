"""
robot_viz.py — Visualización 3D y animación (solo Python / matplotlib)
═══════════════════════════════════════════════════════════════════════
NO PORTABLE A ESP32. Este módulo usa matplotlib exclusivamente para
simular y depurar el comportamiento del robot antes de pasar al hardware.

Exports públicos:
    animate_crawl(robot, gait, n_cycles)  — animación principal con IMU
    visualize(robot, title)               — snapshot 3D estático
    visualize_interactive(robot, title)   — sliders posición + orientación
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.patches as mpatches
from matplotlib.widgets import Slider, Button
from matplotlib.animation import FuncAnimation
from typing import Optional
import itertools
import warnings
warnings.filterwarnings('ignore')

from ver_def.robot_math import rot_rpy, rot_z
from ver_def.robot_params import RobotParams
from ver_def.robot_kinematics import SpiderQuadruped
from ver_def.robot_gait import (CrawlGait, CRAWL_SEQUENCE,
                         DIR_FORWARD, DIR_BACKWARD,
                         DIR_TURN_LEFT, DIR_TURN_RIGHT, DIR_STOP)
from ver_def.robot_imu import IMUSimulator, IMUCompensator
from ver_def.robot_poses import (RoutinePlayer, current_foot_pose,
                          routine_lie_down, routine_stand_up)


# ── Paleta visual ──────────────────────────────────────────────────────

LEG_COLORS = {
    'FL': '#00BFFF',   # azul claro
    'FR': '#FF6B35',   # naranja
    'RL': '#7CFC00',   # verde lima
    'RR': '#FF1493',   # magenta
}
JOINT_COLOR = '#FFFFFF'
BODY_COLOR  = '#2E4057'
BODY_ALPHA  = 0.35


# ──────────────────────────────────────────────────────────────────────
#  PRIMITIVAS DE DIBUJO
# ──────────────────────────────────────────────────────────────────────

def draw_body(ax, params: RobotParams,
              R: np.ndarray = None, t: np.ndarray = None):
    """Dibuja el cuerpo rectangular del robot con orientación R y posición t."""
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
        faces, facecolors=BODY_COLOR, edgecolors='#4A6FA5',
        alpha=BODY_ALPHA, linewidths=1.2))

    a_start = R @ np.array([0., 0., bz + 0.005]) + t
    a_dir   = R @ np.array([0.04, 0., 0.])
    ax.quiver(*a_start, *a_dir, color='#FFD700', linewidth=2,
              arrow_length_ratio=0.4)


def draw_leg(ax, fk_data: dict, name: str):
    """Dibuja una pata con sus segmentos y articulaciones."""
    color = LEG_COLORS[name]
    pts   = [fk_data['coxa'], fk_data['femur'], fk_data['tibia'], fk_data['foot']]

    for p0, p1 in zip(pts, pts[1:]):
        ax.plot([p0[0],p1[0]], [p0[1],p1[1]], [p0[2],p1[2]],
                color=color, linewidth=3.5, solid_capstyle='round')

    for pt, sz, mk in zip(pts, [60,45,35,80], ['o','o','o','^']):
        ax.scatter(*pt, s=sz, c=JOINT_COLOR, marker=mk,
                   edgecolors=color, linewidths=1.5, zorder=5)

    foot = fk_data['foot']
    ax.text(foot[0], foot[1], foot[2]-0.015, name,
            fontsize=7, color=color, ha='center', fontweight='bold')


def _apply_3d_style(ax, cx: float = 0.0, half_view: float = 0.32):
    ax.set_xlim(cx - half_view, cx + half_view)
    ax.set_ylim(-half_view, half_view)
    ax.set_zlim(-0.22, 0.16)
    for spine in ['x', 'y', 'z']:
        ax.tick_params(axis=spine, colors='#555577', labelsize=7)
    ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
    for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
        pane.set_edgecolor('#1E2233')
    ax.grid(color='#1E2233', linestyle='--', linewidth=0.5, alpha=0.6)
    ax.set_xlabel('X (frente →)', color='#8899BB', fontsize=8, labelpad=6)
    ax.set_ylabel('Y (← izq)',    color='#8899BB', fontsize=8, labelpad=6)
    ax.set_zlabel('Z (↑ arriba)', color='#8899BB', fontsize=8, labelpad=6)
    patches = [mpatches.Patch(color=c, label=n) for n, c in LEG_COLORS.items()]
    ax.legend(handles=patches, loc='upper left',
              facecolor='#1A1E2E', edgecolor='#4A6FA5',
              labelcolor='white', fontsize=9)


# ──────────────────────────────────────────────────────────────────────
#  VISUALIZACIÓN ESTÁTICA
# ──────────────────────────────────────────────────────────────────────

def visualize(robot: SpiderQuadruped,
              title: str = "Spider Quadruped — Cinemática"):
    """Snapshot 3D del robot en su postura actual."""
    fig = plt.figure(figsize=(13, 9), facecolor='#0D1117')
    ax  = fig.add_subplot(111, projection='3d')
    ax.set_facecolor('#0D1117')

    draw_body(ax, robot.p)
    all_fk = robot.get_all_fk()
    for name, fk_data in all_fk.items():
        draw_leg(ax, fk_data, name)

    z_floor = min(fk['foot'][2] for fk in all_fk.values()) - 0.01
    lim = 0.22
    xx, yy = np.meshgrid([-lim, lim], [-lim, lim])
    ax.plot_surface(xx, yy, np.full_like(xx, z_floor), alpha=0.07, color='#4A6FA5')

    for name, fk_data in all_fk.items():
        foot = fk_data['foot']
        ax.plot([foot[0]]*2, [foot[1]]*2, [foot[2], z_floor],
                '--', color=LEG_COLORS[name], alpha=0.35, linewidth=1)
        ax.scatter(foot[0], foot[1], z_floor,
                   s=25, c=LEG_COLORS[name], alpha=0.5, marker='x')

    ax.set_xlim(-0.25, 0.25); ax.set_ylim(-0.25, 0.25); ax.set_zlim(-0.25, 0.10)
    for spine in ['x','y','z']: ax.tick_params(axis=spine, colors='#555577', labelsize=7)
    ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
    for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
        pane.set_edgecolor('#1E2233')
    ax.grid(color='#1E2233', linestyle='--', linewidth=0.5, alpha=0.6)
    ax.set_xlabel('X (frente →)', color='#8899BB', fontsize=8, labelpad=6)
    ax.set_ylabel('Y (← izq)',    color='#8899BB', fontsize=8, labelpad=6)
    ax.set_zlabel('Z (↑ arriba)', color='#8899BB', fontsize=8, labelpad=6)
    patches = [mpatches.Patch(color=c, label=n) for n, c in LEG_COLORS.items()]
    ax.legend(handles=patches, loc='upper left',
              facecolor='#1A1E2E', edgecolor='#4A6FA5',
              labelcolor='white', fontsize=9)
    ax.text(0.22, 0, 0.005, '▶ FRENTE', color='#FFD700', fontsize=8, fontweight='bold')
    fig.suptitle(title, color='#C8D8FF', fontsize=14, fontweight='bold', y=0.97)
    ax.view_init(elev=25, azim=-55)
    plt.tight_layout()
    plt.show()


# ──────────────────────────────────────────────────────────────────────
#  VISUALIZACIÓN INTERACTIVA (posición + orientación)
# ──────────────────────────────────────────────────────────────────────

def visualize_interactive(robot: SpiderQuadruped,
                          title: str = "Spider Quadruped — Posición & Orientación"):
    """Sliders de posición X/Y/Z y orientación Roll/Pitch/Yaw del cuerpo."""
    robot.rest_pose()
    all_fk0    = robot.get_all_fk()
    foot_world = {n: fk['foot'].copy() for n, fk in all_fk0.items()}

    fig = plt.figure(figsize=(14, 11), facecolor='#0D1117')
    ax  = fig.add_axes([0.05, 0.43, 0.90, 0.53], projection='3d')
    ax.set_facecolor('#0D1117')

    sl_kw = dict(facecolor='#1A1E2E')
    cL, cR, sw = 0.07, 0.55, 0.38
    ys = [0.30, 0.22, 0.13]

    fig.text(cL+sw/2, 0.375, 'Posición cuerpo [cm]',
             ha='center', color='#AAC8FF', fontsize=10, fontweight='bold')
    fig.text(cR+sw/2, 0.375, 'Orientación [°]',
             ha='center', color='#AAC8FF', fontsize=10, fontweight='bold')

    s_px = Slider(fig.add_axes([cL, ys[0], sw, 0.025], **sl_kw), 'X [cm]', -8., 8., valinit=0., color='#5599DD')
    s_py = Slider(fig.add_axes([cL, ys[1], sw, 0.025], **sl_kw), 'Y [cm]', -8., 8., valinit=0., color='#5599DD')
    s_pz = Slider(fig.add_axes([cL, ys[2], sw, 0.025], **sl_kw), 'Z [cm]', -8., 4., valinit=0., color='#5599DD')
    s_roll  = Slider(fig.add_axes([cR, ys[0], sw, 0.025], **sl_kw), 'Roll  [°]', -30, 30, valinit=0, color='#4A6FA5')
    s_pitch = Slider(fig.add_axes([cR, ys[1], sw, 0.025], **sl_kw), 'Pitch [°]', -30, 30, valinit=0, color='#4A6FA5')
    s_yaw   = Slider(fig.add_axes([cR, ys[2], sw, 0.025], **sl_kw), 'Yaw   [°]', -30, 30, valinit=0, color='#4A6FA5')
    for sl in (s_px, s_py, s_pz, s_roll, s_pitch, s_yaw):
        sl.label.set_color('#C8D8FF'); sl.valtext.set_color('#C8D8FF')

    btn_reset = Button(fig.add_axes([0.44, 0.035, 0.12, 0.038], facecolor='#2E4057'),
                       'Reset', color='#2E4057', hovercolor='#4A6FA5')
    btn_reset.label.set_color('#C8D8FF')
    _suppressed = [False]

    def _draw(t, R):
        elev, azim = ax.elev, ax.azim
        ax.cla(); ax.set_facecolor('#0D1117')
        draw_body(ax, robot.p, R, t)
        for name, leg in robot.legs.items():
            leg.ik(R.T @ (foot_world[name] - t))
        all_fk = robot.get_all_fk()
        for name, fk_data in all_fk.items():
            draw_leg(ax, {k: R@v+t for k,v in fk_data.items()}, name)
        z_floor = min(foot_world[n][2] for n in robot.legs) - 0.01
        lim = 0.32
        xx, yy = np.meshgrid([-lim,lim], [-lim,lim])
        ax.plot_surface(xx, yy, np.full_like(xx, z_floor), alpha=0.07, color='#4A6FA5')
        for name in robot.legs:
            fw = foot_world[name]
            ax.plot([fw[0]]*2, [fw[1]]*2, [fw[2], z_floor],
                    '--', color=LEG_COLORS[name], alpha=0.35, linewidth=1)
            ax.scatter(fw[0], fw[1], z_floor, s=25, c=LEG_COLORS[name], alpha=0.5, marker='x')
        for d in [np.array([.03,0,0]), np.array([0,.03,0]), np.array([0,0,.03])]:
            ax.plot([t[0],t[0]+R[0]@d], [t[1],t[1]+R[1]@d], [t[2],t[2]+R[2]@d],
                    '-', color='#FFD700', linewidth=1.2, alpha=0.7)
        ax.set_xlim(-0.32,0.32); ax.set_ylim(-0.32,0.32); ax.set_zlim(-0.28,0.18)
        for spine in ['x','y','z']: ax.tick_params(axis=spine, colors='#555577', labelsize=7)
        ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.set_edgecolor('#1E2233')
        ax.grid(color='#1E2233', linestyle='--', linewidth=0.5, alpha=0.6)
        ax.set_xlabel('X (frente →)', color='#8899BB', fontsize=8, labelpad=6)
        ax.set_ylabel('Y (← izq)',    color='#8899BB', fontsize=8, labelpad=6)
        ax.set_zlabel('Z (↑ arriba)', color='#8899BB', fontsize=8, labelpad=6)
        patches = [mpatches.Patch(color=c, label=n) for n,c in LEG_COLORS.items()]
        ax.legend(handles=patches, loc='upper left',
                  facecolor='#1A1E2E', edgecolor='#4A6FA5', labelcolor='white', fontsize=9)
        ax.view_init(elev=elev, azim=azim)
        fig.canvas.draw_idle()

    def _state():
        t = np.array([s_px.val/100, s_py.val/100, s_pz.val/100])
        R = rot_rpy(np.radians(s_roll.val), np.radians(s_pitch.val), np.radians(s_yaw.val))
        return t, R

    def update(_):
        if not _suppressed[0]: _draw(*_state())

    def on_reset(_):
        _suppressed[0] = True
        for sl in (s_px, s_py, s_pz, s_roll, s_pitch, s_yaw): sl.set_val(0)
        _suppressed[0] = False
        _draw(np.zeros(3), np.eye(3))

    for sl in (s_px, s_py, s_pz, s_roll, s_pitch, s_yaw): sl.on_changed(update)
    btn_reset.on_clicked(on_reset)
    fig.suptitle(title, color='#C8D8FF', fontsize=14, fontweight='bold', y=0.99)
    _draw(np.zeros(3), np.eye(3))
    ax.view_init(elev=25, azim=-55)
    plt.show()


# ──────────────────────────────────────────────────────────────────────
#  ANIMACIÓN PRINCIPAL — MARCHA CRAWL + IMU + DIRECCIÓN
# ──────────────────────────────────────────────────────────────────────

def animate_crawl(robot: SpiderQuadruped, gait: CrawlGait,
                  n_cycles: Optional[int] = None,
                  autostart: bool = True) -> FuncAnimation:
    """
    Anima la marcha crawl con compensación IMU y control de dirección.

    Controles UI:
      Sliders Marcha  — paso, altura, duración
      Sliders IMU     — amplitud roll y pitch
      Sliders Centro  — offset X/Y/Z del cuerpo (body IK)
      D-pad + teclado — ↑↓←→ / WASD / Space para dirección
      Pausar / Reset
    """
    fps     = 30
    dt_anim = 1.0 / fps

    frames_src = (range(max(1, int(n_cycles * len(CRAWL_SEQUENCE) *
                                    gait.step_duration * fps)))
                  if n_cycles is not None else itertools.count())

    imu      = IMUSimulator(roll_amp=np.radians(0.), pitch_amp=np.radians(0.), freq=0.4)
    comp     = IMUCompensator(alpha=0.15, max_correction_deg=20.)
    t_sim    = [0.0]
    imu_last = [0.0, 0.0]

    # ── Figura ────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 10), facecolor='#0D1117')
    ax  = fig.add_axes([0.05, 0.37, 0.90, 0.57], projection='3d')
    ax.set_facecolor('#0D1117')

    half_view = 0.45
    z_floor   = min(fw[2] for fw in gait.foot_world.values()) - 0.01
    sl_kw     = dict(facecolor='#1A1E2E')

    # ── Encabezados de sección ────────────────────────────────────────
    fig.text(0.23, 0.335, '─── Marcha ───',
             ha='center', color='#AAC8FF', fontsize=9, fontweight='bold')
    fig.text(0.55, 0.335, '─── IMU ───',
             ha='center', color='#FFCC88', fontsize=9, fontweight='bold')

    # ── Sliders marcha ────────────────────────────────────────────────
    sl_len = Slider(fig.add_axes([0.07, 0.265, 0.32, 0.025], **sl_kw),
                    'Paso   [cm]', 2., 12., valinit=gait.step_length*100, color='#5599DD')
    sl_hgt = Slider(fig.add_axes([0.07, 0.190, 0.32, 0.025], **sl_kw),
                    'Altura [cm]', 0.5, 8., valinit=gait.step_height*100, color='#5599DD')
    sl_dur = Slider(fig.add_axes([0.07, 0.115, 0.32, 0.025], **sl_kw),
                    'Dur    [s] ', 0.2, 1.5, valinit=gait.step_duration,  color='#4A6FA5')

    # ── Sliders IMU ───────────────────────────────────────────────────
    sl_ramp = Slider(fig.add_axes([0.44, 0.250, 0.19, 0.025], **sl_kw),
                     'Roll  [°]', 0., 20., valinit=np.degrees(imu.roll_amp),  color='#DD5533')
    sl_pamp = Slider(fig.add_axes([0.44, 0.175, 0.19, 0.025], **sl_kw),
                     'Pitch [°]', 0., 20., valinit=np.degrees(imu.pitch_amp), color='#DD5533')

    for sl in (sl_len, sl_hgt, sl_dur, sl_ramp, sl_pamp):
        sl.label.set_color('#C8D8FF'); sl.valtext.set_color('#C8D8FF')

    # ── Botones pausa / reset ─────────────────────────────────────────
    btn_kw = dict(color='#2E4057', hovercolor='#4A6FA5')
    btn_pause = Button(fig.add_axes([0.70, 0.265, 0.21, 0.038], facecolor='#2E4057'),
                       'Pausar', **btn_kw)
    btn_reset = Button(fig.add_axes([0.70, 0.212, 0.21, 0.038], facecolor='#2E4057'),
                       'Reset',  **btn_kw)
    for btn in (btn_pause, btn_reset):
        btn.label.set_color('#C8D8FF'); btn.label.set_fontsize(10)

    # ── Botones de rutinas ────────────────────────────────────────────
    fig.text(0.805, 0.200, '─── Rutinas ───',
             ha='center', color='#AADDAA', fontsize=8, fontweight='bold')
    btn_lie = Button(fig.add_axes([0.70, 0.152, 0.21, 0.033], facecolor='#1A3020'),
                     '▽  Acostarse', color='#1A3020', hovercolor='#2A5030')
    btn_stu = Button(fig.add_axes([0.70, 0.108, 0.21, 0.033], facecolor='#1A2040'),
                     '△  Levantarse', color='#1A2040', hovercolor='#2A3060')
    for btn in (btn_lie, btn_stu):
        btn.label.set_color('#AADDCC'); btn.label.set_fontsize(9)

    # ── D-pad de dirección ────────────────────────────────────────────
    dir_kw  = dict(color='#1A3050', hovercolor='#3A6090')
    stp_kw  = dict(color='#3A2020', hovercolor='#6A4040')
    btn_fwd = Button(fig.add_axes([0.39, 0.083, 0.12, 0.030], facecolor='#1A3050'), '▲ Adelante', **dir_kw)
    btn_bwd = Button(fig.add_axes([0.39, 0.013, 0.12, 0.030], facecolor='#1A3050'), '▼ Atrás',    **dir_kw)
    btn_lft = Button(fig.add_axes([0.26, 0.048, 0.12, 0.030], facecolor='#1A3050'), '◄ Izq',      **dir_kw)
    btn_rgt = Button(fig.add_axes([0.52, 0.048, 0.12, 0.030], facecolor='#1A3050'), 'Der ►',      **dir_kw)
    btn_stp = Button(fig.add_axes([0.39, 0.048, 0.12, 0.030], facecolor='#3A2020'), '■ Parar',    **stp_kw)
    for btn in (btn_fwd, btn_bwd, btn_lft, btn_rgt, btn_stp):
        btn.label.set_color('#C8D8FF'); btn.label.set_fontsize(8.5)
    fig.text(0.45, 0.120, '─── Dirección (↑↓←→ / WASD / Space) ───',
             ha='center', color='#88AACC', fontsize=8, fontweight='bold')

    # ── IMU legend ────────────────────────────────────────────────────
    fig.text(0.805, 0.165, '● Inclinación (raw)\n● Corrección (filt)',
             ha='center', va='top', color='#AAAAAA', fontsize=7.5)
    for y_dot, col in [(0.165, '#FF4444'), (0.149, '#44EE88')]:
        fig.text(0.762, y_dot, '●', ha='left', va='top', color=col, fontsize=9)

    # ── Sliders posición del centro del cuerpo ────────────────────────
    fig.text(0.795, 0.120, '─── Centro Cuerpo ───',
             ha='center', color='#88CC88', fontsize=8, fontweight='bold')
    sl_bx = Slider(fig.add_axes([0.67, 0.083, 0.24, 0.022], **sl_kw),
                   'Cx [cm]', -5., 5., valinit=0., color='#448844')
    sl_by = Slider(fig.add_axes([0.67, 0.050, 0.24, 0.022], **sl_kw),
                   'Cy [cm]', -5., 5., valinit=0., color='#448844')
    sl_bz = Slider(fig.add_axes([0.67, 0.017, 0.24, 0.022], **sl_kw),
                   'Cz [cm]', -4., 4., valinit=0., color='#448844')
    for sl in (sl_bx, sl_by, sl_bz):
        sl.label.set_color('#C8D8FF'); sl.valtext.set_color('#C8D8FF')

    _paused        = [False]
    step_counter   = [0]
    body_state     = ['stand']   # 'stand' | 'lie'
    _pending_state = ['stand']
    _use_gait      = [True]
    routine_player = RoutinePlayer()

    if not autostart:
        _paused[0] = True
        btn_pause.label.set_text('▶ Iniciar')

    # ── Indicadores IMU en 3D ─────────────────────────────────────────
    def _draw_imu_indicator(bp, roll_raw, pitch_raw):
        SCALE  = 0.35
        center = bp + np.array([0., 0., 0.055])
        red_pt = center + SCALE * np.array([-np.sin(pitch_raw), -np.sin(roll_raw), 0.])
        grn_pt = center + SCALE * np.array([np.sin(comp.pitch_filtered),
                                             np.sin(comp.roll_filtered), 0.])
        for pt, col in [(red_pt, '#FF4444'), (grn_pt, '#44EE88')]:
            ax.plot([center[0],pt[0]], [center[1],pt[1]], [center[2],pt[2]],
                    '-', color=col, linewidth=2.5, alpha=0.85, zorder=15)
            ax.scatter(*pt, s=180, c=col, marker='o',
                       edgecolors='white', linewidths=0.8, zorder=16, alpha=0.9)
        ax.scatter(*center, s=40, c='#FFFFFF', marker='o', alpha=0.5, zorder=17)

    # ── Frame de animación ────────────────────────────────────────────
    def update_frame(_frame):
        elev, azim = ax.elev, ax.azim
        ax.cla(); ax.set_facecolor('#0D1117')

        body_off = np.array([sl_bx.val/100., sl_by.val/100., sl_bz.val/100.])

        if not _paused[0]:
            t_sim[0] += dt_anim
            roll_raw, pitch_raw = imu.read(t_sim[0])
            imu_last[0], imu_last[1] = roll_raw, pitch_raw

            if routine_player.active:
                # ── Rutina de transición activa ──────────────────────
                routine_player.update(dt_anim)
                foot_body = routine_player.current_pose()
                if routine_player.done:
                    body_state[0] = _pending_state[0]
                    if _pending_state[0] == 'stand':
                        gait.reset()
                        _use_gait[0] = True
                for name, pos in foot_body.items():
                    robot.legs[name].ik(pos)
            elif _use_gait[0]:
                # ── Marcha normal ────────────────────────────────────
                prev_idx  = gait.swing_idx
                foot_body = gait.update(dt_anim)
                if gait.swing_idx != prev_idx:
                    step_counter[0] += 1
                foot_body = {n: p - body_off for n, p in foot_body.items()}
                foot_body = comp.compensate(foot_body, roll_raw, pitch_raw)
                for name, pos in foot_body.items():
                    robot.legs[name].ik(pos)
            # else: robot congelado en pose acostado

        bp     = gait.body_pos.copy()
        R_body = rot_z(gait.body_yaw)
        bp_eff = bp + R_body @ body_off
        cx     = bp_eff[0]

        # Cuerpo y patas
        draw_body(ax, robot.p, R=R_body, t=bp_eff)
        all_fk = robot.get_all_fk()
        for name, fk_data in all_fk.items():
            draw_leg(ax, {k: R_body @ v + bp_eff for k, v in fk_data.items()}, name)

        # Pie en swing (marcador dorado)
        sw_foot = R_body @ all_fk[gait.current_swing]['foot'] + bp_eff
        ax.scatter(*sw_foot, s=130, c='#FFD700', marker='o',
                   edgecolors='#FF8C00', linewidths=2, zorder=10)

        # Suelo y proyecciones
        xx, yy = np.meshgrid([cx-half_view, cx+half_view], [-half_view, half_view])
        ax.plot_surface(xx, yy, np.full_like(xx, z_floor), alpha=0.07, color='#4A6FA5')
        for name, fk_data in all_fk.items():
            fw = fk_data['foot'] + bp_eff
            ax.plot([fw[0]]*2, [fw[1]]*2, [fw[2], z_floor],
                    '--', color=LEG_COLORS[name], alpha=0.4, linewidth=1)
            ax.scatter(fw[0], fw[1], z_floor,
                       s=30, c=LEG_COLORS[name], alpha=0.55, marker='x')

        _draw_imu_indicator(bp_eff, imu_last[0], imu_last[1])

        # Estilo
        ax.set_xlim(cx-half_view, cx+half_view)
        ax.set_ylim(-half_view, half_view); ax.set_zlim(-0.22, 0.16)
        for spine in ['x','y','z']: ax.tick_params(axis=spine, colors='#555577', labelsize=7)
        ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.set_edgecolor('#1E2233')
        ax.grid(color='#1E2233', linestyle='--', linewidth=0.5, alpha=0.6)
        ax.set_xlabel('X (frente →)', color='#8899BB', fontsize=8, labelpad=6)
        ax.set_ylabel('Y (← izq)',    color='#8899BB', fontsize=8, labelpad=6)
        ax.set_zlabel('Z (↑ arriba)', color='#8899BB', fontsize=8, labelpad=6)
        patches = [mpatches.Patch(color=c, label=n) for n,c in LEG_COLORS.items()]
        ax.legend(handles=patches, loc='upper left',
                  facecolor='#1A1E2E', edgecolor='#4A6FA5', labelcolor='white', fontsize=9)

        # Título
        t_pct = gait.t_phase / gait.step_duration * 100.
        if _paused[0]:                  estado = '⏸ PAUSA'
        elif routine_player.active:     estado = f'Rutina: {body_state[0]}→{_pending_state[0]}'
        elif not _use_gait[0]:          estado = f'■ {body_state[0].upper()}'
        elif gait._stopped:             estado = '■ PARADO'
        else:                           estado = f'Swing: {gait.current_swing}'
        pend_str = f' → {gait.pending_direction}' if gait.pending_direction else ''
        fig.suptitle(
            f"Crawl+IMU  │  {estado}  [{t_pct:.0f}%]  │  "
            f"Dir:{gait.direction}{pend_str}  │  Yaw:{np.degrees(gait.body_yaw):.1f}°",
            color='#C8D8FF', fontsize=12, fontweight='bold', y=0.97)

        # Panel de info (superpuesto en axes 3D)
        rr = np.degrees(imu_last[0]); pr = np.degrees(imu_last[1])
        rf = np.degrees(comp.roll_filtered); pf = np.degrees(comp.pitch_filtered)
        home_str  = f'  ({gait._homing_steps}→home)' if gait._homing_steps > 0 else ''
        pend_info = f'\nPend:{gait.pending_direction}' if gait.pending_direction else ''
        ax.text2D(0.99, 0.99,
                  f"Paso:{gait.step_length*100:.1f}cm  Alt:{gait.step_height*100:.1f}cm\n"
                  f"Vel:{gait.body_vel*100:.1f}cm/s  Pasos:{step_counter[0]}\n"
                  f"Dir:{gait.direction}{home_str}{pend_info}\n"
                  f"Roll  {rr:+5.1f}°→{rf:+5.1f}°\n"
                  f"Pitch {pr:+5.1f}°→{pf:+5.1f}°",
                  transform=ax.transAxes, ha='right', va='top',
                  color='#7799BB', fontsize=7, fontfamily='monospace',
                  bbox=dict(boxstyle='round,pad=0.3', facecolor='#0D1117',
                            alpha=0.75, edgecolor='none'))

        # Tabla de ángulos articulares (esquina inferior izquierda)
        q_rows = ['  Pata   Coxa    Fémur   Tibia']
        for _n in ['FL', 'FR', 'RL', 'RR']:
            _q = np.degrees(robot.legs[_n].q)
            q_rows.append(f'  {_n}   {_q[0]:>+6.1f}°  {_q[1]:>+6.1f}°  {_q[2]:>+6.1f}°')
        ax.text2D(0.01, 0.01, '\n'.join(q_rows),
                  transform=ax.transAxes, ha='left', va='bottom',
                  color='#88CCAA', fontsize=7, fontfamily='monospace',
                  bbox=dict(boxstyle='round,pad=0.3', facecolor='#0A0E18',
                            alpha=0.85, edgecolor='#1E2A3A'))

        ax.view_init(elev=elev, azim=azim)

    # ── Callbacks ─────────────────────────────────────────────────────
    def on_march_change(_):
        old_dur = gait.step_duration
        new_dur = sl_dur.val
        if old_dur > 0:
            gait.t_phase = gait.t_phase * (new_dur / old_dur)
        gait.step_length   = sl_len.val / 100.
        gait.step_height   = sl_hgt.val / 100.
        gait.step_duration = new_dur
        gait.body_vel      = gait.step_length / gait.step_duration
        gait.angular_vel   = gait.yaw_per_step / gait.step_duration

    def on_imu_change(_):
        imu.roll_amp  = np.radians(sl_ramp.val)
        imu.pitch_amp = np.radians(sl_pamp.val)

    def on_pause(_):
        _paused[0] = not _paused[0]
        btn_pause.label.set_text('Pausar' if not _paused[0] else 'Reanudar')
        fig.canvas.draw_idle()

    def on_reset(_):
        gait.reset(); comp.reset()
        routine_player.reset()
        body_state[0]    = 'stand'
        _pending_state[0]= 'stand'
        _use_gait[0]     = True
        t_sim[0] = 0.; imu_last[0] = 0.; imu_last[1] = 0.
        step_counter[0] = 0
        if not autostart:
            _paused[0] = True
            btn_pause.label.set_text('▶ Iniciar')
        else:
            _paused[0] = False
            btn_pause.label.set_text('Pausar')

    def on_lie_down(_):
        if body_state[0] == 'stand' and not routine_player.active:
            _pending_state[0] = 'lie'
            _use_gait[0]      = False
            _paused[0]        = False
            routine_player.start(current_foot_pose(robot),
                                 routine_lie_down(robot))

    def on_stand_up(_):
        if body_state[0] == 'lie' and not routine_player.active:
            _pending_state[0] = 'stand'
            _paused[0]        = False
            routine_player.start(current_foot_pose(robot),
                                 routine_stand_up(robot))

    def on_key(event):
        d = {'up': DIR_FORWARD,    'w': DIR_FORWARD,
             'down': DIR_BACKWARD, 's': DIR_BACKWARD,
             'left': DIR_TURN_LEFT,'a': DIR_TURN_LEFT,
             'right':DIR_TURN_RIGHT,'d':DIR_TURN_RIGHT,
             ' ':    DIR_STOP,     'x': DIR_STOP}.get(event.key)
        if d is not None:
            gait.set_direction(d)

    for sl in (sl_len, sl_hgt, sl_dur): sl.on_changed(on_march_change)
    for sl in (sl_ramp, sl_pamp):       sl.on_changed(on_imu_change)
    btn_pause.on_clicked(on_pause)
    btn_reset.on_clicked(on_reset)
    btn_lie.on_clicked(on_lie_down)
    btn_stu.on_clicked(on_stand_up)
    btn_fwd.on_clicked(lambda _: gait.set_direction(DIR_FORWARD))
    btn_bwd.on_clicked(lambda _: gait.set_direction(DIR_BACKWARD))
    btn_lft.on_clicked(lambda _: gait.set_direction(DIR_TURN_LEFT))
    btn_rgt.on_clicked(lambda _: gait.set_direction(DIR_TURN_RIGHT))
    btn_stp.on_clicked(lambda _: gait.set_direction(DIR_STOP))
    fig.canvas.mpl_connect('key_press_event', on_key)

    ax.view_init(elev=28, azim=-55)
    anim = FuncAnimation(fig, update_frame, frames=frames_src,
                         interval=int(dt_anim * 1000), blit=False, repeat=False)
    plt.show()
    return anim

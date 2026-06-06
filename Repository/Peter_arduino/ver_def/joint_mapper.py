"""
joint_mapper.py — Control manual de ángulos articulares (FK directo)
═════════════════════════════════════════════════════════════════════
Herramienta de calibración y mapeo ángulo-simulación.

Mueve cada articulación con sliders y observa el resultado de la
cinemática DIRECTA (FK) en tiempo real. Útil para:
  • Verificar que la simulación coincida con el robot físico
  • Entender el mapeo de signos de cada servo
  • Encontrar la postura de reposo correcta
  • Ver hasta dónde llega cada pata con cada combinación de ángulos

Convención de ejes (frame del cuerpo):
    +X → Frente   +Y → Izquierda   +Z → Arriba

Disposición de patas:
    FL ──────── FR
    │  CUERPO   │
    RL ──────── RR

Ejecutar desde la carpeta ver_def/:
    python joint_mapper.py
"""

import sys
import os

_here   = os.path.dirname(os.path.abspath(__file__))   # …/ver_def
_parent = os.path.dirname(_here)                        # …/Python_simu
# ver_def/*.py usa 'from ver_def.X import' → necesita el padre en sys.path
# ver_def/robot_viz.py usa 'from robot_math import' → necesita ver_def también
for _p in (_parent, _here):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.patches as mpatches
from matplotlib.widgets import Slider, Button
import warnings
warnings.filterwarnings('ignore')

from robot_params     import RobotParams
from robot_kinematics import SpiderQuadruped


# ── Paleta visual (igual que el simulador principal) ───────────────────

LEG_COLORS = {
    'FL': '#00BFFF',
    'FR': '#FF6B35',
    'RL': '#7CFC00',
    'RR': '#FF1493',
}
BODY_COLOR  = '#2E4057'
BODY_ALPHA  = 0.35
JOINT_COLOR = '#FFFFFF'
LEG_ORDER   = ['FL', 'FR', 'RL', 'RR']


# ──────────────────────────────────────────────────────────────────────
#  PRIMITIVAS DE DIBUJO (replicadas para no depender de robot_viz)
# ──────────────────────────────────────────────────────────────────────

def _draw_body(ax, params):
    bx, by, bz = params.body_length/2, params.body_width/2, 0.015
    corners = np.array([
        [-bx,-by, bz],[ bx,-by, bz],[ bx, by, bz],[-bx, by, bz],
        [-bx,-by,-bz],[ bx,-by,-bz],[ bx, by,-bz],[-bx, by,-bz],
    ])
    faces = [
        [corners[0],corners[1],corners[2],corners[3]],
        [corners[4],corners[5],corners[6],corners[7]],
        [corners[0],corners[1],corners[5],corners[4]],
        [corners[2],corners[3],corners[7],corners[6]],
        [corners[0],corners[3],corners[7],corners[4]],
        [corners[1],corners[2],corners[6],corners[5]],
    ]
    ax.add_collection3d(Poly3DCollection(
        faces, facecolors=BODY_COLOR, edgecolors='#4A6FA5',
        alpha=BODY_ALPHA, linewidths=1.2))
    ax.quiver(0, 0, bz+0.005, 0.04, 0, 0,
              color='#FFD700', linewidth=2, arrow_length_ratio=0.4)
    ax.text(0.06, 0, bz+0.006, 'FRENTE', color='#FFD700',
            fontsize=9, fontweight='bold', va='center')


def _draw_leg(ax, fk_data, name):
    color = LEG_COLORS[name]
    pts   = [fk_data['coxa'], fk_data['femur'], fk_data['tibia'], fk_data['foot']]
    for p0, p1 in zip(pts, pts[1:]):
        ax.plot([p0[0],p1[0]], [p0[1],p1[1]], [p0[2],p1[2]],
                color=color, linewidth=3.5, solid_capstyle='round')
    sizes   = [60, 45, 35, 90]
    markers = ['o', 'o', 'o', '^']
    for pt, sz, mk in zip(pts, sizes, markers):
        ax.scatter(*pt, s=sz, c=JOINT_COLOR, marker=mk,
                   edgecolors=color, linewidths=1.5, zorder=5)
    foot = fk_data['foot']
    ax.text(foot[0], foot[1], foot[2] - 0.016, name,
            fontsize=11, color=color, ha='center', fontweight='bold')


# ──────────────────────────────────────────────────────────────────────
#  HERRAMIENTA PRINCIPAL
# ──────────────────────────────────────────────────────────────────────

def visualize_joint_mapper(robot: SpiderQuadruped):
    """
    Ventana interactiva: 12 sliders (Coxa/Fémur/Tibia × 4 patas) →
    FK directo → visualización 3D.

    El ángulo se aplica directamente a leg.q sin pasar por IK,
    así puedes explorar cualquier combinación incluso fuera del
    workspace normal del robot.
    """
    robot.rest_pose()   # inicializa leg.q en todos los ejes

    fig = plt.figure(figsize=(17, 11), facecolor='#0D1117')
    fig.suptitle(
        'Joint Mapper  ─  Cinemática Directa (FK)  ─  mover sliders = mover robot',
        color='#C8D8FF', fontsize=15, fontweight='bold', y=0.985)

    # ── Vista 3D ──────────────────────────────────────────────────────
    ax = fig.add_axes([0.04, 0.40, 0.92, 0.56], projection='3d')
    ax.set_facecolor('#0D1117')

    p      = robot.p
    sl_kw  = dict(facecolor='#141824')

    # ── Layout de sliders ─────────────────────────────────────────────
    # 4 columnas (una por pata), 3 filas (Coxa, Fémur, Tibia)
    #
    #  x posición de cada columna (izquierda del slider)
    col_x = {'FL': 0.04, 'FR': 0.285, 'RL': 0.530, 'RR': 0.775}
    sl_w  = 0.195   # ancho de cada slider
    sl_h  = 0.025   # altura

    #  y posición de cada fila (bottom del axes del slider)
    row_y = {'coxa': 0.285, 'femur': 0.195, 'tibia': 0.105}

    joint_cfg = [
        ('coxa',  p.coxa_lim,  '#5599DD', 'Coxa  (yaw)'),
        ('femur', p.femur_lim, '#DD9933', 'Fémur (pitch)'),
        ('tibia', p.tibia_lim, '#CC4477', 'Tibia (pitch)'),
    ]

    # Etiquetas de pata (encima de cada columna)
    for name in LEG_ORDER:
        fig.text(col_x[name] + sl_w/2, 0.370,
                 name,
                 ha='center', color=LEG_COLORS[name],
                 fontsize=16, fontweight='bold')

    # Etiquetas de articulación (a la izquierda, una sola vez)
    for jname, _, jcol, jlabel in joint_cfg:
        fig.text(0.002, row_y[jname] + sl_h/2, jlabel,
                 va='center', color=jcol, fontsize=11, fontweight='bold')

    # Crear sliders ── dict[leg][joint_idx]
    sliders = {}
    for leg_name in LEG_ORDER:
        leg    = robot.legs[leg_name]
        q0_deg = np.degrees(leg.q)
        row = []
        for j, (jname, (lo, hi), jcol, _) in enumerate(joint_cfg):
            ax_sl = fig.add_axes(
                [col_x[leg_name], row_y[jname], sl_w, sl_h], **sl_kw)
            sl = Slider(ax_sl, '', lo, hi,
                        valinit=q0_deg[j], valfmt='%+.1f°', color=jcol)
            sl.label.set_color('#C8D8FF')
            sl.valtext.set_color('#C8D8FF')
            sl.valtext.set_fontsize(12)
            row.append(sl)
        sliders[leg_name] = row

    # ── Botones ────────────────────────────────────────────────────────
    btn_kw   = dict(color='#2E4057', hovercolor='#4A6FA5')
    btn_rest = Button(
        fig.add_axes([0.04,  0.022, 0.13, 0.040], facecolor='#2E4057'),
        '↺  Reposo', **btn_kw)
    btn_zero = Button(
        fig.add_axes([0.195, 0.022, 0.13, 0.040], facecolor='#2E4057'),
        '○  Todo 0°', **btn_kw)
    btn_prnt = Button(
        fig.add_axes([0.350, 0.022, 0.17, 0.040], facecolor='#1A3050'),
        '⎙  Imprimir ángulos', color='#1A3050', hovercolor='#2A5070')
    for btn in (btn_rest, btn_zero, btn_prnt):
        btn.label.set_color('#C8D8FF')
        btn.label.set_fontsize(12)

    # ── Panel de convención de signos ─────────────────────────────────
    fig.text(
        0.545, 0.075,
        "Convención de signos  (frame de cada pata, eje radial = hacia afuera)\n"
        "  Coxa   (+) → gira hacia +Y del cuerpo  (izquierda global)\n"
        "  Fémur  (+) → sube el fémur  (+Z)    (−) → baja\n"
        "  Tibia  (−) → dobla la rodilla hacia abajo  (postura normal < 0)\n"
        "  Cero coxa = pie apuntando al frente del frame de su pata\n"
        "  Cero fémur/tibia = segmentos horizontales (pata extendida lateral)",
        ha='left', va='top',
        color='#7799AA', fontsize=10, fontfamily='monospace',
        bbox=dict(boxstyle='round,pad=0.5', facecolor='#0C1018',
                  alpha=0.90, edgecolor='#223344'))

    # ── Función de redibujado ─────────────────────────────────────────
    def redraw():
        elev, azim = ax.elev, ax.azim
        ax.cla()
        ax.set_facecolor('#0D1117')

        # Aplicar ángulos de los sliders directamente a leg.q (FK puro)
        for leg_name in LEG_ORDER:
            robot.legs[leg_name].q = np.radians(
                [sl.val for sl in sliders[leg_name]])

        # Dibujar robot
        _draw_body(ax, p)
        all_fk = robot.get_all_fk()
        for leg_name, fk_data in all_fk.items():
            _draw_leg(ax, fk_data, leg_name)

        # Suelo y proyecciones de pies
        z_floor = -0.23
        lim = 0.32
        xx, yy = np.meshgrid([-lim, lim], [-lim, lim])
        ax.plot_surface(xx, yy, np.full_like(xx, z_floor),
                        alpha=0.07, color='#4A6FA5')
        for leg_name, fk_data in all_fk.items():
            foot = fk_data['foot']
            ax.plot([foot[0]]*2, [foot[1]]*2, [foot[2], z_floor],
                    '--', color=LEG_COLORS[leg_name], alpha=0.35, linewidth=1)
            ax.scatter(foot[0], foot[1], z_floor,
                       s=25, c=LEG_COLORS[leg_name], alpha=0.5, marker='x')

        # Tabla de ángulos + posición del pie (esquina superior izquierda)
        header = (f"  {'Pata':<4}  {'Coxa':>8}  {'Fémur':>8}  {'Tibia':>8}"
                  f"   {'Pie X':>7} {'Pie Y':>7} {'Pie Z':>7}")
        rows = [header, '  ' + '─'*62]
        for leg_name in LEG_ORDER:
            leg   = robot.legs[leg_name]
            q_deg = np.degrees(leg.q)
            foot  = all_fk[leg_name]['foot'] * 100   # → cm
            rows.append(
                f"  {leg_name:<4}  {q_deg[0]:>+7.1f}°  {q_deg[1]:>+7.1f}°  {q_deg[2]:>+7.1f}°"
                f"   {foot[0]:>+6.1f}  {foot[1]:>+6.1f}  {foot[2]:>+6.1f} cm"
            )
        ax.text2D(0.01, 0.99, '\n'.join(rows),
                  transform=ax.transAxes, ha='left', va='top',
                  color='#99BBDD', fontsize=10, fontfamily='monospace',
                  bbox=dict(boxstyle='round,pad=0.35', facecolor='#0A0E18',
                            alpha=0.85, edgecolor='#1E2A3A'))

        # Ejes y estilo
        ax.set_xlim(-0.32, 0.32)
        ax.set_ylim(-0.32, 0.32)
        ax.set_zlim(-0.25, 0.18)
        for spine in ['x','y','z']:
            ax.tick_params(axis=spine, colors='#555577', labelsize=10)
        ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.set_edgecolor('#1E2233')
        ax.grid(color='#1E2233', linestyle='--', linewidth=0.5, alpha=0.6)
        ax.set_xlabel('X  (frente →)', color='#8899BB', fontsize=11, labelpad=6)
        ax.set_ylabel('Y  (← izq)',    color='#8899BB', fontsize=11, labelpad=6)
        ax.set_zlabel('Z  (↑ arriba)', color='#8899BB', fontsize=11, labelpad=6)
        patches = [mpatches.Patch(color=c, label=n) for n, c in LEG_COLORS.items()]
        ax.legend(handles=patches, loc='upper right',
                  facecolor='#1A1E2E', edgecolor='#4A6FA5',
                  labelcolor='white', fontsize=12)
        ax.view_init(elev=elev, azim=azim)
        fig.canvas.draw_idle()

    # ── Callbacks ─────────────────────────────────────────────────────

    def on_change(_):
        redraw()

    def on_reset_rest(_):
        robot.rest_pose()
        for leg_name in LEG_ORDER:
            q0 = np.degrees(robot.legs[leg_name].q)
            for j, sl in enumerate(sliders[leg_name]):
                sl.set_val(q0[j])

    def on_zero(_):
        for leg_name in LEG_ORDER:
            for sl in sliders[leg_name]:
                sl.set_val(0.0)

    def on_print(_):
        all_fk = robot.get_all_fk()
        print('\n' + '='*72)
        print(f"  {'Pata':<5}  {'Coxa':>9}  {'Fémur':>9}  {'Tibia':>9}"
              f"  {'Pie X (cm)':>10}  {'Pie Y (cm)':>10}  {'Pie Z (cm)':>10}")
        print('  ' + '-'*68)
        for leg_name in LEG_ORDER:
            q_deg = np.degrees(robot.legs[leg_name].q)
            foot  = all_fk[leg_name]['foot'] * 100
            print(f"  {leg_name:<5}  {q_deg[0]:>+8.2f}°  {q_deg[1]:>+8.2f}°  {q_deg[2]:>+8.2f}°"
                  f"  {foot[0]:>+10.2f}  {foot[1]:>+10.2f}  {foot[2]:>+10.2f}")
        print('='*72 + '\n')

    for leg_name in LEG_ORDER:
        for sl in sliders[leg_name]:
            sl.on_changed(on_change)
    btn_rest.on_clicked(on_reset_rest)
    btn_zero.on_clicked(on_zero)
    btn_prnt.on_clicked(on_print)

    ax.view_init(elev=25, azim=-55)
    redraw()
    plt.show()


# ──────────────────────────────────────────────────────────────────────
#  PUNTO DE ENTRADA
# ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    params = RobotParams(
        body_length = 0.20,
        body_width  = 0.14,
        L_coxa      = 0.05,
        L_femur     = 0.10,
        L_tibia     = 0.12,
        rest_x      = 0.11,
        rest_z      = -0.10,
    )
    robot = SpiderQuadruped(params)
    visualize_joint_mapper(robot)

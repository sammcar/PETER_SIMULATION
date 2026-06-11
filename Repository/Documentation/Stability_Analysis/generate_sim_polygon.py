#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dynamic Gait Cycle and Support Polygon Simulator (Publication Quality).
Animates telemetry from stability_log.csv, rendering the McGhee-Frank (1968)
Stability Margin as a perpendicular arrow from CoM to the critical support edge.
"""

import os
import math
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon, Circle, FancyArrowPatch

# ── 1. PUBLICATION STYLE ──────────────────────────────────────────────────────
def set_publication_style():
    plt.rcParams.update({
        'font.family': 'serif',
        'font.size': 10,
        'axes.labelsize': 11,
        'axes.titlesize': 12,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'legend.fontsize': 9,
        'figure.dpi': 120,
        'axes.grid': True,
        'grid.alpha': 0.3,
        'grid.linestyle': '--',
        'axes.spines.top': False,
        'axes.spines.right': False,
    })
    sns.set_palette("colorblind")


# ── 2. GEOMETRY UTILITIES ─────────────────────────────────────────────────────
def _sort_ccw(pts):
    """Sort 2-D points counter-clockwise around their centroid."""
    if pts.shape[0] < 3:
        return pts
    cx, cy = np.mean(pts, axis=0)
    angles = np.arctan2(pts[:, 1] - cy, pts[:, 0] - cx)
    return pts[np.argsort(angles)]


def _tr_color(tr):
    """Map tipover risk to a traffic-light colour."""
    if tr < 0.8:
        return '#4CAF50'   # green  – stable
    if tr < 1.0:
        return '#FFC107'   # yellow – alert
    return '#F44336'       # red    – critical


def _foot_stability(grounded_pts, com=(0.0, 0.0)):
    """
    Compute McGhee-Frank stability geometry from 3 CCW-ordered foot positions.

    Returns a dict with:
      sm          – Stability Margin [m]: min perpendicular distance CoM → edge
      r_in        – Inradius of the support triangle [m]
      incenter    – (x, y) of the inscribed circle centre
      nearest     – index of the critical (nearest) edge
      nearest_edge– ((x1,y1), (x2,y2)) of the critical edge
      sm_foot     – foot of the perpendicular from CoM to the critical edge
    Returns None if fewer than 3 grounded feet.
    """
    if len(grounded_pts) != 3:   # SM only valid for exactly 3 feet (McGhee-Frank 1968)
        return None

    a, b, c = grounded_pts[0], grounded_pts[1], grounded_pts[2]
    # Enforce CCW orientation
    if (b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1]) <= 0:
        b, c = c, b

    sides = [
        math.hypot(b[0]-a[0], b[1]-a[1]),   # |AB|
        math.hypot(c[0]-b[0], c[1]-b[1]),   # |BC|
        math.hypot(a[0]-c[0], a[1]-c[1]),   # |CA|
    ]
    perim = sum(sides)
    area  = abs((a[0]*(b[1]-c[1]) + b[0]*(c[1]-a[1]) + c[0]*(a[1]-b[1])) / 2.0)
    r_in  = (2.0 * area / perim) if perim > 1e-9 else 1e-9

    # Incenter: weighted by the length of the opposite side (same as ROS reference)
    incenter = (
        (sides[1]*a[0] + sides[2]*b[0] + sides[0]*c[0]) / perim,
        (sides[1]*a[1] + sides[2]*b[1] + sides[0]*c[1]) / perim,
    )

    # Signed perpendicular distance from CoM to each directed edge
    # Positive = CoM is to the LEFT of the edge (inside a CCW polygon)
    edges = [(a, b), (b, c), (c, a)]
    dists = []
    for pi, pj in edges:
        num = (pj[0]-pi[0])*(com[1]-pi[1]) - (pj[1]-pi[1])*(com[0]-pi[0])
        den = math.hypot(pj[0]-pi[0], pj[1]-pi[1])
        dists.append(num / den if den > 1e-9 else 0.0)

    nearest = int(np.argmin(dists))
    pi, pj  = edges[nearest]

    # Foot of the perpendicular from CoM onto the critical edge
    ex, ey = pj[0]-pi[0], pj[1]-pi[1]
    t = ((com[0]-pi[0])*ex + (com[1]-pi[1])*ey) / (ex**2 + ey**2 + 1e-12)
    sm_foot = (pi[0] + t*ex, pi[1] + t*ey)

    return {
        'sm': dists[nearest],
        'r_in': r_in,
        'incenter': incenter,
        'edges': edges,
        'nearest': nearest,
        'nearest_edge': edges[nearest],
        'sm_foot': sm_foot,
    }


# ── 3. ANIMATOR ───────────────────────────────────────────────────────────────
class AcademicGaitAnimator:

    def __init__(self, csv_path):
        if not os.path.exists(csv_path):
            print(f"Error: telemetry log not found at '{csv_path}'")
            sys.exit(1)
        print(f"Loading experimental records from: {csv_path}")
        self.df = pd.read_csv(csv_path)
        self.frames = []
        self._parse_csv_telemetry()

    def _parse_csv_telemetry(self):
        for _, row in self.df.iterrows():
            feet_all = np.array([
                [row['foot_RU_x'], row['foot_RU_y']],
                [row['foot_LU_x'], row['foot_LU_y']],
                [row['foot_RD_x'], row['foot_RD_y']],
                [row['foot_LD_x'], row['foot_LD_y']],
            ], dtype=np.float64)

            mode          = str(row.get('mode', 'C')).upper()
            leg_in_air_idx = int(row['leg_in_air'])

            on_ground = [True] * 4
            if mode == 'C' and 0 <= leg_in_air_idx < 4:
                on_ground[leg_in_air_idx] = False

            grounded_list = [feet_all[i] for i in range(4) if     on_ground[i]]
            swing_list    = [feet_all[i] for i in range(4) if not on_ground[i]]

            grounded_pts = (np.array(grounded_list, dtype=np.float64).reshape(-1, 2)
                            if grounded_list else np.empty((0, 2)))
            swing_pts    = (np.array(swing_list,    dtype=np.float64).reshape(-1, 2)
                            if swing_list    else np.empty((0, 2)))

            if len(grounded_pts) >= 3:
                grounded_pts = _sort_ccw(grounded_pts)

            self.frames.append({
                'timestamp':   float(row['timestamp']),
                'mode':        mode,
                'feet_all':    feet_all,
                'grounded_pts': grounded_pts,
                'swing_pts':   swing_pts,
                'sm':          float(row['SM']),
                'sm_norm':     float(row['SM_norm']),
                'tr':          float(row['TR']),
                'area':        float(row['triangle_area']),
                'inradius':    float(row['inradius']),
            })
        print(f"Successfully compiled {len(self.frames)} animation frames.")

    # ── init ──────────────────────────────────────────────────────────────────
    def init_plot(self):
        self.line_poly.set_data([], [])
        self.critical_edge_line.set_data([], [])
        self.poly_patch.set_xy(np.empty((0, 2)))
        self.incenter_circle.set_radius(0)
        self.sm_arrow.set_positions((-10, -10), (-10, -10))
        self.sm_arrow.set_visible(False)
        self.sm_label.set_text('')
        self.sm_label.set_visible(False)
        self.incenter_marker.set_offsets(np.empty((0, 2)))
        self.stance_feet_marker.set_offsets(np.empty((0, 2)))
        self.swing_foot_marker.set_offsets(np.empty((0, 2)))
        self.com_marker.set_offsets(np.empty((0, 2)))
        # self.geo_center_marker.set_offsets(np.empty((0, 2)))
        self.tr_badge.set_text('')
        self.tr_badge.set_visible(False)
        self.text_metrics.set_text('')
        return self._all_artists()

    # ── per-frame update ──────────────────────────────────────────────────────
    def update_frame(self, frame_idx):
        frame = self.frames[frame_idx]
        com   = (0.0, 0.0)

        # Foot markers
        self.stance_feet_marker.set_offsets(np.empty((0, 2)))
        self.swing_foot_marker.set_offsets(np.empty((0, 2)))
        if len(frame['grounded_pts']) > 0:
            self.stance_feet_marker.set_offsets(np.asarray(frame['grounded_pts']))
        if len(frame['swing_pts']) > 0:
            self.swing_foot_marker.set_offsets(np.asarray(frame['swing_pts']))

        # CoM fixed at body-frame origin
        self.com_marker.set_offsets(np.array([[0.0, 0.0]]))

        # Geometric centre of grounded feet
        # if len(frame['grounded_pts']) > 0:
        #     gc = np.mean(frame['grounded_pts'], axis=0)
        #     self.geo_center_marker.set_offsets(np.array([gc]))
        # else:
        #     self.geo_center_marker.set_offsets(np.empty((0, 2)))

        if len(frame['grounded_pts']) >= 3:
            pts    = np.asarray(frame['grounded_pts'])
            closed = np.vstack([pts, pts[0]])

            # Support polygon boundary and colour-coded fill
            self.line_poly.set_data(closed[:, 0], closed[:, 1])
            self.poly_patch.set_xy(pts)
            self.poly_patch.set_facecolor(_tr_color(frame['tr']))
            self.poly_patch.set_visible(True)

            if len(frame['grounded_pts']) == 3:
                # SM only meaningful during active crawl (exactly 3 feet on ground)
                stab = _foot_stability(list(frame['grounded_pts']), com)

                if stab is not None and stab['sm'] > 0:
                    # Critical edge highlighted in red
                    p1, p2 = stab['nearest_edge']
                    self.critical_edge_line.set_data([p1[0], p2[0]], [p1[1], p2[1]])
                    self.critical_edge_line.set_visible(True)

                    # SM arrow: CoM → foot of perpendicular on the critical edge
                    foot = stab['sm_foot']
                    self.sm_arrow.set_positions(com, foot)
                    self.sm_arrow.set_visible(True)

                    # SM label at arrow midpoint
                    mx = (com[0] + foot[0]) / 2.0 + 0.015
                    my = (com[1] + foot[1]) / 2.0 + 0.015
                    self.sm_label.set_position((mx, my))
                    self.sm_label.set_text(f'SM = {stab["sm"]*100:.1f} cm')
                    self.sm_label.set_visible(True)

                    # Inscribed circle at the incenter
                    self.incenter_circle.set_center(stab['incenter'])
                    self.incenter_circle.set_radius(stab['r_in'])
                    self.incenter_circle.set_visible(True)

                    # Incenter marker
                    self.incenter_marker.set_offsets(np.array([stab['incenter']]))

                else:
                    self._hide_stability_artists()

                # TR badge
                self.tr_badge.set_text(f'TR = {frame["tr"]:.3f}')
                self.tr_badge.get_bbox_patch().set_facecolor(_tr_color(frame['tr']))
                self.tr_badge.set_visible(True)

            else:
                # 4 feet on ground: static stance, SM not applicable
                self._hide_stability_artists()
                self.tr_badge.set_text('Static stance')
                self.tr_badge.get_bbox_patch().set_facecolor('#90CAF9')
                self.tr_badge.set_visible(True)

        else:
            self.line_poly.set_data([], [])
            self.poly_patch.set_visible(False)
            self._hide_stability_artists()
            self.tr_badge.set_visible(False)

        # Metrics text box
        self.text_metrics.set_text(
            f"Time Stage    : {frame['timestamp']:.2f} s\n"
            f"Active Mode   : {frame['mode']}\n"
            f"Grounded Legs : {len(frame['grounded_pts'])}\n"
            f"Support Area  : {frame['area']:.4f} m²\n"
            f"Stability Marg: {frame['sm']:.4f} m\n"
            f"Tipover Risk  : {frame['tr']:.2f}\n"
            f"Status        : {'STABLE' if frame['sm'] > 0 else 'TRANSITION'}"
        )
        return self._all_artists()

    def _hide_stability_artists(self):
        self.critical_edge_line.set_data([], [])
        self.critical_edge_line.set_visible(False)
        self.sm_arrow.set_positions((-10, -10), (-10, -10))
        self.sm_arrow.set_visible(False)
        self.sm_label.set_visible(False)
        self.incenter_circle.set_visible(False)
        self.incenter_marker.set_offsets(np.empty((0, 2)))

    def _all_artists(self):
        return (
            self.line_poly, self.critical_edge_line,
            self.poly_patch, self.incenter_circle, self.sm_arrow,
            self.sm_label, self.incenter_marker,
            self.stance_feet_marker, self.swing_foot_marker,
            self.com_marker,  # self.geo_center_marker,
            self.tr_badge, self.text_metrics,
        )

    # ── figure setup and animation launch ─────────────────────────────────────
    def run(self):
        timestamps    = [f['timestamp'] for f in self.frames]
        mean_interval = float(np.mean(np.diff(timestamps)) * 1000.0) if len(timestamps) > 1 else 50.0

        self.fig, self.ax = plt.subplots(figsize=(6.5, 6.5))
        self.ax.set_xlim(-0.4, 0.4)
        self.ax.set_ylim(-0.4, 0.4)
        self.ax.set_xlabel("Body Width Coordinate X [m]", fontweight='bold')
        self.ax.set_ylabel("Body Length Coordinate Y [m]", fontweight='bold')
        self.ax.set_aspect('equal', adjustable='box')

        # Patches (drawn first, underneath everything else)
        self.poly_patch = Polygon(np.empty((0, 2)), facecolor='#4CAF50',
                                  edgecolor='none', alpha=0.15)
        self.ax.add_patch(self.poly_patch)

        self.incenter_circle = Circle((0, 0), 0, fill=False, ls='--',
                                      color='#546E7A', lw=1.0, alpha=0.75, zorder=2,
                                      label='Inscribed Circle ($r_{in}$)')
        self.ax.add_patch(self.incenter_circle)

        self.sm_arrow = FancyArrowPatch(
            (-10, -10), (-10, -10),
            arrowstyle='->', color='#D32F2F',
            lw=1.5, mutation_scale=12, zorder=5,
        )
        self.ax.add_patch(self.sm_arrow)

        # Lines
        self.line_poly, = self.ax.plot(
            [], [], color='#1565C0', lw=1.8, zorder=3, label='Support Polygon')
        self.critical_edge_line, = self.ax.plot(
            [], [], color='#D32F2F', lw=2.5, zorder=4, label='Critical Edge')

        # Scatter markers
        self.incenter_marker    = self.ax.scatter(
            [], [], color='#546E7A', s=60, marker='+', linewidths=1.8,
            zorder=5, label='Incenter')
        self.stance_feet_marker = self.ax.scatter(
            [], [], color='#388e3c', s=100, marker='o', edgecolors='black',
            zorder=6, label='Stance Feet')
        self.swing_foot_marker  = self.ax.scatter(
            [], [], color='#f57c00', s=100, marker='X', edgecolors='black',
            zorder=6, label='Swing Foot')
        self.com_marker         = self.ax.scatter(
            [], [], color='#d32f2f', s=160, marker='P', edgecolors='black',
            zorder=7, label='CoM (base_link)')
        # self.geo_center_marker  = self.ax.scatter(
        #     [], [], color='#7b1fa2', s=120, marker='D', edgecolors='black',
        #     zorder=7, label='Geometric Center')

        # Text elements
        self.sm_label = self.ax.text(
            0, 0, '', fontsize=8, color='#D32F2F',
            ha='center', fontweight='bold', zorder=6)
        self.tr_badge = self.ax.text(
            0.03, 0.97, '', transform=self.ax.transAxes,
            fontsize=10, fontweight='bold', va='top', zorder=8,
            bbox=dict(boxstyle='round,pad=0.3', fc='#4CAF50', alpha=0.85))
        self.text_metrics = self.ax.text(
            -0.38, -0.38, '',
            fontsize=9, fontfamily='monospace',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#f5f5f5',
                      edgecolor='gray', alpha=0.9))

        self.ax.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='none')
        plt.title("PETER Hybrid Locomotion — Stability Analysis",
                  pad=20, fontsize=12, fontweight='bold')

        print(f"Launching simulation ({len(self.frames)} frames)...")
        anim = FuncAnimation(   # noqa: F841
            self.fig, self.update_frame,
            frames=len(self.frames),
            init_func=self.init_plot,
            interval=mean_interval,
            blit=True, repeat=True,
        )
        plt.show()


# ── 4. ENTRY POINT ────────────────────────────────────────────────────────────
if __name__ == '__main__':
    set_publication_style()

    script_dir       = os.path.dirname(os.path.abspath(__file__))
    default_log_path = os.path.join(script_dir, 'cvs')

    if os.path.exists(default_log_path):
        files = [os.path.join(default_log_path, f)
                 for f in os.listdir(default_log_path) if f.endswith('.csv')]
        target_csv = (max(files, key=os.path.getctime) if files
                      else os.path.join(default_log_path, 'stability_session_report_mode_changes.csv'))
    else:
        target_csv = os.path.join(default_log_path, 'stability_session_report_mode_changes.csv')

    if len(sys.argv) > 1:
        target_csv = sys.argv[1]

    animator = AcademicGaitAnimator(target_csv)
    animator.run()

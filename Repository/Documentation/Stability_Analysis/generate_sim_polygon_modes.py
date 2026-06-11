#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mode-Transition Stability Simulator (Publication Quality).
Variant of generate_sim_polygon.py for stability_session_report_mode_changes.csv.

Key differences vs. the original:
  - All 4 feet are always on the ground (leg_in_air = -1).
  - SM/TR/inradius in the CSV are sentinel values; all metrics are
    recomputed at render time from the actual foot positions.
  - The McGhee-Frank SM is generalised to the 4-foot convex polygon:
      SM = min perpendicular distance from CoM to any polygon edge.
  - The active locomotion mode (C / H / X) is shown as a colour-coded badge.
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
    """Traffic-light colour by tipover risk."""
    if tr < 0.8:
        return '#4CAF50'
    if tr < 1.0:
        return '#FFC107'
    return '#F44336'


MODE_COLORS = {
    'C': '#1565C0',   # blue   – crawl
    'H': '#E65100',   # orange – hybrid
    'X': '#6A1B9A',   # purple – cross/trot
}


def _foot_stability(grounded_pts, com=(0.0, 0.0)):
    """
    Generalised McGhee-Frank stability geometry for N >= 3 grounded feet.

    For 3 feet  → triangle incenter and inradius (exact).
    For 4 feet  → polygon centroid and 2·area/perimeter as r_in approximation.

    Returns a dict with:
      sm           – min perpendicular distance from CoM to any polygon edge [m]
      sm_norm      – sm / r_in
      tr           – tipover risk = 1 − sm_norm
      r_in         – inradius (or approx. for quadrilateral)
      center       – incenter (triangle) or centroid (quadrilateral)
      nearest_edge – ((x1,y1), (x2,y2)) of the critical edge
      sm_foot      – foot of perpendicular from CoM to the critical edge
      area         – support polygon area [m²]
    Returns None if fewer than 3 grounded feet.
    """
    n = len(grounded_pts)
    if n < 3:
        return None

    pts = _sort_ccw(np.asarray(grounded_pts, dtype=np.float64))

    # ── Polygon area (shoelace) and perimeter ─────────────────────────────────
    area = 0.0
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        area += a[0] * b[1] - b[0] * a[1]
    area  = abs(area) / 2.0
    perim = sum(
        math.hypot(pts[i][0] - pts[(i+1)%n][0],
                   pts[i][1] - pts[(i+1)%n][1])
        for i in range(n)
    )

    # ── Polygon centre ────────────────────────────────────────────────────────
    if n == 3:
        # Exact incenter for triangle: weighted by opposite side length
        a_, b_, c_ = pts[0], pts[1], pts[2]
        sides = [
            math.hypot(b_[0]-a_[0], b_[1]-a_[1]),
            math.hypot(c_[0]-b_[0], c_[1]-b_[1]),
            math.hypot(a_[0]-c_[0], a_[1]-c_[1]),
        ]
        center = (
            (sides[1]*a_[0] + sides[2]*b_[0] + sides[0]*c_[0]) / perim,
            (sides[1]*a_[1] + sides[2]*b_[1] + sides[0]*c_[1]) / perim,
        )
        r_in = (2.0 * area / perim) if perim > 1e-9 else 1e-9
    else:
        # Centroid for quadrilateral
        center = (float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1])))
        # r_in = min perpendicular distance from centroid to any edge
        # This guarantees the inscribed circle stays strictly inside the polygon
        cx, cy = center
        edges_tmp = [(pts[i], pts[(i+1) % n]) for i in range(n)]
        r_in = min(
            abs((pj[0]-pi[0])*(cy-pi[1]) - (pj[1]-pi[1])*(cx-pi[0]))
            / math.hypot(pj[0]-pi[0], pj[1]-pi[1])
            for pi, pj in edges_tmp
        )

    # ── Signed distances from CoM to each directed edge ───────────────────────
    edges = [(pts[i], pts[(i + 1) % n]) for i in range(n)]
    dists = []
    for pi, pj in edges:
        num = (pj[0]-pi[0])*(com[1]-pi[1]) - (pj[1]-pi[1])*(com[0]-pi[0])
        den = math.hypot(pj[0]-pi[0], pj[1]-pi[1])
        dists.append(num / den if den > 1e-9 else 0.0)

    nearest = int(np.argmin(dists))
    sm      = dists[nearest]
    sm_norm = sm / r_in
    tr      = 1.0 - sm_norm

    pi, pj = edges[nearest]
    ex, ey = pj[0]-pi[0], pj[1]-pi[1]
    t = ((com[0]-pi[0])*ex + (com[1]-pi[1])*ey) / (ex**2 + ey**2 + 1e-12)
    sm_foot = (pi[0] + t*ex, pi[1] + t*ey)

    return {
        'sm': sm, 'sm_norm': sm_norm, 'tr': tr,
        'r_in': r_in, 'center': center,
        'edges': edges, 'nearest': nearest,
        'nearest_edge': edges[nearest],
        'sm_foot': sm_foot,
        'area': area,
    }


# ── 3. ANIMATOR ───────────────────────────────────────────────────────────────
class ModeTransitionAnimator:

    def __init__(self, csv_path):
        if not os.path.exists(csv_path):
            print(f"Error: telemetry log not found at '{csv_path}'")
            sys.exit(1)
        print(f"Loading records from: {csv_path}")
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

            mode = str(row.get('mode', 'C')).upper()
            leg_in_air_idx = int(row['leg_in_air'])

            # Determine which feet are on the ground
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

            # Recompute stability metrics from geometry (CSV values are sentinels)
            stab = _foot_stability(list(grounded_pts))
            sm       = stab['sm']       if stab else 0.0
            sm_norm  = stab['sm_norm']  if stab else 0.0
            tr       = stab['tr']       if stab else 1.0
            area     = stab['area']     if stab else float(row['triangle_area'])
            inradius = stab['r_in']     if stab else float(row['inradius'])

            self.frames.append({
                'timestamp':    float(row['timestamp']),
                'mode':         mode,
                'feet_all':     feet_all,
                'grounded_pts': grounded_pts,
                'swing_pts':    swing_pts,
                'sm':           sm,
                'sm_norm':      sm_norm,
                'tr':           tr,
                'area':         area,
                'inradius':     inradius,
            })
        print(f"Compiled {len(self.frames)} animation frames.")

    # ── init ──────────────────────────────────────────────────────────────────
    def init_plot(self):
        self.line_poly.set_data([], [])
        self.critical_edge_line.set_data([], [])
        self.poly_patch.set_xy(np.empty((0, 2)))
        self.center_circle.set_radius(0)
        self.sm_arrow.set_positions((-10, -10), (-10, -10))
        self.sm_arrow.set_visible(False)
        self.sm_label.set_text('')
        self.sm_label.set_visible(False)
        self.center_marker.set_offsets(np.empty((0, 2)))
        self.stance_feet_marker.set_offsets(np.empty((0, 2)))
        self.swing_foot_marker.set_offsets(np.empty((0, 2)))
        self.com_marker.set_offsets(np.empty((0, 2)))
        self.tr_badge.set_text('')
        self.tr_badge.set_visible(False)
        self.mode_badge.set_text('')
        self.mode_badge.set_visible(False)
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

        # Mode badge
        mode_color = MODE_COLORS.get(frame['mode'], '#546E7A')
        self.mode_badge.set_text(f"Mode: {frame['mode']}")
        self.mode_badge.get_bbox_patch().set_facecolor(mode_color)
        self.mode_badge.set_visible(True)

        n_grounded = len(frame['grounded_pts'])

        if n_grounded >= 3:
            pts    = np.asarray(frame['grounded_pts'])
            closed = np.vstack([pts, pts[0]])

            # Support polygon — colour by TR
            self.line_poly.set_data(closed[:, 0], closed[:, 1])
            self.poly_patch.set_xy(pts)
            self.poly_patch.set_facecolor(_tr_color(frame['tr']))
            self.poly_patch.set_visible(True)

            # Stability geometry (valid for 3 or 4 feet)
            stab = _foot_stability(list(frame['grounded_pts']), com)

            if stab is not None and stab['sm'] > 0:
                # Critical edge
                p1, p2 = stab['nearest_edge']
                self.critical_edge_line.set_data([p1[0], p2[0]], [p1[1], p2[1]])
                self.critical_edge_line.set_visible(True)

                # SM arrow: CoM → foot of perpendicular on critical edge
                foot = stab['sm_foot']
                self.sm_arrow.set_positions(com, foot)
                self.sm_arrow.set_visible(True)

                # SM label at midpoint
                mx = (com[0] + foot[0]) / 2.0 + 0.015
                my = (com[1] + foot[1]) / 2.0 + 0.015
                self.sm_label.set_position((mx, my))
                self.sm_label.set_text(f'SM = {stab["sm"]*100:.1f} cm')
                self.sm_label.set_visible(True)

                # Inscribed / inradius circle at polygon centre
                self.center_circle.set_center(stab['center'])
                self.center_circle.set_radius(stab['r_in'])
                self.center_circle.set_visible(True)

                # Centre marker
                self.center_marker.set_offsets(np.array([stab['center']]))

                # TR badge
                self.tr_badge.set_text(f'TR = {stab["tr"]:.3f}')
                self.tr_badge.get_bbox_patch().set_facecolor(_tr_color(stab['tr']))
                self.tr_badge.set_visible(True)

                # Update frame metrics with recomputed values
                frame['sm']   = stab['sm']
                frame['tr']   = stab['tr']
                frame['area'] = stab['area']

            else:
                self._hide_stability_artists()
                self.tr_badge.set_visible(False)

        else:
            self.line_poly.set_data([], [])
            self.poly_patch.set_visible(False)
            self._hide_stability_artists()
            self.tr_badge.set_visible(False)

        # Metrics text box
        n_label = f"{n_grounded} (active)" if n_grounded < 4 else "4 (transition)"
        status  = 'STABLE' if frame['sm'] > 0 else 'TRANSITION'
        self.text_metrics.set_text(
            f"Time Stage    : {frame['timestamp']:.2f} s\n"
            f"Active Mode   : {frame['mode']}\n"
            f"Grounded Legs : {n_label}\n"
            f"Support Area  : {frame['area']:.4f} m²\n"
            f"Stability Marg: {frame['sm']:.4f} m\n"
            f"Tipover Risk  : {frame['tr']:.2f}\n"
            f"Status        : {status}"
        )
        return self._all_artists()

    def _hide_stability_artists(self):
        self.critical_edge_line.set_data([], [])
        self.critical_edge_line.set_visible(False)
        self.sm_arrow.set_positions((-10, -10), (-10, -10))
        self.sm_arrow.set_visible(False)
        self.sm_label.set_visible(False)
        self.center_circle.set_visible(False)
        self.center_marker.set_offsets(np.empty((0, 2)))

    def _all_artists(self):
        return (
            self.line_poly, self.critical_edge_line,
            self.poly_patch, self.center_circle, self.sm_arrow,
            self.sm_label, self.center_marker,
            self.stance_feet_marker, self.swing_foot_marker,
            self.com_marker,
            self.tr_badge, self.mode_badge, self.text_metrics,
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

        # Patches
        self.poly_patch = Polygon(np.empty((0, 2)), facecolor='#4CAF50',
                                  edgecolor='none', alpha=0.15)
        self.ax.add_patch(self.poly_patch)

        self.center_circle = Circle((0, 0), 0, fill=False, ls='--',
                                    color='#546E7A', lw=1.0, alpha=0.75, zorder=2,
                                    label='Inradius Circle ($r_{in}$)')
        self.ax.add_patch(self.center_circle)

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
        self.center_marker      = self.ax.scatter(
            [], [], color='#546E7A', s=60, marker='+', linewidths=1.8,
            zorder=5, label='Polygon Centre')
        self.stance_feet_marker = self.ax.scatter(
            [], [], color='#388e3c', s=100, marker='o', edgecolors='black',
            zorder=6, label='Stance Feet')
        self.swing_foot_marker  = self.ax.scatter(
            [], [], color='#f57c00', s=100, marker='X', edgecolors='black',
            zorder=6, label='Swing Foot')
        self.com_marker         = self.ax.scatter(
            [], [], color='#d32f2f', s=160, marker='P', edgecolors='black',
            zorder=7, label='CoM (base_link)')

        # Text elements
        self.sm_label = self.ax.text(
            0, 0, '', fontsize=8, color='#D32F2F',
            ha='center', fontweight='bold', zorder=6)

        # TR badge — top-left
        self.tr_badge = self.ax.text(
            0.03, 0.97, '', transform=self.ax.transAxes,
            fontsize=10, fontweight='bold', va='top', zorder=8,
            bbox=dict(boxstyle='round,pad=0.3', fc='#4CAF50', alpha=0.85))

        # Mode badge — below TR badge, left side (avoids legend at upper-right)
        self.mode_badge = self.ax.text(
            0.03, 0.86, '', transform=self.ax.transAxes,
            fontsize=10, fontweight='bold', va='top', ha='left', zorder=8,
            bbox=dict(boxstyle='round,pad=0.3', fc='#1565C0', alpha=0.85),
            color='white')

        self.text_metrics = self.ax.text(
            -0.38, -0.38, '',
            fontsize=9, fontfamily='monospace',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#f5f5f5',
                      edgecolor='gray', alpha=0.9))

        self.ax.legend(loc='upper right', frameon=True, facecolor='white',
                       edgecolor='none')
        plt.title("PETER Hybrid Locomotion — Mode Transition Stability",
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
    default_csv      = os.path.join(script_dir, 'cvs',
                                    'stability_session_report_mode_changes.csv')

    # Command-line override
    csv_path = sys.argv[1] if len(sys.argv) > 1 else default_csv

    if not os.path.exists(csv_path):
        # Fallback: latest CSV in cvs/
        cvs_dir = os.path.join(script_dir, 'cvs')
        if os.path.exists(cvs_dir):
            files = [os.path.join(cvs_dir, f)
                     for f in os.listdir(cvs_dir) if f.endswith('.csv')]
            if files:
                csv_path = max(files, key=os.path.getctime)

    animator = ModeTransitionAnimator(csv_path)
    animator.run()

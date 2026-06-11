#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gait Cycle Snapshot Phases — Publication Figure.
Renders 6 specific timestamps as a 2×3 multi-panel figure using the
McGhee-Frank (1968) stability visualization from generate_sim_polygon.py.
"""

import os
import sys
import math
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import seaborn as sns
from matplotlib.patches import Circle
from string import ascii_uppercase

TARGET_TIMESTAMPS = [120.46, 121.06, 121.40, 121.91, 122.42, 122.75]
LEG_NAMES = {0: 'RU', 1: 'LU', 2: 'RD', 3: 'LD'}

# ── 1. PUBLICATION STYLE ──────────────────────────────────────────────────────
def set_publication_style():
    plt.rcParams.update({
        'font.family': 'serif',
        'font.size': 9,
        'axes.labelsize': 9,
        'axes.titlesize': 10,
        'xtick.labelsize': 8,
        'ytick.labelsize': 8,
        'legend.fontsize': 9.5,
        'figure.dpi': 300,
        'savefig.dpi': 300,
        'savefig.bbox': 'tight',
        'axes.grid': True,
        'grid.alpha': 0.3,
        'grid.linestyle': '--',
        'axes.spines.top': False,
        'axes.spines.right': False,
    })
    sns.set_palette("colorblind")


# ── 2. GEOMETRY (identical to generate_sim_polygon.py) ───────────────────────
def _sort_ccw(pts):
    pts = np.asarray(pts, dtype=np.float64)
    if len(pts) < 3:
        return pts
    cx, cy = np.mean(pts, axis=0)
    angles = np.arctan2(pts[:, 1] - cy, pts[:, 0] - cx)
    return pts[np.argsort(angles)]


def _tr_color(tr):
    if tr < 0.8:
        return '#4CAF50'
    if tr < 1.0:
        return '#FFC107'
    return '#F44336'


def _foot_stability(grounded_pts, com=(0.0, 0.0)):
    """McGhee-Frank SM geometry — valid only for exactly 3 grounded feet."""
    if len(grounded_pts) != 3:
        return None
    a, b, c = grounded_pts[0], grounded_pts[1], grounded_pts[2]
    if (b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1]) <= 0:
        b, c = c, b
    sides = [
        math.hypot(b[0]-a[0], b[1]-a[1]),
        math.hypot(c[0]-b[0], c[1]-b[1]),
        math.hypot(a[0]-c[0], a[1]-c[1]),
    ]
    perim = sum(sides)
    area  = abs((a[0]*(b[1]-c[1]) + b[0]*(c[1]-a[1]) + c[0]*(a[1]-b[1])) / 2.0)
    r_in  = (2.0 * area / perim) if perim > 1e-9 else 1e-9
    incenter = (
        (sides[1]*a[0] + sides[2]*b[0] + sides[0]*c[0]) / perim,
        (sides[1]*a[1] + sides[2]*b[1] + sides[0]*c[1]) / perim,
    )
    edges = [(a, b), (b, c), (c, a)]
    dists = []
    for pi, pj in edges:
        num = (pj[0]-pi[0])*(com[1]-pi[1]) - (pj[1]-pi[1])*(com[0]-pi[0])
        den = math.hypot(pj[0]-pi[0], pj[1]-pi[1])
        dists.append(num / den if den > 1e-9 else 0.0)
    nearest = int(np.argmin(dists))
    pi, pj  = edges[nearest]
    ex, ey  = pj[0]-pi[0], pj[1]-pi[1]
    t = ((com[0]-pi[0])*ex + (com[1]-pi[1])*ey) / (ex**2 + ey**2 + 1e-12)
    sm_foot = (pi[0] + t*ex, pi[1] + t*ey)
    return {
        'sm': dists[nearest], 'r_in': r_in, 'incenter': incenter,
        'nearest_edge': edges[nearest], 'sm_foot': sm_foot,
    }


# ── 3. SINGLE PANEL RENDERER ──────────────────────────────────────────────────
def _draw_panel(ax, row, panel_label):
    com = (0.0, 0.0)

    feet_all = np.array([
        [row['foot_RU_x'], row['foot_RU_y']],
        [row['foot_LU_x'], row['foot_LU_y']],
        [row['foot_RD_x'], row['foot_RD_y']],
        [row['foot_LD_x'], row['foot_LD_y']],
    ], dtype=np.float64)

    mode       = str(row.get('mode', 'C')).upper()
    leg_in_air = int(row['leg_in_air'])
    tr         = float(row['TR'])
    timestamp  = float(row['timestamp'])

    on_ground = [True] * 4
    if mode == 'C' and 0 <= leg_in_air < 4:
        on_ground[leg_in_air] = False

    grounded_list = [feet_all[i] for i in range(4) if     on_ground[i]]
    swing_list    = [feet_all[i] for i in range(4) if not on_ground[i]]
    n_grounded    = len(grounded_list)

    grounded_pts = (_sort_ccw(np.array(grounded_list))
                    if n_grounded >= 3 else np.array(grounded_list))

    # Reference cross at origin
    ax.axhline(0, color='k', lw=0.4, ls='--', alpha=0.3)
    ax.axvline(0, color='k', lw=0.4, ls='--', alpha=0.3)

    # ── Support polygon ───────────────────────────────────────────────────────
    if n_grounded >= 3:
        closed = np.vstack([grounded_pts, grounded_pts[0]])
        ax.fill(grounded_pts[:, 0], grounded_pts[:, 1],
                color=_tr_color(tr), alpha=0.15, zorder=1)
        ax.plot(closed[:, 0], closed[:, 1],
                color='#1565C0', lw=1.8, zorder=3)

    # ── Stability decorations (only valid for exactly 3 grounded feet) ────────
    if n_grounded == 3:
        stab = _foot_stability(list(grounded_pts), com)

        if stab is not None and stab['sm'] > 0:
            # Critical (nearest) edge in red
            p1, p2 = stab['nearest_edge']
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]],
                    color='#D32F2F', lw=2.5, zorder=4)

            # SM arrow: CoM → foot of perpendicular on critical edge
            foot = stab['sm_foot']
            ax.annotate(
                '', xy=foot, xytext=com,
                arrowprops=dict(arrowstyle='->', color='#D32F2F',
                                lw=1.5, mutation_scale=14),
                zorder=5,
            )

            # SM label at arrow midpoint
            mx = (com[0] + foot[0]) / 2.0 + 0.014
            my = (com[1] + foot[1]) / 2.0 + 0.014
            ax.text(mx, my, f'SM = {stab["sm"]*100:.1f} cm',
                    fontsize=7.5, color='#D32F2F',
                    ha='center', fontweight='bold', zorder=6)

            # Inscribed circle at incenter
            ax.add_patch(Circle(stab['incenter'], stab['r_in'],
                                fill=False, ls='--', color='#546E7A',
                                lw=1.0, alpha=0.8, zorder=2))

            # Incenter marker
            ax.plot(*stab['incenter'], '+', color='#546E7A',
                    ms=8, mew=1.6, zorder=5)

        # TR badge — top-right corner
        ax.text(0.97, 0.97, f'TR = {tr:.3f}',
                transform=ax.transAxes,
                fontsize=8.5, fontweight='bold', va='top', ha='right',
                bbox=dict(boxstyle='round,pad=0.3',
                          fc=_tr_color(tr), alpha=0.85))

    else:
        # 4 feet on ground: static stance
        ax.text(0.97, 0.97, 'Static stance',
                transform=ax.transAxes, fontsize=8, va='top', ha='right',
                bbox=dict(boxstyle='round,pad=0.3', fc='#90CAF9', alpha=0.85))

    # ── Feet ──────────────────────────────────────────────────────────────────
    for i, (fx, fy) in enumerate(feet_all):
        if on_ground[i]:
            ax.scatter(fx, fy, color='#388e3c', s=95, marker='o',
                       edgecolors='black', linewidths=0.8, zorder=6)
        else:
            ax.scatter(fx, fy, color='#f57c00', s=95, marker='X',
                       edgecolors='black', linewidths=0.8, zorder=6)

    # ── CoM fixed at body-frame origin ────────────────────────────────────────
    ax.scatter(0, 0, color='#d32f2f', s=160, marker='P',
               edgecolors='black', linewidths=0.8, zorder=7)

    # ── Panel label (A, B, C …) ───────────────────────────────────────────────
    ax.text(-0.10, 1.06, panel_label, transform=ax.transAxes,
            fontsize=12, fontweight='bold', va='top')

    # ── Subplot title ─────────────────────────────────────────────────────────
    if n_grounded == 3:
        swing_name = LEG_NAMES.get(leg_in_air, '?')
        phase_str  = f'{swing_name} Swing Phase'
    else:
        phase_str = '4-Leg Stance'
    ax.set_title(f't = {timestamp:.2f} s  |  {phase_str}', fontsize=9.5, pad=6)

    # ── Axis formatting ───────────────────────────────────────────────────────
    ax.set_xlim(-0.3, 0.3)
    ax.set_ylim(-0.35, 0.35)
    ax.set_aspect('equal')
    ax.set_xlabel('Body X [m]', fontsize=8.5)
    ax.set_ylabel('Body Y [m]', fontsize=8.5)


# ── 4. FIGURE ASSEMBLY ────────────────────────────────────────────────────────
def generate_snapshot_figure(csv_path, timestamps=None, output_stem='fig_stability_phases'):
    targets = timestamps if timestamps is not None else TARGET_TIMESTAMPS

    df = pd.read_csv(csv_path)
    print(f'Loaded {len(df)} rows — '
          f'timestamp range: {df["timestamp"].min():.2f} – {df["timestamp"].max():.2f} s')
    print(f'Target timestamps: {targets}')

    rows = []
    for t in targets:
        idx      = (df['timestamp'] - t).abs().idxmin()
        actual_t = df.loc[idx, 'timestamp']
        print(f'  Target {t:.2f} s → matched {actual_t:.3f} s '
              f'(leg_in_air={int(df.loc[idx,"leg_in_air"])}, '
              f'SM={df.loc[idx,"SM"]:.4f}, TR={df.loc[idx,"TR"]:.3f})')
        rows.append(df.loc[idx])

    set_publication_style()

    n = len(rows)
    ncols = min(n, 3)
    nrows = math.ceil(n / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(6 * ncols, 6.5 * nrows))
    axes = np.array(axes).flatten()

    for i, (ax, row) in enumerate(zip(axes, rows)):
        _draw_panel(ax, row, ascii_uppercase[i])

    for ax in axes[n:]:
        ax.set_visible(False)

    # ── Shared legend (proxy artists) ─────────────────────────────────────────
    legend_handles = [
        plt.Line2D([0], [0], color='#1565C0', lw=1.8,
                   label='Support Polygon'),
        plt.Line2D([0], [0], color='#D32F2F', lw=2.5,
                   label='Critical Edge'),
        plt.Line2D([0], [0], color='#546E7A', ls='--', lw=1.0,
                   label='Inscribed Circle ($r_{in}$)'),
        plt.Line2D([0], [0], color='#D32F2F', lw=1.5, marker='>',
                   markersize=7, label='SM Arrow (CoM → Critical Edge)'),
        plt.Line2D([0], [0], color='#546E7A', marker='+', ms=8,
                   lw=0, mew=1.6, label='Incenter'),
        plt.Line2D([0], [0], color='#388e3c', marker='o', ms=8,
                   lw=0, markeredgecolor='black', label='Stance Feet'),
        plt.Line2D([0], [0], color='#f57c00', marker='X', ms=8,
                   lw=0, markeredgecolor='black', label='Swing Foot'),
        plt.Line2D([0], [0], color='#d32f2f', marker='P', ms=9,
                   lw=0, markeredgecolor='black', label='CoM (base_link)'),
    ]

    fig.legend(handles=legend_handles, loc='lower center', ncol=4,
               bbox_to_anchor=(0.5, -0.03), frameon=False, fontsize=9.5,
               handlelength=2.0, columnspacing=1.5)

    fig.suptitle(
        'PETER Quadruped — Static Stability Analysis\n'
        'McGhee-Frank (1968) Stability Margin',
        fontsize=13, fontweight='bold', y=1.01,
    )

    plt.tight_layout(rect=[0, 0.06, 1, 1])

    for ext in ('pdf', 'png'):
        out_path = f'{output_stem}.{ext}'
        plt.savefig(out_path, bbox_inches='tight')
        print(f'[OK] Saved → {out_path}')

    plt.show()


# ── 5. ENTRY POINT ────────────────────────────────────────────────────────────
if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Default CSV: latest file in cvs/
    cvs_dir      = os.path.join(script_dir, 'cvs')
    default_files = ([os.path.join(cvs_dir, f)
                      for f in os.listdir(cvs_dir) if f.endswith('.csv')]
                     if os.path.exists(cvs_dir) else [])
    default_csv  = (max(default_files, key=os.path.getctime)
                    if default_files else '')

    parser = argparse.ArgumentParser(
        description='Generate a multi-panel stability snapshot figure.',
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=(
            'Examples:\n'
            '  # Use defaults (latest CSV, hardcoded timestamps)\n'
            '  python3 generate_snapshot_phases.py\n\n'
            '  # Custom CSV only\n'
            '  python3 generate_snapshot_phases.py --csv cvs/stability_session_report_mode_changes.csv\n\n'
            '  # Custom CSV + timestamps\n'
            '  python3 generate_snapshot_phases.py \\\n'
            '      --csv cvs/stability_session_report_mode_changes.csv \\\n'
            '      --t 51.83 63.58 70.03 75.42 79.66 95.07\n\n'
            '  # Custom output name\n'
            '  python3 generate_snapshot_phases.py --t 51.83 63.58 --output fig_modes'
        ),
    )
    parser.add_argument(
        '--csv', default=default_csv, metavar='PATH',
        help=f'Path to stability CSV  (default: {default_csv or "none found"})',
    )
    parser.add_argument(
        '--t', nargs='+', type=float, default=None, metavar='T',
        help='Target timestamps in seconds, space-separated\n'
             '(default: hardcoded TARGET_TIMESTAMPS)',
    )
    parser.add_argument(
        '--output', default=os.path.join(script_dir, 'fig_stability_phases'),
        metavar='STEM',
        help='Output filename stem without extension  (default: fig_stability_phases)',
    )

    args = parser.parse_args()

    if not args.csv or not os.path.exists(args.csv):
        parser.error(f'CSV not found: "{args.csv}"')

    generate_snapshot_figure(args.csv, timestamps=args.t, output_stem=args.output)

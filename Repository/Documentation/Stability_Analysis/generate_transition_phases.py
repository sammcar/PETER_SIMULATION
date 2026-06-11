#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mode Transition Stability Figure — Publication Quality.

For each mode transition detected in the CSV, renders 3 frames:
  - Frame 1: first row where the new mode appears (transition onset, t₀)
  - Frame 2: closest row to t₀ + 1 s
  - Frame 3: closest row to t₀ + 2 s

Layout: GridSpec with a label column (showing FROM → TO) + 3 panel columns.
Uses the generalised McGhee-Frank SM for both 3-foot and 4-foot polygons.

Usage:
  python3 generate_transition_phases.py
  python3 generate_transition_phases.py --csv path/to/file.csv
  python3 generate_transition_phases.py --csv file.csv --output fig_transitions
"""

import os
import sys
import math
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
import seaborn as sns
from matplotlib.patches import Circle
from string import ascii_uppercase

# ── Mode colour palette ───────────────────────────────────────────────────────
MODE_COLORS = {
    'C': '#1565C0',   # blue   – crawl
    'H': '#E65100',   # orange – hybrid
    'X': '#6A1B9A',   # purple – cross/trot
}
MODE_LABELS = {
    'C': 'Quadrupedal\nWalking (C)',
    'H': 'Differential\nRolling (H)',
    'X': 'Omnidirectional\nMotion (X)',
}
MODE_FULL_NAMES = {
    'C': 'C — Quadrupedal Walking',
    'H': 'H — Differential Rolling',
    'X': 'X — Omnidirectional Motion',
}

LEG_NAMES = {0: 'RU', 1: 'LU', 2: 'RD', 3: 'LD'}

COL_TITLES = ['Onset  ($t_0$)', '$t_0 + 1\\,\\mathrm{s}$', '$t_0 + 2\\,\\mathrm{s}$']

# ── 1. PUBLICATION STYLE ──────────────────────────────────────────────────────
def set_publication_style():
    plt.rcParams.update({
        'font.family': 'serif',
        'font.size': 9,
        'axes.labelsize': 8.5,
        'axes.titlesize': 9,
        'xtick.labelsize': 7.5,
        'ytick.labelsize': 7.5,
        'legend.fontsize': 9,
        'figure.dpi': 120,
        'savefig.dpi': 300,
        'savefig.bbox': 'tight',
        'axes.grid': True,
        'grid.alpha': 0.3,
        'grid.linestyle': '--',
        'axes.spines.top': False,
        'axes.spines.right': False,
    })
    sns.set_palette('colorblind')


# ── 2. GEOMETRY (generalised for 3 or 4 feet) ────────────────────────────────
def _sort_ccw(pts):
    pts = np.asarray(pts, dtype=np.float64)
    if len(pts) < 3:
        return pts
    cx, cy = np.mean(pts, axis=0)
    return pts[np.argsort(np.arctan2(pts[:, 1] - cy, pts[:, 0] - cx))]


def _tr_color(tr):
    if tr < 0.8:
        return '#4CAF50'
    if tr < 1.0:
        return '#FFC107'
    return '#F44336'


def _foot_stability(grounded_pts, com=(0.0, 0.0)):
    """
    Generalised McGhee-Frank SM for N ≥ 3 grounded feet.
      3 feet → exact triangle incenter + inradius.
      4 feet → polygon centroid; r_in = min dist centroid→edge (circle stays inside).
    """
    n = len(grounded_pts)
    if n < 3:
        return None

    pts = _sort_ccw(np.asarray(grounded_pts, dtype=np.float64))

    # Shoelace area
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

    edges = [(pts[i], pts[(i + 1) % n]) for i in range(n)]

    if n == 3:
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
        cx, cy = float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1]))
        center = (cx, cy)
        # True inradius at centroid = min perpendicular distance to any edge
        r_in = min(
            abs((pj[0]-pi[0])*(cy-pi[1]) - (pj[1]-pi[1])*(cx-pi[0]))
            / math.hypot(pj[0]-pi[0], pj[1]-pi[1])
            for pi, pj in edges
        )

    # Signed distances from CoM to each directed edge (positive = inside)
    dists = []
    for pi, pj in edges:
        num = (pj[0]-pi[0])*(com[1]-pi[1]) - (pj[1]-pi[1])*(com[0]-pi[0])
        den = math.hypot(pj[0]-pi[0], pj[1]-pi[1])
        dists.append(num / den if den > 1e-9 else 0.0)

    nearest = int(np.argmin(dists))
    sm      = dists[nearest]
    sm_norm = sm / r_in if r_in > 1e-9 else 0.0
    tr      = 1.0 - sm_norm

    pi, pj  = edges[nearest]
    ex, ey  = pj[0]-pi[0], pj[1]-pi[1]
    t_      = ((com[0]-pi[0])*ex + (com[1]-pi[1])*ey) / (ex**2 + ey**2 + 1e-12)
    sm_foot = (pi[0] + t_*ex, pi[1] + t_*ey)

    return {
        'sm': sm, 'sm_norm': sm_norm, 'tr': tr,
        'r_in': r_in, 'center': center, 'area': area,
        'nearest_edge': edges[nearest], 'sm_foot': sm_foot,
    }


# ── 3. TRANSITION DETECTION ───────────────────────────────────────────────────
def find_transitions(df):
    """
    Returns a list of dicts, one per mode change:
      from_mode, to_mode, t0, frames (3 × pd.Series)
    """
    transitions = []
    prev_mode = str(df.iloc[0]['mode']).upper()

    for i in range(1, len(df)):
        curr_mode = str(df.iloc[i]['mode']).upper()
        if curr_mode != prev_mode:
            t0   = float(df.iloc[i]['timestamp'])
            idx0 = df.index[i]
            idx1 = (df['timestamp'] - (t0 + 1.0)).abs().idxmin()
            idx2 = (df['timestamp'] - (t0 + 2.0)).abs().idxmin()
            transitions.append({
                'from_mode': prev_mode,
                'to_mode':   curr_mode,
                't0':        t0,
                'frames':    [df.loc[idx0], df.loc[idx1], df.loc[idx2]],
            })
            prev_mode = curr_mode

    return transitions


# ── 4. SINGLE PANEL RENDERER ──────────────────────────────────────────────────
def _draw_panel(ax, row, panel_label, col_idx, from_mode, to_mode):
    com = (0.0, 0.0)

    feet_all = np.array([
        [row['foot_RU_x'], row['foot_RU_y']],
        [row['foot_LU_x'], row['foot_LU_y']],
        [row['foot_RD_x'], row['foot_RD_y']],
        [row['foot_LD_x'], row['foot_LD_y']],
    ], dtype=np.float64)

    mode       = str(row.get('mode', 'C')).upper()
    leg_in_air = int(row['leg_in_air'])
    timestamp  = float(row['timestamp'])

    on_ground = [True] * 4
    if mode == 'C' and 0 <= leg_in_air < 4:
        on_ground[leg_in_air] = False

    grounded_list = [feet_all[i] for i in range(4) if on_ground[i]]
    swing_list    = [feet_all[i] for i in range(4) if not on_ground[i]]
    n_grounded    = len(grounded_list)

    grounded_pts = (_sort_ccw(np.array(grounded_list))
                    if n_grounded >= 3 else np.array(grounded_list))

    # Reference cross at origin
    ax.axhline(0, color='k', lw=0.4, ls='--', alpha=0.3)
    ax.axvline(0, color='k', lw=0.4, ls='--', alpha=0.3)

    # Compute stability
    stab = _foot_stability(list(grounded_pts), com) if n_grounded >= 3 else None
    tr   = stab['tr'] if stab else 1.0

    # Support polygon — colour-coded fill
    if n_grounded >= 3:
        closed = np.vstack([grounded_pts, grounded_pts[0]])
        ax.fill(grounded_pts[:, 0], grounded_pts[:, 1],
                color=_tr_color(tr), alpha=0.15, zorder=1)
        ax.plot(closed[:, 0], closed[:, 1],
                color='#1565C0', lw=1.6, zorder=3)

    # Stability decorations
    if stab is not None and stab['sm'] > 0:
        # Critical edge
        p1, p2 = stab['nearest_edge']
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]],
                color='#D32F2F', lw=2.2, zorder=4)

        # SM arrow: CoM → foot of perpendicular
        foot = stab['sm_foot']
        ax.annotate('', xy=foot, xytext=com,
                    arrowprops=dict(arrowstyle='->', color='#D32F2F',
                                    lw=1.4, mutation_scale=12),
                    zorder=5)

        # SM label
        mx = (com[0] + foot[0]) / 2.0 + 0.013
        my = (com[1] + foot[1]) / 2.0 + 0.013
        ax.text(mx, my, f'SM={stab["sm"]*100:.1f} cm',
                fontsize=7, color='#D32F2F',
                ha='center', fontweight='bold', zorder=6)

        # Inradius circle at polygon centre
        ax.add_patch(Circle(stab['center'], stab['r_in'],
                            fill=False, ls='--', color='#546E7A',
                            lw=0.9, alpha=0.8, zorder=2))
        ax.plot(*stab['center'], '+', color='#546E7A',
                ms=7, mew=1.4, zorder=5)

        # TR badge — top-right
        ax.text(0.97, 0.97, f'TR={tr:.3f}',
                transform=ax.transAxes,
                fontsize=7.5, fontweight='bold', va='top', ha='right',
                bbox=dict(boxstyle='round,pad=0.25',
                          fc=_tr_color(tr), alpha=0.85))

    # Mode badge — top-left (below panel label)
    ax.text(0.03, 0.86, mode,
            transform=ax.transAxes,
            fontsize=8, fontweight='bold', va='top', ha='left',
            color='white',
            bbox=dict(boxstyle='round,pad=0.25',
                      fc=MODE_COLORS.get(mode, '#546E7A'), alpha=0.9))

    # Feet
    for i, (fx, fy) in enumerate(feet_all):
        if on_ground[i]:
            ax.scatter(fx, fy, color='#388e3c', s=80, marker='o',
                       edgecolors='black', linewidths=0.7, zorder=6)
        else:
            ax.scatter(fx, fy, color='#f57c00', s=80, marker='X',
                       edgecolors='black', linewidths=0.7, zorder=6)

    # CoM
    ax.scatter(0, 0, color='#d32f2f', s=130, marker='P',
               edgecolors='black', linewidths=0.7, zorder=7)

    # Panel label
    ax.text(-0.10, 1.06, panel_label, transform=ax.transAxes,
            fontsize=10, fontweight='bold', va='top')

    # Title: timestamp + timing offset label
    timing = COL_TITLES[col_idx]
    ax.set_title(f'{timing}   (t = {timestamp:.2f} s)', fontsize=8.5, pad=5)

    ax.set_xlim(-0.4, 0.4)
    ax.set_ylim(-0.45, 0.45)
    ax.set_aspect('equal')
    ax.set_xlabel('Body X [m]', fontsize=8)
    ax.set_ylabel('Body Y [m]', fontsize=8)


# ── 5. TRANSITION LABEL COLUMN ────────────────────────────────────────────────
def _draw_transition_label(ax, from_mode, to_mode):
    """Draws the FROM → TO mode label in a text-only axis."""
    ax.axis('off')
    fc = MODE_COLORS.get(from_mode, '#546E7A')
    tc = MODE_COLORS.get(to_mode,   '#546E7A')

    # FROM badge
    ax.text(0.5, 0.68, MODE_LABELS.get(from_mode, from_mode),
            transform=ax.transAxes, ha='center', va='center',
            fontsize=8.5, fontweight='bold', color='white',
            bbox=dict(boxstyle='round,pad=0.4', fc=fc, alpha=0.92))

    # Arrow
    ax.annotate('', xy=(0.5, 0.43), xytext=(0.5, 0.57),
                xycoords='axes fraction', textcoords='axes fraction',
                arrowprops=dict(arrowstyle='->', color='#333333', lw=1.8))

    # TO badge
    ax.text(0.5, 0.32, MODE_LABELS.get(to_mode, to_mode),
            transform=ax.transAxes, ha='center', va='center',
            fontsize=8.5, fontweight='bold', color='white',
            bbox=dict(boxstyle='round,pad=0.4', fc=tc, alpha=0.92))


# ── 6. SINGLE-GROUP RENDERER ─────────────────────────────────────────────────
def _render_transition_group(transitions, output_stem, part_label='', start_row=0):
    """Renders a subset of transitions into one publication-quality figure.

    Parameters
    ----------
    transitions : list of dicts   (output of find_transitions)
    output_stem : str             base path without extension
    part_label  : str             optional suffix for suptitle, e.g. 'Part 1 of 2'
    start_row   : int             offset for panel-letter index (A=0, D=3, …)
    """
    n_trans = len(transitions)
    if n_trans == 0:
        return

    fig_w = 17.0
    fig_h = 5.8 * n_trans

    fig = plt.figure(figsize=(fig_w, fig_h))
    gs  = gridspec.GridSpec(
        n_trans, 4,
        figure=fig,
        width_ratios=[0.13, 1, 1, 1],
        hspace=0.55,
        wspace=0.28,
    )

    for row_i, trans in enumerate(transitions):
        ax_lbl = fig.add_subplot(gs[row_i, 0])
        _draw_transition_label(ax_lbl, trans['from_mode'], trans['to_mode'])

        for col_j, frame_row in enumerate(trans['frames']):
            ax    = fig.add_subplot(gs[row_i, col_j + 1])
            label = f'{ascii_uppercase[start_row + row_i]}{col_j + 1}'
            _draw_panel(ax, frame_row, label, col_j,
                        trans['from_mode'], trans['to_mode'])

    # ── Shared legend ─────────────────────────────────────────────────────────
    legend_handles = [
        plt.Line2D([0], [0], color='#1565C0', lw=1.6,
                   label='Support Polygon'),
        plt.Line2D([0], [0], color='#D32F2F', lw=2.2,
                   label='Critical Edge'),
        plt.Line2D([0], [0], color='#546E7A', ls='--', lw=0.9,
                   label='Inradius Circle ($r_{in}$)'),
        plt.Line2D([0], [0], color='#D32F2F', lw=1.4, marker='>',
                   markersize=6, label='SM Arrow (CoM $\\rightarrow$ Critical Edge)'),
        plt.Line2D([0], [0], color='#546E7A', marker='+', ms=7,
                   lw=0, mew=1.4, label='Polygon Centre'),
        plt.Line2D([0], [0], color='#388e3c', marker='o', ms=7,
                   lw=0, markeredgecolor='black', label='Stance Feet'),
        plt.Line2D([0], [0], color='#f57c00', marker='X', ms=7,
                   lw=0, markeredgecolor='black', label='Swing Foot'),
        plt.Line2D([0], [0], color='#d32f2f', marker='P', ms=8,
                   lw=0, markeredgecolor='black', label=r'CoM (base\_link)'),
        mpatches.Patch(fc=MODE_COLORS['C'], label=MODE_FULL_NAMES['C']),
        mpatches.Patch(fc=MODE_COLORS['H'], label=MODE_FULL_NAMES['H']),
        mpatches.Patch(fc=MODE_COLORS['X'], label=MODE_FULL_NAMES['X']),
    ]

    fig.legend(handles=legend_handles, loc='lower center', ncol=4,
               bbox_to_anchor=(0.5, 0.01), frameon=False,
               fontsize=9, handlelength=2.0, columnspacing=1.4)

    title = ('Static Stability Analysis across Locomotion Mode Transitions\n'
             'Generalised McGhee-Frank (1968) Support Polygon')
    if part_label:
        title += f'  —  {part_label}'

    fig.suptitle(title, fontsize=13, fontweight='bold', y=0.99)

    fig.subplots_adjust(left=0.05, right=0.97, top=0.91, bottom=0.10,
                        hspace=0.55, wspace=0.28)

    for ext in ('pdf', 'png'):
        out = f'{output_stem}.{ext}'
        plt.savefig(out, bbox_inches='tight')
        print(f'[OK] → {out}')

    plt.show()


# ── 7. FIGURE ASSEMBLY ────────────────────────────────────────────────────────
def generate_transition_figure(csv_path, output_stem='fig_transition_phases'):
    df = pd.read_csv(csv_path)
    print(f'Loaded {len(df)} rows — '
          f'range: {df["timestamp"].min():.2f} – {df["timestamp"].max():.2f} s')

    transitions = find_transitions(df)
    if not transitions:
        print('No mode transitions found in this CSV.')
        return

    print(f'\nDetected {len(transitions)} transitions:')
    for tr in transitions:
        ts = [float(f['timestamp']) for f in tr['frames']]
        print(f'  {tr["from_mode"]} → {tr["to_mode"]}  '
              f't₀={ts[0]:.3f}s  t₁={ts[1]:.3f}s  t₂={ts[2]:.3f}s')

    set_publication_style()

    chunk   = 3   # transitions per figure page
    parts   = [transitions[i:i + chunk] for i in range(0, len(transitions), chunk)]
    n_parts = len(parts)

    for p_idx, part in enumerate(parts):
        if n_parts > 1:
            part_label = f'Part {p_idx + 1} of {n_parts}'
            stem       = f'{output_stem}_part{p_idx + 1}'
        else:
            part_label = ''
            stem       = output_stem

        print(f'\n── Rendering {part_label or "figure"} → {stem} ──')
        _render_transition_group(
            part, stem,
            part_label = part_label,
            start_row  = p_idx * chunk,
        )


# ── 8. ENTRY POINT ────────────────────────────────────────────────────────────
if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_csv = os.path.join(script_dir, 'cvs',
                               'stability_session_report_mode_changes.csv')

    parser = argparse.ArgumentParser(
        description='Publication figure of mode-transition stability phases.',
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=(
            'Examples:\n'
            '  python3 generate_transition_phases.py\n'
            '  python3 generate_transition_phases.py --csv cvs/stability_session_report_mode_changes.csv\n'
            '  python3 generate_transition_phases.py --csv file.csv --output fig_transitions\n'
        ),
    )
    parser.add_argument(
        '--csv', default=default_csv, metavar='PATH',
        help=f'Path to stability CSV  (default: stability_session_report_mode_changes.csv)',
    )
    parser.add_argument(
        '--output',
        default=os.path.join(script_dir, 'fig_transition_phases'),
        metavar='STEM',
        help='Output filename stem without extension  (default: fig_transition_phases)',
    )

    args = parser.parse_args()

    if not os.path.exists(args.csv):
        parser.error(f'CSV not found: "{args.csv}"')

    generate_transition_figure(args.csv, output_stem=args.output)

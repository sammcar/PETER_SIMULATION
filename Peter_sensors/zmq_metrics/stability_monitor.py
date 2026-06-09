#!/usr/bin/env python3
"""
stability_monitor.py — Monitor de estabilidad estática del cuadrúpedo.

Calcula SM, SM_norm y TR (McGhee & Frank 1968) a partir de la posición de
los pies y el centro de masa, publicados por el controlador en /peter/stability_data.

Guarda stability_log.csv y muestra 4 gráficas en tiempo real.
"""

import csv
import math
import threading
import time
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle

from transport import Node, spin, PoseArray
from topics import TOPIC_STABILITY_DATA

LOG_FILE  = str(Path.home() / 'stability_log.csv')
WINDOW_S  = 30.0
ALERT_TR  = 0.8
CRIT_TR   = 1.0

LEG_NAMES  = ['RU', 'LU', 'RD', 'LD']
LEG_COLORS = ['#1976D2', '#388E3C', '#F57C00', '#7B1FA2']

CSV_FIELDS = [
    'timestamp', 'leg_in_air',
    'foot_RU_x', 'foot_RU_y', 'foot_LU_x', 'foot_LU_y',
    'foot_RD_x', 'foot_RD_y', 'foot_LD_x', 'foot_LD_y',
    'SM', 'SM_norm', 'TR', 'triangle_area', 'inradius',
]


# ── Stability math (unchanged from original) ──────────────────────────────────

def _signed_dist(pi, pj, c):
    num = (pj[0]-pi[0])*(c[1]-pi[1]) - (pj[1]-pi[1])*(c[0]-pi[0])
    den = math.hypot(pj[0]-pi[0], pj[1]-pi[1])
    return num / den if den > 1e-9 else 0.0


def _ensure_ccw(pts):
    a, b, c = pts
    cross = (b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1])
    return [a, b, c] if cross > 0 else [a, c, b]


def compute_stability(feet3, com):
    a, b, c = _ensure_ccw(feet3)
    side_lens = [
        math.hypot(b[0]-a[0], b[1]-a[1]),
        math.hypot(c[0]-b[0], c[1]-b[1]),
        math.hypot(a[0]-c[0], a[1]-c[1]),
    ]
    perim = sum(side_lens)
    area  = abs((a[0]*(b[1]-c[1]) + b[0]*(c[1]-a[1]) + c[0]*(a[1]-b[1])) / 2.0)
    r_in  = (2.0 * area / perim) if perim > 1e-9 else 1e-9

    dists   = [_signed_dist(a, b, com), _signed_dist(b, c, com), _signed_dist(c, a, com)]
    sm      = min(dists)
    sm_norm = sm / r_in
    tr      = 1.0 - sm_norm

    incenter = (
        (side_lens[1]*a[0] + side_lens[2]*b[0] + side_lens[0]*c[0]) / perim,
        (side_lens[1]*a[1] + side_lens[2]*b[1] + side_lens[0]*c[1]) / perim,
    )
    return {
        'sm': sm, 'sm_norm': sm_norm, 'tr': tr,
        'area': area, 'r_in': r_in,
        'verts': [a, b, c],
        'dists': dists,
        'nearest': int(np.argmin(dists)),
        'incenter': incenter,
    }


# ── ZMQ Node (replaces ROS2 StabilityMonitor) ─────────────────────────────────

class StabilityMonitor(Node):

    def __init__(self):
        super().__init__('peter_stability_monitor')
        self.lock         = threading.Lock()
        self._t0          = None
        self._latest_pose = None
        self._latest_sm   = None

        self._ht    = deque()
        self._hsm   = deque()
        self._hsmn  = deque()
        self._htr   = deque()
        self._harea = deque()
        self._csv_rows = []

        self.create_subscription(PoseArray, TOPIC_STABILITY_DATA, self._cb, 10)
        self.get_logger().info('PETER Stability Monitor started.')

    def _cb(self, msg):
        if len(msg.poses) < 5:
            return

        feet_all  = [(-msg.poses[i].position.y,  msg.poses[i].position.x) for i in range(4)]
        on_ground = [msg.poses[i].orientation.w > 0.5 for i in range(4)]
        com       = (-msg.poses[4].position.y, msg.poses[4].position.x)
        grounded  = [feet_all[i] for i in range(4) if on_ground[i]]
        air_leg   = next((i for i in range(4) if not on_ground[i]), -1)

        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0

        with self.lock:
            self._latest_pose = {
                't': t, 'feet_all': feet_all, 'on_ground': on_ground,
                'com': com, 'n_grounded': len(grounded),
            }
            if len(grounded) != 3:
                return

            res = compute_stability(grounded, com)
            self._latest_sm = res

            self._ht.append(t)
            self._hsm.append(res['sm'])
            self._hsmn.append(res['sm_norm'])
            self._htr.append(res['tr'])
            self._harea.append(res['area'])

            self._csv_rows.append({
                'timestamp': t, 'leg_in_air': air_leg,
                'foot_RU_x': feet_all[0][0], 'foot_RU_y': feet_all[0][1],
                'foot_LU_x': feet_all[1][0], 'foot_LU_y': feet_all[1][1],
                'foot_RD_x': feet_all[2][0], 'foot_RD_y': feet_all[2][1],
                'foot_LD_x': feet_all[3][0], 'foot_LD_y': feet_all[3][1],
                'SM': res['sm'], 'SM_norm': res['sm_norm'], 'TR': res['tr'],
                'triangle_area': res['area'], 'inradius': res['r_in'],
            })

    def save_csv(self):
        with self.lock:
            rows = list(self._csv_rows)
        with open(LOG_FILE, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            w.writeheader()
            w.writerows(rows)
        self.get_logger().info(f'CSV guardado ({len(rows)} muestras) → {LOG_FILE}')

    def print_stats(self):
        with self.lock:
            if not self._htr:
                print('No data recorded.')
                return
            trs  = np.array(self._htr)
            sms  = np.array(self._hsm)
            smns = np.array(self._hsmn)

        alert_pct  = 100.0 * float(np.mean(trs > ALERT_TR))
        n_critical = int(np.sum(trs >= CRIT_TR))

        print('\n====== SESSION STATISTICS ======')
        print(f'  Samples           : {len(trs)}')
        print(f'  TR   mean/min/max : {trs.mean():.3f} / {trs.min():.3f} / {trs.max():.3f}')
        print(f'  SM_norm mean/min  : {smns.mean():.3f} / {smns.min():.3f}')
        print(f'  SM min [m]        : {sms.min():.4f}')
        print(f'  Time in alert zone (TR > 0.8) : {alert_pct:.1f}%')
        print(f'  Critical events   (TR >= 1.0) : {n_critical}')
        print(f'  Log: {LOG_FILE}')
        print('================================\n')


# ── Plot helpers (unchanged from original) ────────────────────────────────────

def _tr_color(tr):
    if tr < ALERT_TR:  return '#4CAF50'
    if tr < CRIT_TR:   return '#FFC107'
    return '#F44336'


def _setup_polygon_ax(ax):
    ax.set_title('Support Polygon and CoM', fontsize=9)
    ax.set_xlabel('← Right    Body-Y    Left →  [m]', fontsize=8)
    ax.set_ylabel('Body-X (forward ↑)  [m]', fontsize=8)
    ax.set_xlim(-0.25, 0.25); ax.set_ylim(-0.30, 0.35)
    ax.set_aspect('equal'); ax.tick_params(labelsize=7)


def _draw_polygon(ax, pose, sm_res):
    ax.cla(); _setup_polygon_ax(ax)
    ax.axhline(0, color='k', lw=0.4, ls='--', alpha=0.35)
    ax.axvline(0, color='k', lw=0.4, ls='--', alpha=0.35)

    feet_all   = pose['feet_all']
    on_ground  = pose['on_ground']
    com        = pose['com']
    n_grounded = pose['n_grounded']

    grounded_pts = [feet_all[i] for i in range(4) if on_ground[i]]

    if n_grounded >= 3:
        pts    = np.array(grounded_pts)
        cx, cy = pts[:, 0].mean(), pts[:, 1].mean()
        order  = np.argsort(np.arctan2(pts[:, 1]-cy, pts[:, 0]-cx))
        hull   = pts[order]
        closed = np.vstack([hull, hull[0]])
        fill_c = _tr_color(sm_res['tr']) if sm_res else '#90CAF9'
        ax.fill(closed[:, 0], closed[:, 1], alpha=0.13, color=fill_c)
        ax.plot(closed[:, 0], closed[:, 1], color='#1565C0', lw=1.2, zorder=3)

    if sm_res is not None and n_grounded == 3:
        verts    = sm_res['verts']
        nearest  = sm_res['nearest']
        edges    = [(verts[i], verts[(i+1) % 3]) for i in range(3)]
        p1, p2   = edges[nearest]
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='#D32F2F', lw=2.2, zorder=4)
        ax.add_patch(Circle(sm_res['incenter'], sm_res['r_in'],
                            fill=False, ls='--', color='#546E7A', lw=0.9, alpha=0.7))
        ax.plot(*sm_res['incenter'], '+', color='#546E7A', ms=6, mew=1.2, zorder=4)
        mx, my = (p1[0]+p2[0])/2.0, (p1[1]+p2[1])/2.0
        ax.annotate('', xy=(mx, my), xytext=com,
                    arrowprops=dict(arrowstyle='->', color='#D32F2F', lw=1.3))
        ax.text((com[0]+mx)/2.0, (com[1]+my)/2.0+0.015,
                f"SM = {sm_res['sm']*100:.1f} cm", fontsize=7, color='#D32F2F', ha='center')
        ax.text(0.03, 0.97, f"TR = {sm_res['tr']:.3f}", transform=ax.transAxes,
                fontsize=10, fontweight='bold', va='top',
                bbox=dict(boxstyle='round,pad=0.3', fc=_tr_color(sm_res['tr']), alpha=0.85))
    elif n_grounded == 4:
        ax.text(0.03, 0.97, 'Static stance\n(4 feet on ground)',
                transform=ax.transAxes, fontsize=8, va='top',
                bbox=dict(boxstyle='round,pad=0.3', fc='#90CAF9', alpha=0.85))

    for i, (fx, fy) in enumerate(feet_all):
        mk = 'o' if on_ground[i] else 'x'
        ax.plot(fx, fy, marker=mk, color=LEG_COLORS[i], ms=9 if on_ground[i] else 8,
                markeredgewidth=1.6, zorder=7)
        ax.text(fx+0.012, fy+0.012, LEG_NAMES[i], fontsize=7.5,
                color=LEG_COLORS[i], fontweight='bold', zorder=8)
    ax.plot(*com, 'k+', ms=13, mew=2.2, zorder=9)
    ax.text(com[0]+0.010, com[1]-0.018, 'CoM', fontsize=7, color='black', zorder=9)


def _setup_ts_ax(ax):
    ax.set_title('SM$_{norm}$ and Tipover Risk vs. Time', fontsize=9)
    ax.set_xlabel('Time [s]', fontsize=8); ax.set_ylabel('Value', fontsize=8)
    ax.set_ylim(-0.25, 1.65); ax.tick_params(labelsize=7)
    ax.axhline(CRIT_TR,  color='#D32F2F', ls='--', lw=1.0, alpha=0.85, label='TR=1.0 critical')
    ax.axhline(ALERT_TR, color='#F9A825', ls='--', lw=1.0, alpha=0.85, label='TR=0.8 alert')
    ax.axhline(0.0,      color='#388E3C', ls='--', lw=0.7, alpha=0.55)
    ln_smn, = ax.plot([], [], color='#1565C0', lw=1.3, label='SM$_{norm}$')
    ln_tr,  = ax.plot([], [], color='#E65100', lw=1.3, label='TR')
    ax.legend(fontsize=7, loc='upper left')
    return ln_smn, ln_tr


def _update_ts(ax, ln_smn, ln_tr, ht, hsmn, htr):
    if not ht: return
    t_arr = np.array(ht); t_now = t_arr[-1]
    mask  = t_arr >= (t_now - WINDOW_S)
    t_win = t_arr[mask]
    ln_smn.set_data(t_win, np.array(hsmn)[mask])
    ln_tr.set_data(t_win,  np.array(htr)[mask])
    ax.set_xlim(max(0.0, t_now - WINDOW_S), t_now + 1.0)
    while ax.collections: ax.collections[0].remove()
    tr_w = np.array(htr)[mask]
    ax.fill_between(t_win, ALERT_TR, np.clip(tr_w, ALERT_TR, None), color='#FFC107', alpha=0.18)
    ax.fill_between(t_win, CRIT_TR,  np.clip(tr_w, CRIT_TR,  None), color='#F44336', alpha=0.22)


def _setup_area_ax(ax):
    ax.set_title('Support Triangle Area vs. Time', fontsize=9)
    ax.set_xlabel('Time [s]', fontsize=8); ax.set_ylabel('Area [m²]', fontsize=8)
    ax.tick_params(labelsize=7)
    ln_area, = ax.plot([], [], color='#00796B', lw=1.3)
    return ln_area


def _update_area(ax, ln_area, ht, harea):
    if not ht: return
    t_arr = np.array(ht); t_now = t_arr[-1]
    mask  = t_arr >= (t_now - WINDOW_S)
    t_win = t_arr[mask]; a_win = np.array(harea)[mask]
    ln_area.set_data(t_win, a_win)
    ax.set_xlim(max(0.0, t_now - WINDOW_S), t_now + 1.0)
    if len(a_win) > 1:
        rng = a_win.ptp(); pad = max(rng * 0.12, 2e-5)
        ax.set_ylim(a_win.min() - pad, a_win.max() + pad)


def _draw_histogram(ax, htr):
    ax.cla()
    ax.set_title('TR Distribution (Session)', fontsize=9)
    ax.set_xlabel('Tipover Risk (TR)', fontsize=8); ax.set_ylabel('Count', fontsize=8)
    ax.tick_params(labelsize=7)
    if not htr:
        ax.text(0.5, 0.5, 'No data yet', ha='center', va='center',
                transform=ax.transAxes, fontsize=9, color='gray')
        return
    tr_arr = np.array(htr)
    bins   = np.linspace(min(tr_arr.min()-0.05, 0.0), max(tr_arr.max()+0.05, 1.5), 35)
    n, edges, patches = ax.hist(tr_arr, bins=bins, color='#546E7A', edgecolor='white', lw=0.4)
    for patch, left in zip(patches, edges[:-1]):
        if left >= CRIT_TR:  patch.set_facecolor('#D32F2F')
        elif left >= ALERT_TR: patch.set_facecolor('#F9A825')
    ax.axvline(ALERT_TR, color='#F9A825', ls='--', lw=1.3, label='0.8 alert')
    ax.axvline(CRIT_TR,  color='#D32F2F', ls='--', lw=1.3, label='1.0 critical')
    ax.legend(fontsize=7)
    ax.text(0.97, 0.97,
            f'n={len(tr_arr)}\nmean={tr_arr.mean():.3f}\nσ={tr_arr.std():.3f}\nmin={tr_arr.min():.3f}',
            transform=ax.transAxes, fontsize=7.5, va='top', ha='right',
            bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.85))


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    node = StabilityMonitor()

    spin_thread = threading.Thread(target=node.spin, daemon=True)
    spin_thread.start()

    fig = plt.figure(figsize=(14, 8))
    fig.suptitle('PETER Quadruped — Static Stability Analysis', fontsize=12, fontweight='bold')

    ax_poly = fig.add_subplot(2, 2, 1)
    ax_ts   = fig.add_subplot(2, 2, 2)
    ax_area = fig.add_subplot(2, 2, 3)
    ax_hist = fig.add_subplot(2, 2, 4)

    _setup_polygon_ax(ax_poly)
    ln_smn, ln_tr = _setup_ts_ax(ax_ts)
    ln_area       = _setup_area_ax(ax_area)
    _draw_histogram(ax_hist, [])
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    def animate(_frame):
        with node.lock:
            pose   = node._latest_pose
            sm_res = node._latest_sm
            ht     = list(node._ht)
            hsmn   = list(node._hsmn)
            htr    = list(node._htr)
            harea  = list(node._harea)
        if pose is not None:
            _draw_polygon(ax_poly, pose, sm_res)
        _update_ts(ax_ts, ln_smn, ln_tr, ht, hsmn, htr)
        _update_area(ax_area, ln_area, ht, harea)
        _draw_histogram(ax_hist, htr)
        fig.tight_layout(rect=[0, 0, 1, 0.95])

    ani = FuncAnimation(fig, animate, interval=150, cache_frame_data=False)  # noqa

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.print_stats()
        node.save_csv()
        node.destroy_node()


if __name__ == '__main__':
    main()

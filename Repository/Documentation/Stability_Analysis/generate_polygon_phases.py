#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gait Cycle Polygon Phases Generator (Publication Version).
Generates a chronologically ordered sequence of the gait cycle,
including intermediate 4-leg support phases, with technical English terminology.
"""

import math
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.patches import Polygon, Circle

# ── 1. PUBLICATION STYLE SETTINGS ──
def set_publication_style():
    plt.rcParams.update({
        'font.family': 'serif',
        'font.size': 10,
        'axes.labelsize': 11,
        'axes.titlesize': 12,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'legend.fontsize': 10,
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

# ── 2. KINEMATIC GEOMETRY UTILITIES ──
def _sort_ccw(pts):
    """Sorts points in counter-clockwise order to properly render convex polygons."""
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    return sorted(pts, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))

# ── 3. CHRONOLOGICAL PHASE RENDERING ──
def plot_chronological_gait(csv_path, output_pdf="fig_gait_polygon_phases.pdf"):
    df = pd.read_csv(csv_path)
    
    # Map leg indices to nomenclature
    leg_names = {0: 'RU', 1: 'LU', 2: 'RD', 3: 'LD'}
    
    # Detect chronological phases by finding changes in 'leg_in_air'
    df['phase_block'] = (df['leg_in_air'] != df['leg_in_air'].shift()).cumsum()
    
    # Extract stable representative samples from each block
    valid_blocks = df['phase_block'].unique()[1:-1] # Drop boundary blocks to avoid noise
    phase_rows = []
    
    for b in valid_blocks:
        block_data = df[df['phase_block'] == b]
        if len(block_data) >= 3: # Filter transient artifacts
            mid_idx = len(block_data) // 2
            phase_rows.append(block_data.iloc[mid_idx])
            
    sampled_df = pd.DataFrame(phase_rows)
    
    # Limit to exactly 8 phases to show one complete and logical gait cycle sequence
    n_phases = min(len(sampled_df), 8)
    sampled_df = sampled_df.iloc[:n_phases]
    
    # Setup 2x4 Grid
    fig, axes = plt.subplots(2, 4, figsize=(16, 8))
    axes = axes.flatten()
    
    for idx, (ax_idx, row) in enumerate(zip(range(8), sampled_df.iterrows())):
        ax = axes[ax_idx]
        row_data = row[1]
        phase_val = int(row_data['leg_in_air'])
        
        # Extract planar foot coordinates (Base frame)
        feet = [
            (row_data['foot_RU_x'], row_data['foot_RU_y']),
            (row_data['foot_LU_x'], row_data['foot_LU_y']),
            (row_data['foot_RD_x'], row_data['foot_RD_y']),
            (row_data['foot_LD_x'], row_data['foot_LD_y'])
        ]
        
        sm = row_data['SM']
        com = (0.0, 0.0) # CoM is strictly at the origin in the base frame
        
        if phase_val in leg_names:
            # 3-Leg Support Phase
            ground_feet = [feet[i] for i in range(4) if i != phase_val]
            swing_foot = feet[phase_val]
            phase_title = f"Step {idx+1}: {leg_names[phase_val]} Swing Phase"
        else:
            # 4-Leg Support Phase (CoM shift forward)
            ground_feet = feet
            swing_foot = None
            phase_title = f"Step {idx+1}: 4-Leg Support\n(CoM Impulse)"
            
        ground_feet = _sort_ccw(ground_feet)
        
        # Render Support Polygon
        poly = Polygon(ground_feet, closed=True, alpha=0.15, color='#388E3C', edgecolor='#388E3C', lw=2)
        ax.add_patch(poly)
        xs, ys = zip(*(ground_feet + [ground_feet[0]]))
        ax.plot(xs, ys, color='#388E3C', lw=1.5, label="Support Polygon" if ax_idx==0 else "")
        
        # Render Stability Margin (SM as radius around CoM per McGhee & Frank)
        sm_circle = Circle(com, sm, color='#1976D2', fill=False, linestyle='--', alpha=0.8, lw=1.5, label="Stability Margin (SM)" if ax_idx==0 else "")
        ax.add_patch(sm_circle)
        
        # Render Center of Mass
        ax.plot(*com, marker='P', color='black', markersize=8, label="Center of Mass (CoM)" if ax_idx==0 else "")
        
        # Direction of Motion Arrow (Forward X-axis impulse)
        if ax_idx == 0:
            ax.arrow(0, 0, 0.06, 0, head_width=0.015, head_length=0.02, fc='black', ec='black', alpha=0.8, label="Direction of Motion")
        else:
            ax.arrow(0, 0, 0.06, 0, head_width=0.015, head_length=0.02, fc='black', ec='black', alpha=0.8)
        
        # Render Feet
        for j, (fx, fy) in enumerate(ground_feet):
            ax.plot(fx, fy, 'o', color='#F57C00', markersize=6, label="Support Feet" if ax_idx==0 and j==0 else "")
            
        if swing_foot:
            ax.plot(*swing_foot, 'o', color='#D32F2F', markersize=7, alpha=0.6, label="Swing Foot" if ax_idx==0 else "")
        
        # Axis Settings
        ax.set_aspect('equal')
        ax.set_xlim([-0.25, 0.25])
        ax.set_ylim([-0.3, 0.3])
        ax.set_title(f"{phase_title}\nSM: {sm:.3f} m", fontsize=11)
        ax.set_xlabel("X-Axis [m]")
        ax.set_ylabel("Y-Axis [m]")
        
    # Hide empty subplots if the sequence had fewer than 8 phases
    for ax_idx in range(n_phases, 8):
        axes[ax_idx].set_visible(False)
        
    # Global Legend Placement
    fig.legend(loc='lower center', ncol=6, bbox_to_anchor=(0.5, -0.05), frameon=False)
    plt.tight_layout()
    
    # Export Vectorial Graphics
    plt.savefig(output_pdf)
    plt.savefig(output_pdf.replace('.pdf', '.png'))
    print(f"[OK] Chronological sequence exported to: {output_pdf}")

if __name__ == '__main__':
    set_publication_style()
    plot_chronological_gait('familia_c2_pendiente\\test_003_SUCCESS\\stability_log.csv')
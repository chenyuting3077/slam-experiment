#!/usr/bin/env python3
"""Dot plot of start-to-end distance (should be ~0 on a closed loop) per system for the two lidars.

usage: plot_end_to_end_compare.py OUTPUTS_COMPAL_AMR_DIR OUT.png
Reads dataset3_vanjee_full and dataset3_livox_full under the given folder. Log x axis. Not an accuracy ranking:
there is no ground truth, and the closed loop is inferred, not verified.
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

INK, INK2, GRID, SURFACE = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
# Color follows the system (as in the other lab figures); the lidar is shown by marker: hollow circle = Vanjee, filled diamond = Livox.
BLUE, ORANGE, AQUA, YELLOW = '#2a78d6', '#eb6834', '#1baf7a', '#eda100'
ROWS = [
    ('Cartographer', 'cartographer/trajectory_tum.txt', 'cartographer/trajectory_tum.txt', BLUE),
    ('Cartographer\n(Xsens IMU)', None, 'cartographer_xsens/trajectory_tum.txt', BLUE),
    ('RTAB-Map', 'rtabmap/rtabmap_optimized_tum.txt', 'rtabmap/rtabmap_optimized_tum.txt', ORANGE),
    ('FAST-LIO2', 'fastlio2/fastlio_trajectory_tum.txt', 'fastlio2/fastlio_trajectory_tum.txt', AQUA),
    ('LIO-SAM', 'liosam/liosam_trajectory_tum.txt', 'liosam/liosam_trajectory_tum.txt', YELLOW),
]


def dist(p):
    if p is None or not os.path.exists(p):
        return None
    a = np.loadtxt(p)
    a = a[np.argsort(a[:, 0])]
    return float(np.linalg.norm(a[-1, 1:4] - a[0, 1:4]))


base = sys.argv[1]
vd, ld = (os.path.join(base, 'dataset3_vanjee_full'), os.path.join(base, 'dataset3_livox_full'))
fig, ax = plt.subplots(figsize=(9.5, 4.6), facecolor=SURFACE)
ax.set_facecolor(SURFACE)
for s in ('top', 'right', 'left'):
    ax.spines[s].set_visible(False)
ax.spines['bottom'].set_color(GRID)
ax.grid(axis='x', color=GRID, lw=0.8)
ax.tick_params(colors=INK2, labelsize=9, length=0)
for i, (label, vrel, lrel, col) in enumerate(ROWS):
    y = len(ROWS) - 1 - i
    ax.axhline(y, color=GRID, lw=0.8, zorder=0)
    for d, mk, filled, dy in ((dist(os.path.join(vd, vrel)) if vrel else None, 'o', False, 0.13),
                              (dist(os.path.join(ld, lrel)), 'D', True, -0.13)):
        if d is None:
            continue
        d = max(d, 0.01)
        ax.plot(d, y + dy, mk, color=col, mfc=col if filled else SURFACE, mew=2, ms=9, zorder=3)
        ax.annotate(f'{d:,.0f} m' if d >= 100 else f'{d:.3g} m', (d, y + dy), (9, -3), textcoords='offset points', color=INK, fontsize=8.5)
ax.set_xscale('log')
ax.set_xlim(0.008, 3e4)
ax.set_yticks(range(len(ROWS)))
ax.set_yticklabels([r[0] for r in ROWS][::-1], color=INK)
ax.set_ylim(-0.6, len(ROWS) - 0.4)
ax.set_xlabel('start-to-end distance (m, log) -- 0 on a perfectly closed loop', color=INK2)
ax.plot([], [], 'o', color=INK2, mfc=SURFACE, mew=2, ms=9, label='Vanjee (4-line, merged), Xsens IMU')
ax.plot([], [], 'D', color=INK2, ms=9, label='Livox Mid360 + Livox IMU')
ax.legend(frameon=False, fontsize=9, labelcolor=INK2, loc='upper right')
ax.set_title('Full bag, 809 s: does the trajectory return to its start?', color=INK, fontsize=11, loc='left')
fig.tight_layout()
fig.savefig(sys.argv[2], dpi=130, facecolor=SURFACE)
print('saved', sys.argv[2])

#!/usr/bin/env python3
"""Trajectory comparison for a Compal AMR lab run folder (outputs/compal_amr/<run>/), e.g. dataset3_livox_full.

usage: plot_lab_livox_comparison.py RUN_DIR OUT.png
Left: top view of every trajectory, each rigidly (Umeyama SE3, over the whole overlap, like an ATE) aligned to the reference. Right: position error vs time against the reference.
The reference is Cartographer with the Xsens IMU (it closes the loop best) -- NOT ground truth; there is none.
Missing trajectories are skipped.
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

INK, INK2, GRID, SURFACE = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
# Color follows the system, same as trajectory_compare_compal_amr_full.png: Cartographer blue, RTAB-Map orange,
# FAST-LIO2 aqua (fixed categorical slots 1-3); LIO-SAM takes the next slot (yellow). The two Cartographer variants
# share the color and differ by line style.
SERIES = [
    ('Cartographer (Livox IMU)', 'cartographer/trajectory_tum.txt', '#2a78d6', '-'),
    ('RTAB-Map', 'rtabmap/rtabmap_optimized_tum.txt', '#eb6834', '-'),
    ('FAST-LIO2', 'fastlio2/fastlio_trajectory_tum.txt', '#1baf7a', '-'),
    ('LIO-SAM', 'liosam/liosam_trajectory_tum.txt', '#eda100', '-'),
]
REF = ('Cartographer (Xsens IMU, reference)', 'cartographer_xsens/trajectory_tum.txt', '#2a78d6', (0, (5, 2, 1, 2)))
ODOM = ('/odometry', 'reference/odometry_tum.txt', '#8a8985', (0, (4, 3)))


def load(p):
    a = np.loadtxt(p)
    return a[np.argsort(a[:, 0])]


def at(ref, t):
    return np.stack([np.interp(t, ref[:, 0], ref[:, k]) for k in (1, 2, 3)], 1)


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    u, _, vt = np.linalg.svd((dst - mu_d).T @ (src - mu_s) / len(src))
    d = np.eye(3)
    d[2, 2] = np.sign(np.linalg.det(u @ vt))
    r = u @ d @ vt
    return r, mu_d - r @ mu_s


def main():
    run, out = sys.argv[1:3]
    ref = load(os.path.join(run, REF[1]))
    t0 = ref[0, 0]
    curves = []
    for name, rel, color, ls in [ODOM] + SERIES:
        p = os.path.join(run, rel)
        if not os.path.exists(p):
            continue
        e = load(p)
        ok = (e[:, 0] >= ref[0, 0]) & (e[:, 0] <= ref[-1, 0])
        e = e[ok]
        r, t = umeyama(e[:, 1:4], at(ref, e[:, 0]))
        pa = e[:, 1:4] @ r.T + t
        err = np.linalg.norm(pa - at(ref, e[:, 0]), axis=1)
        curves.append((name, color, ls, e[:, 0] - t0, pa, err))

    c = ref[:, 1:3].mean(0)
    _, _, vt = np.linalg.svd(ref[:, 1:3] - c, full_matrices=False)

    def top(p):
        return (p[:, :2] - c) @ vt.T

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5.2), facecolor=SURFACE, gridspec_kw={'width_ratios': [1.15, 1]})
    for a in (a1, a2):
        a.set_facecolor(SURFACE)
        a.grid(color=GRID, lw=0.8)
        for s in ('top', 'right'):
            a.spines[s].set_visible(False)
        for s in ('left', 'bottom'):
            a.spines[s].set_color(GRID)
        a.tick_params(colors=INK2, labelsize=9)

    rp = top(ref[:, 1:4])
    a1.plot(rp[:, 0], rp[:, 1], color=REF[2], lw=3.2, ls=REF[3], alpha=0.55, label=REF[0])
    for name, color, ls, t, pa, err in curves:
        q = top(pa)
        a1.plot(q[::max(1, len(q) // 3000), 0], q[::max(1, len(q) // 3000), 1], color=color, lw=1.6, ls=ls, label=name)
    a1.plot(*rp[0], 'o', color=INK, ms=8, mec=SURFACE, mew=2)
    a1.annotate('start', rp[0], (8, 8), textcoords='offset points', color=INK, fontsize=9)
    a1.set_aspect('equal', adjustable='datalim')
    a1.set_xlabel('along the main axis (m)', color=INK2)
    a1.set_ylabel('across (m)', color=INK2)
    a1.set_title('Top view (each trajectory SE3-aligned to the reference)', color=INK, fontsize=11, loc='left')
    a1.legend(frameon=False, fontsize=8.5, labelcolor=INK2, loc='upper right')

    for k, (name, color, ls, t, pa, err) in enumerate(curves):
        a2.plot(t, np.maximum(err, 0.03), color=color, lw=1.8, ls=ls, label=name)
        a2.annotate(name.split(' (')[0], (t[-1], max(err[-1], 0.03)), (-4, 6 + 11 * (k % 3)), textcoords='offset points',
                    color=INK, fontsize=8.5, ha='right')
    a2.set_yscale('log')
    a2.set_xlabel('time (s)', color=INK2)
    a2.set_ylabel('position error vs reference (m, log)', color=INK2)
    a2.set_title('Disagreement with the reference over time', color=INK, fontsize=11, loc='left')
    fig.tight_layout()
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    print('saved', out)


if __name__ == '__main__':
    main()

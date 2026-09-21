#!/usr/bin/env python3
"""Trajectory overlay + FAST-LIO2 divergence plot for a Compal AMR lab run.

usage: plot_lab_trajectories.py REF_TUM CARTO_TUM RTAB_TUM FASTLIO_TUM out.png
"""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

INK, INK2, GRID, SURFACE = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
BLUE, ORANGE, AQUA, GRAY = '#2a78d6', '#eb6834', '#1baf7a', '#8a8985'


def load(p):
    a = np.loadtxt(p)
    return a[np.argsort(a[:, 0])]


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    u, _, vt = np.linalg.svd((dst - mu_d).T @ (src - mu_s) / len(src))
    d = np.eye(3)
    d[2, 2] = np.sign(np.linalg.det(u @ vt))
    r = u @ d @ vt
    return r, mu_d - r @ mu_s


def ref_at(ref, t):
    return np.stack([np.interp(t, ref[:, 0], ref[:, k]) for k in (1, 2, 3)], 1)


def aligned(est, ref, seconds=60):
    m = est[:, 0] - ref[0, 0] < seconds
    r, t = umeyama(est[m, 1:4], ref_at(ref, est[m, 0]))
    return est[:, 1:4] @ r.T + t


ref, carto, rtab, fl = (load(p) for p in sys.argv[1:5])
t0 = ref[0, 0]
pc, pr, pf = aligned(carto, ref), aligned(rtab, ref), aligned(fl, ref)

fig, (a1, a2) = plt.subplots(1, 2, figsize=(12.5, 5.2), facecolor=SURFACE,
                             gridspec_kw={'width_ratios': [1.1, 1]})
for a in (a1, a2):
    a.set_facecolor(SURFACE)
    a.grid(color=GRID, lw=0.8)
    for s in ('top', 'right'):
        a.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        a.spines[s].set_color(GRID)
    a.tick_params(colors=INK2, labelsize=9)

a1.plot(ref[::20, 1], ref[::20, 2], color=GRAY, lw=1.6, ls=(0, (4, 3)), label='/odometry (reference, drifts)')
a1.plot(pc[:, 0], pc[:, 1], color=BLUE, lw=2, label='Cartographer')
a1.plot(pr[:, 0], pr[:, 1], color=ORANGE, lw=2, label='RTAB-Map (optimized)')
a1.plot(*ref[0, 1:3], 'o', color=INK, ms=8, mec=SURFACE, mew=2)
a1.annotate('start', ref[0, 1:3], (8, 8), textcoords='offset points', color=INK, fontsize=9)
a1.plot(*ref[-1, 1:3], 's', color=GRAY, ms=8, mec=SURFACE, mew=2)
a1.annotate('/odometry end (9.3 m off)', ref[-1, 1:3], (8, -14), textcoords='offset points', color=INK2, fontsize=9)
a1.set_aspect('equal', adjustable='datalim')
a1.set_xlabel('x (m)', color=INK2)
a1.set_ylabel('y (m)', color=INK2)
a1.set_title('Full bag (809 s): loop closed by Cartographer and RTAB-Map', color=INK, fontsize=11, loc='left')
a1.legend(frameon=False, fontsize=9, labelcolor=INK2, loc='best')

tf = fl[:, 0] - t0
err = np.linalg.norm(pf - ref_at(ref, fl[:, 0]), axis=1)
a2.plot(tf, np.maximum(err, 0.01), color=AQUA, lw=2)
a2.set_yscale('log')
a2.set_xlabel('time (s)', color=INK2)
a2.set_ylabel('position error vs /odometry (m, log)', color=INK2)
a2.set_title('FAST-LIO2 (official) diverges', color=INK, fontsize=11, loc='left')
for thr, txt in ((5, 'first > 5 m'), (50, 'first > 50 m')):
    i = np.argmax(err > thr)
    a2.plot(tf[i], err[i], 'o', color=AQUA, ms=8, mec=SURFACE, mew=2)
    a2.annotate(f'{txt}: t = {tf[i]:.0f} s', (tf[i], err[i]), (-12, 10), textcoords='offset points',
                color=INK, fontsize=9, ha='right')
a2.annotate('FAST-LIO2', (tf[-1], err[-1]), (-6, 8), textcoords='offset points', color=INK, fontsize=9, ha='right')
fig.tight_layout()
fig.savefig(sys.argv[5], dpi=130, facecolor=SURFACE)
print('saved', sys.argv[5])

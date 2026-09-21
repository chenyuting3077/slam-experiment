#!/usr/bin/env python3
"""Headless top-down render of a point cloud (height-colored) plus the trajectory (white).

Same look as render_static_map.py (black background, red->blue height rainbow, white trajectory) but needs
only numpy/matplotlib -- no open3d/GL window. The view is rotated so the trajectory's main axis is
horizontal, and a scale bar is drawn.

usage: render_topdown_map.py TRAJ_TUM CLOUD.pcd OUT.png "title" [--max-points N]
"""
import argparse

import matplotlib
import matplotlib.colors
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

def height_rainbow(t, darken=0.55):
    """Same look as incremental_playback_viewer.height_rainbow: low=red -> yellow -> green -> cyan -> blue=high, darkened."""
    t = np.clip(t, 0.0, 1.0)
    hsv = np.stack([t * (240.0 / 360.0), np.ones_like(t), np.ones_like(t)], -1)
    return matplotlib.colors.hsv_to_rgb(hsv) * darken


NP_TYPES = {('F', 4): '<f4', ('F', 8): '<f8', ('U', 1): 'u1', ('U', 2): '<u2', ('U', 4): '<u4',
            ('I', 1): 'i1', ('I', 2): '<i2', ('I', 4): '<i4'}


def read_pcd_xyz(path, max_points):
    with open(path, 'rb') as f:
        hdr = {}
        while True:
            line = f.readline().decode('ascii', 'ignore').strip()
            k, _, v = line.partition(' ')
            hdr[k] = v
            if k == 'DATA':
                break
        offset = f.tell()
    fields, sizes, types = hdr['FIELDS'].split(), [int(s) for s in hdr['SIZE'].split()], hdr['TYPE'].split()
    n = int(hdr['POINTS'])
    rng = np.random.default_rng(0)
    if hdr['DATA'].startswith('binary') and hdr['DATA'] != 'binary_compressed':
        dt = np.dtype([(fn, NP_TYPES[(t, s)]) for fn, s, t in zip(fields, sizes, types)])
        arr = np.memmap(path, dtype=dt, mode='r', offset=offset, shape=(n,))
        idx = np.sort(rng.choice(n, max_points, replace=False)) if n > max_points else slice(None)
        sel = arr[idx]
        xyz = np.stack([sel['x'], sel['y'], sel['z']], 1).astype(np.float64)
    elif hdr['DATA'] == 'ascii':
        ix = [fields.index(c) for c in 'xyz']
        with open(path) as f:
            for _ in range(len(hdr)):
                f.readline()
            xyz = np.array([[float(ln.split()[i]) for i in ix] for ln in f if ln.strip()])
    else:
        raise SystemExit(f'unsupported PCD encoding {hdr["DATA"]}')
    return xyz[np.isfinite(xyz).all(1)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('trajectory')
    ap.add_argument('cloud')
    ap.add_argument('out')
    ap.add_argument('title')
    ap.add_argument('--max-points', type=int, default=4_000_000)
    ap.add_argument('--no-title', action='store_true', help='omit the in-image title (e.g. when a grid adds labels)')
    ap.add_argument('--start-height', type=float, default=None,
                    help='height above ground (m) of the trajectory frame at its first pose; if given, colors '
                         'use height above ground = z - z_start + this (assumes flat ground at the start)')
    ap.add_argument('--z-range', type=float, nargs=2, default=None, metavar=('LO', 'HI'),
                    help='fixed color range (m, above ground when --start-height is set); default is the '
                         "cloud's own 2nd..98th percentile, which is NOT comparable between tiles")
    ap.add_argument('--level-gravity', type=float, nargs=3, default=None, metavar=('AX', 'AY', 'AZ'),
                    help='accelerometer reading (any unit) of the sensor at rest in the trajectory/cloud frame; both are '
                         'rotated so that this direction becomes +z (for frames that are not gravity aligned, e.g. '
                         'FAST_LIO/LIO-SAM world frames = initial sensor frame)')
    ap.add_argument('--level-plane', action='store_true',
                    help="rotate trajectory and cloud so that the trajectory's least-squares plane is horizontal (assumes the "
                         'robot drove on roughly flat ground); for frames with unknown tilt, e.g. LIO-SAM')
    ap.add_argument('--rotate', type=float, default=0.0, help='extra view rotation in degrees (counter-clockwise)')
    ap.add_argument('--px', type=int, default=1280, help='long side of the raster in pixels')
    args = ap.parse_args()

    traj = np.loadtxt(args.trajectory)[:, 1:4]
    pts = read_pcd_xyz(args.cloud, args.max_points)
    if args.level_gravity is not None:
        g = np.array(args.level_gravity, float)
        g /= np.linalg.norm(g)
        v = np.cross(g, [0, 0, 1.0])
        c = g[2]
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        Rl = np.eye(3) + vx + vx @ vx / (1 + c)  # Rodrigues: rotates g onto +z
        traj, pts = traj @ Rl.T, pts @ Rl.T

    if args.level_plane:
        A = np.c_[traj[:, 0], traj[:, 1], np.ones(len(traj))]
        a_, b_, _ = np.linalg.lstsq(A, traj[:, 2], rcond=None)[0]
        n = np.array([-a_, -b_, 1.0])
        n /= np.linalg.norm(n)
        v = np.cross(n, [0, 0, 1.0])
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        Rp = np.eye(3) + vx + vx @ vx / (1 + n[2])
        traj, pts = traj @ Rp.T, pts @ Rp.T

    c = traj[:, :2].mean(0)
    _, _, vt = np.linalg.svd(traj[:, :2] - c, full_matrices=False)
    R = vt  # rows: main axis, second axis
    a = np.radians(args.rotate)
    R = np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]]) @ R
    tr2 = (traj[:, :2] - c) @ R.T
    pt2 = (pts[:, :2] - c) @ R.T

    lo, hi = tr2.min(0), tr2.max(0)
    pad = 0.08 * (hi - lo).max() + 3.0
    lo, hi = lo - pad, hi + pad
    keep = (pt2[:, 0] > lo[0]) & (pt2[:, 0] < hi[0]) & (pt2[:, 1] > lo[1]) & (pt2[:, 1] < hi[1])
    pt2, pz = pt2[keep], pts[keep, 2]
    if args.start_height is not None:
        pz = pz - traj[0, 2] + args.start_height

    span = hi - lo
    scale = args.px / span.max()
    W, H = int(np.ceil(span[0] * scale)), int(np.ceil(span[1] * scale))
    img = np.zeros((H, W, 3), np.float32)
    if len(pz):
        zlo, zhi = args.z_range if args.z_range else (np.percentile(pz, 2), np.percentile(pz, 98))
        col = height_rainbow((pz - zlo) / max(zhi - zlo, 1e-6))
        order = np.argsort(pz)
        ix = np.clip(((pt2[order, 0] - lo[0]) * scale).astype(int), 0, W - 1)
        iy = np.clip(((pt2[order, 1] - lo[1]) * scale).astype(int), 0, H - 1)
        img[iy, ix] = col[order]

    top = 0 if args.no_title else 50
    fig = plt.figure(figsize=(W / 100, (H + top) / 100), dpi=100, facecolor='black')
    ax = fig.add_axes([0, 0, 1, H / (H + top)])
    ax.imshow(img, origin='lower', extent=[lo[0], hi[0], lo[1], hi[1]], interpolation='nearest')
    ax.plot(tr2[:, 0], tr2[:, 1], color='white', lw=1.6)
    ax.plot(*tr2[0], 'o', color='white', ms=7, mec='black', mew=1.5)
    ax.plot(*tr2[-1], 's', color='white', ms=7, mec='black', mew=1.5)
    ax.axis('off')
    ax.set_aspect('equal')
    bar = 10 ** np.floor(np.log10(span.max() / 4))
    x0, y0 = lo[0] + 0.03 * span[0], lo[1] + 0.04 * span[1]
    ax.plot([x0, x0 + bar], [y0, y0], color='white', lw=3)
    ax.text(x0, y0 + 0.012 * span[1], f'{bar:g} m', color='white', fontsize=11, va='bottom')
    if args.z_range:
        cb = ax.inset_axes([0.70, 0.11, 0.26, 0.05])
        cb.imshow(height_rainbow(np.linspace(0, 1, 256))[None, :, :], aspect='auto')
        cb.set_yticks([])
        cb.set_xticks([0, 255])
        cb.set_xticklabels([f'{args.z_range[0]:g} m', f'{args.z_range[1]:g} m'], color='white', fontsize=9)
        cb.tick_params(length=0, pad=2)
        for sp in cb.spines.values():
            sp.set_visible(False)
        cb.set_title('height above ground' if args.start_height is not None else 'height (z)',
                     color='white', fontsize=9, pad=2)
    if not args.no_title:
        fig.text(0.5, 1 - 25 / (H + top), args.title, color='white', fontsize=15, fontweight='bold',
                 ha='center', va='center')
    fig.savefig(args.out, facecolor='black')
    print(f'{args.out}: {len(pts)} pts read, {int(keep.sum())} drawn, raster {W}x{H}, z range [{pz.min():.1f},{pz.max():.1f}]'
          if len(pz) else f'{args.out}: no points in view')


if __name__ == '__main__':
    main()

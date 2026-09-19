#!/usr/bin/env python3
"""Compute Absolute Trajectory Error (ATE) of an estimated TUM trajectory
against a ground-truth TUM trajectory: associate poses by nearest timestamp,
find the best-fit SE(3) alignment (rotation + translation, no scale -- both
already metric) via Umeyama/Kabsch on the associated positions, then report
RMSE/mean/median/max of the per-pose position error after alignment.
"""
import argparse

import numpy as np


def load_tum(path):
    times, trans, quats = [], [], []
    with open(path) as f:
        for line in f:
            parts = line.split()
            if not parts or parts[0].startswith('#'):
                continue
            t = float(parts[0])
            x, y, z, qx, qy, qz, qw = (float(v) for v in parts[1:8])
            times.append(t)
            trans.append([x, y, z])
            quats.append([qx, qy, qz, qw])
    return np.array(times), np.array(trans), np.array(quats)


def associate(times_a, times_b, max_dt=0.1):
    """For each time in a, find the closest time in b within max_dt."""
    idx_b = np.searchsorted(times_b, times_a)
    pairs = []
    for i, j in enumerate(idx_b):
        candidates = [j - 1, j]
        best_j, best_dt = None, None
        for c in candidates:
            if 0 <= c < len(times_b):
                dt = abs(times_b[c] - times_a[i])
                if best_dt is None or dt < best_dt:
                    best_dt, best_j = dt, c
        if best_j is not None and best_dt <= max_dt:
            pairs.append((i, best_j))
    return pairs


def align_se3(p, q):
    """Best-fit R (3x3, det=+1) and t (3,) mapping points p onto q (Umeyama, no scale)."""
    pc, qc = p - p.mean(axis=0), q - q.mean(axis=0)
    h = pc.T @ qc
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = vt.T @ u.T
    t = q.mean(axis=0) - r @ p.mean(axis=0)
    return r, t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('estimate', help='TUM trajectory to evaluate')
    ap.add_argument('groundtruth', help='TUM ground-truth trajectory')
    ap.add_argument('--max-dt', type=float, default=0.1, help='max time gap (s) for association')
    args = ap.parse_args()

    times_e, trans_e, _ = load_tum(args.estimate)
    times_g, trans_g, _ = load_tum(args.groundtruth)

    pairs = associate(times_e, times_g, args.max_dt)
    if len(pairs) < 3:
        print(f"{args.estimate}: only {len(pairs)} associated poses (need >=3) -- cannot align.")
        return

    p = np.array([trans_e[i] for i, j in pairs])
    q = np.array([trans_g[j] for i, j in pairs])
    r, t = align_se3(p, q)
    p_aligned = (r @ p.T).T + t
    errors = np.linalg.norm(p_aligned - q, axis=1)

    rmse = np.sqrt((errors ** 2).mean())
    print(f"{args.estimate}: {len(pairs)}/{len(times_e)} poses associated "
          f"(of {len(times_g)} ground-truth poses)")
    print(f"  ATE RMSE={rmse:.3f}m  mean={errors.mean():.3f}m  "
          f"median={np.median(errors):.3f}m  max={errors.max():.3f}m  std={errors.std():.3f}m")


if __name__ == '__main__':
    main()

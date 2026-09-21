#!/usr/bin/env python3
"""ATE (Umeyama SE3, no scale) and path stats of TUM trajectories against a reference TUM trajectory.

usage: compare_to_reference.py REF_TUM NAME=EST_TUM [NAME=EST_TUM ...]
The reference is interpolated at each estimate's timestamps; only overlapping times are used.
"""
import sys

import numpy as np


def load(path):
    a = np.loadtxt(path)
    return a[np.argsort(a[:, 0])]


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    u, _, vt = np.linalg.svd((dst - mu_d).T @ (src - mu_s) / len(src))
    d = np.eye(3)
    d[2, 2] = np.sign(np.linalg.det(u @ vt))
    r = u @ d @ vt
    return r, mu_d - r @ mu_s


def main():
    ref = load(sys.argv[1])
    print(f"reference: {len(ref)} poses, path {np.linalg.norm(np.diff(ref[:,1:4],axis=0),axis=1).sum():.2f}m")
    for arg in sys.argv[2:]:
        name, path = arg.split('=', 1)
        est = load(path)
        m = (est[:, 0] >= ref[0, 0]) & (est[:, 0] <= ref[-1, 0])
        est = est[m]
        r_pos = np.stack([np.interp(est[:, 0], ref[:, 0], ref[:, k]) for k in (1, 2, 3)], 1)
        e_pos = est[:, 1:4]
        R, t = umeyama(e_pos, r_pos)
        err = np.linalg.norm((e_pos @ R.T + t) - r_pos, axis=1)
        length = np.linalg.norm(np.diff(e_pos, axis=0), axis=1).sum()
        print(f"{name}: poses={len(est)} path={length:.2f}m ATE_rmse={np.sqrt((err**2).mean()):.3f}m "
              f"max={err.max():.3f}m z_range=[{e_pos[:,2].min():.2f},{e_pos[:,2].max():.2f}]")


if __name__ == '__main__':
    main()

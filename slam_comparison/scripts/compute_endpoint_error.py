#!/usr/bin/env python3
"""Compute end-to-end translation error (Euclidean distance between first
and last pose) for a TUM-format trajectory file, matching how LIO-SAM's
Table II "End-to-end translation error" is defined."""
import sys

import numpy as np


def main(path: str) -> None:
    with open(path) as f:
        lines = [l.strip() for l in f if l.strip() and not l.startswith('#')]

    first = np.array([float(x) for x in lines[0].split()[1:4]])
    last = np.array([float(x) for x in lines[-1].split()[1:4]])
    error = np.linalg.norm(last - first)

    path_length = 0.0
    prev = first
    for line in lines[1:]:
        p = np.array([float(x) for x in line.split()[1:4]])
        path_length += np.linalg.norm(p - prev)
        prev = p

    print(f"{path}: poses={len(lines)} path_length={path_length:.2f}m "
          f"end-to-end error={error:.3f}m "
          f"start=({first[0]:.2f},{first[1]:.2f},{first[2]:.2f}) "
          f"end=({last[0]:.2f},{last[1]:.2f},{last[2]:.2f})")


if __name__ == '__main__':
    for p in sys.argv[1:]:
        main(p)

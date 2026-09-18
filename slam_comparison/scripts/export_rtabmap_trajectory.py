#!/usr/bin/env python3
"""Export RTAB-Map's rtabmap.db Node poses to a TUM-format trajectory file.

RTAB-Map stores each node's pose as a 3x4 float (row-major) transformation
matrix blob: [R | t]. TUM format is "timestamp x y z qx qy qz qw".
"""
import sqlite3
import struct
import sys

import numpy as np


def matrix_to_quaternion(rot):
    """3x3 rotation matrix -> (qx, qy, qz, qw), standard Shepperd's method."""
    trace = rot[0, 0] + rot[1, 1] + rot[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (rot[2, 1] - rot[1, 2]) * s
        qy = (rot[0, 2] - rot[2, 0]) * s
        qz = (rot[1, 0] - rot[0, 1]) * s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])
        qw = (rot[2, 1] - rot[1, 2]) / s
        qx = 0.25 * s
        qy = (rot[0, 1] + rot[1, 0]) / s
        qz = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2])
        qw = (rot[0, 2] - rot[2, 0]) / s
        qx = (rot[0, 1] + rot[1, 0]) / s
        qy = 0.25 * s
        qz = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1])
        qw = (rot[1, 0] - rot[0, 1]) / s
        qx = (rot[0, 2] + rot[2, 0]) / s
        qy = (rot[1, 2] + rot[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def main(db_path: str, out_path: str) -> None:
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("SELECT id, stamp, pose FROM Node WHERE pose IS NOT NULL ORDER BY id")
    rows = cur.fetchall()

    with open(out_path, 'w') as f:
        for node_id, stamp, pose_blob in rows:
            if pose_blob is None or len(pose_blob) < 48:
                continue
            vals = struct.unpack('<12f', pose_blob[:48])
            mat = np.array(vals, dtype=np.float64).reshape(3, 4)
            rot = mat[:, :3]
            trans = mat[:, 3]
            qx, qy, qz, qw = matrix_to_quaternion(rot)
            f.write(f"{stamp:.6f} {trans[0]:.6f} {trans[1]:.6f} {trans[2]:.6f} "
                    f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")

    print(f"Wrote {len(rows)} poses to {out_path}")


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])

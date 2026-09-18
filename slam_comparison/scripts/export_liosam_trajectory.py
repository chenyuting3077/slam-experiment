#!/usr/bin/env python3
"""Export LIO-SAM's transformations.pcd (x y z intensity roll pitch yaw time,
binary PCD) keyframe poses to a TUM-format trajectory file."""
import struct
import sys

import numpy as np


def euler_to_quaternion(roll, pitch, yaw):
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def parse_pcd_header(f):
    fields, sizes, types, counts = None, None, None, None
    npoints = None
    while True:
        line = f.readline().decode('ascii').strip()
        if line.startswith('FIELDS'):
            fields = line.split()[1:]
        elif line.startswith('SIZE'):
            sizes = [int(x) for x in line.split()[1:]]
        elif line.startswith('TYPE'):
            types = line.split()[1:]
        elif line.startswith('COUNT'):
            counts = [int(x) for x in line.split()[1:]]
        elif line.startswith('POINTS'):
            npoints = int(line.split()[1])
        elif line.startswith('DATA'):
            assert 'binary' in line
            break
    return fields, sizes, types, npoints


TYPE_MAP = {('F', 4): 'f', ('F', 8): 'd', ('I', 4): 'i', ('U', 4): 'I'}


def main(pcd_path: str, out_path: str) -> None:
    with open(pcd_path, 'rb') as f:
        fields, sizes, types, npoints = parse_pcd_header(f)
        fmt = '<' + ''.join(TYPE_MAP[(t, s)] for t, s in zip(types, sizes))
        point_step = struct.calcsize(fmt)
        data = f.read(point_step * npoints)

    idx = {name: i for i, name in enumerate(fields)}
    with open(out_path, 'w') as out:
        for i in range(npoints):
            vals = struct.unpack_from(fmt, data, i * point_step)
            x, y, z = vals[idx['x']], vals[idx['y']], vals[idx['z']]
            roll, pitch, yaw = vals[idx['roll']], vals[idx['pitch']], vals[idx['yaw']]
            t = vals[idx['time']]
            qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
            out.write(f"{t:.6f} {x:.6f} {y:.6f} {z:.6f} "
                      f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
    print(f"Wrote {npoints} poses to {out_path}")


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])

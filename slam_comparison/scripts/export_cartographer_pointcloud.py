#!/usr/bin/env python3
"""Build a world-frame point cloud PCD for Cartographer from its exported
TUM trajectory (cartographer_campus.txt) + the raw /points_raw scans in the
bag. cartographer_assets_writer itself hits a hardcoded internal grid-size
limit (hybrid_grid.h "new_bits <= 8") on a map this large, so we transform
points ourselves using the poses cartographer already computed.
"""
import struct
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader


def load_tum(path):
    times, poses = [], []
    with open(path) as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            t = float(parts[0])
            x, y, z, qx, qy, qz, qw = (float(v) for v in parts[1:8])
            # quaternion -> 3x3 rotation matrix
            n = qx * qx + qy * qy + qz * qz + qw * qw
            s = 2.0 / n
            R = np.array([
                [1 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw), s * (qx * qz + qy * qw)],
                [s * (qx * qy + qz * qw), 1 - s * (qx * qx + qz * qz), s * (qy * qz - qx * qw)],
                [s * (qx * qz - qy * qw), s * (qy * qz + qx * qw), 1 - s * (qx * qx + qy * qy)],
            ])
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = [x, y, z]
            times.append(t)
            poses.append(T)
    return np.array(times), poses


def voxel_downsample(points, voxel_size):
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[idx]


def main(bag_path, tum_path, out_pcd, point_stride=3, voxel_size=0.1):
    times, poses = load_tum(tum_path)
    accumulated = []

    with AnyReader([Path(bag_path)]) as reader:
        conns = [c for c in reader.connections if c.topic == '/points_raw']
        count = 0
        for conn, timestamp, rawdata in reader.messages(connections=conns):
            msg = reader.deserialize(rawdata, conn.msgtype)
            t = timestamp / 1e9
            i = np.searchsorted(times, t)
            if i == 0 or i >= len(times):
                continue
            # nearest of the two neighbors
            if abs(times[i] - t) > abs(times[i - 1] - t):
                i -= 1
            if abs(times[i] - t) > 0.5:
                continue
            pose = poses[i]

            data = np.frombuffer(msg.data, dtype=np.uint8)
            pts = data.reshape(-1, msg.point_step)
            xyz = np.frombuffer(pts[:, 0:12].tobytes(), dtype=np.float32).reshape(-1, 3)
            xyz = xyz[::point_stride]
            valid = np.isfinite(xyz).all(axis=1)
            xyz = xyz[valid]
            if len(xyz) == 0:
                continue

            xyz_h = np.hstack([xyz, np.ones((len(xyz), 1), dtype=np.float32)])
            world = (pose @ xyz_h.T).T[:, :3]
            accumulated.append(world.astype(np.float32))
            count += 1
            if count % 1000 == 0:
                print(f"  processed {count} scans, accumulated points so far: "
                      f"{sum(len(a) for a in accumulated)}", flush=True)

    all_points = np.vstack(accumulated)
    print(f"Total raw points before voxel downsample: {len(all_points)}")
    all_points = voxel_downsample(all_points, voxel_size)
    print(f"Total points after voxel downsample ({voxel_size}m): {len(all_points)}")

    with open(out_pcd, 'wb') as f:
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            f"WIDTH {len(all_points)}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {len(all_points)}\n"
            "DATA binary\n"
        )
        f.write(header.encode('ascii'))
        f.write(all_points.astype('<f4').tobytes())
    print(f"Wrote {out_pcd}")


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3])

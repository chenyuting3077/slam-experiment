#!/usr/bin/env python3
"""Build a world-frame point cloud for RTAB-Map from its optimized node poses + the raw scans.

rtabmap-export --scan produces 0 points for these databases, so pair each optimized node pose
(TUM: stamp x y z qx qy qz qw) with the raw /vanjee_points719e_merged scan of the same stamp
(the cloud is already in base_link, the RTAB-Map frame_id) and transform it.

usage: assemble_rtabmap_cloud.py BAG_DIR poses_tum.txt out.pcd [voxel_m] [--topic T] [--sensor-in-base x y z qx qy qz qw]
  --topic       PointCloud2 topic (default /vanjee_points719e_merged); x/y/z are read from the message fields
  --sensor-in-base  pose of the cloud's frame in RTAB-Map's frame_id (default identity, as for the Vanjee cloud)
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

import argparse

ap = argparse.ArgumentParser()
ap.add_argument("bag")
ap.add_argument("poses")
ap.add_argument("out")
ap.add_argument("voxel", nargs="?", type=float, default=0.1)
ap.add_argument("--topic", default="/vanjee_points719e_merged")
ap.add_argument("--sensor-in-base", type=float, nargs=7, default=[0, 0, 0, 0, 0, 0, 1])
args = ap.parse_args()
bag, poses_path, out, voxel = args.bag, args.poses, args.out, args.voxel
poses = np.loadtxt(poses_path)
stamps = poses[:, 0]


def rot(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n
    return np.array([[1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
                     [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
                     [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions("", ""))
r.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
S_T = np.array(args.sensor_in_base[:3])
S_R = None
chunks, used = [], set()
while r.has_next():
    _, data, _ = r.read_next()
    m = deserialize_message(data, PointCloud2)
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    i = int(np.argmin(np.abs(stamps - t)))
    if abs(stamps[i] - t) > 0.06 or i in used:
        continue
    used.add(i)
    names = {f.name: f.offset for f in m.fields}
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                   "offsets": [names["x"], names["y"], names["z"]], "itemsize": m.point_step})
    pts = np.frombuffer(bytes(m.data), dtype=dt, count=m.width)
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(np.float64)
    xyz = xyz[np.isfinite(xyz).all(1)]
    if S_R is None:
        S_R = rot(args.sensor_in_base[3:])
    xyz = xyz @ S_R.T + S_T
    R, p = rot(poses[i, 4:8]), poses[i, 1:4]
    w = xyz @ R.T + p
    chunks.append(np.unique(np.round(w / voxel).astype(np.int32), axis=0))
    if len(used) == len(stamps):
        break
allv = np.unique(np.concatenate(chunks), axis=0).astype(np.float32) * voxel
print(f"matched {len(used)}/{len(stamps)} nodes -> {len(allv)} points")
with open(out, "wb") as f:
    f.write((f"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
             f"WIDTH {len(allv)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(allv)}\nDATA binary\n").encode())
    f.write(allv.tobytes())

#!/usr/bin/env python3
"""Build a world-frame point cloud for RTAB-Map from its optimized node poses + the raw scans.

rtabmap-export --scan produces 0 points for these databases, so pair each optimized node pose
(TUM: stamp x y z qx qy qz qw) with the raw /vanjee_points719e_merged scan of the same stamp
(the cloud is already in base_link, the RTAB-Map frame_id) and transform it.

usage: assemble_rtabmap_cloud.py BAG_DIR poses_tum.txt out.pcd [voxel_m]
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

bag, poses_path, out = sys.argv[1:4]
voxel = float(sys.argv[4]) if len(sys.argv) > 4 else 0.1
poses = np.loadtxt(poses_path)
stamps = poses[:, 0]
DT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"), ("ring", "<u2"), ("t", "<f8")])


def rot(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n
    return np.array([[1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
                     [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
                     [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions("", ""))
r.set_filter(rosbag2_py.StorageFilter(topics=["/vanjee_points719e_merged"]))
chunks, used = [], set()
while r.has_next():
    _, data, _ = r.read_next()
    m = deserialize_message(data, PointCloud2)
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    i = int(np.argmin(np.abs(stamps - t)))
    if abs(stamps[i] - t) > 0.03 or i in used:
        continue
    used.add(i)
    pts = np.frombuffer(bytes(m.data), dtype=DT, count=m.width)
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(np.float64)
    xyz = xyz[np.isfinite(xyz).all(1)]
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

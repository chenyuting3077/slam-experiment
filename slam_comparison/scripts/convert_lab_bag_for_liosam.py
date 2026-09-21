#!/usr/bin/env python3
"""Rewrite the merged Vanjee cloud into LIO-SAM's Velodyne PointXYZIRT layout (ROS2 bag).

usage: convert_lab_bag_for_liosam.py SRC_BAG DST_BAG
- /vanjee_points719e_merged (x,y,z,intensity,ring u16,timestamp f64 absolute) ->
  /lidar_points: x,y,z,intensity f32, ring u16, time f32 (seconds since scan start; header stamp is moved to scan start); NaN points dropped.
- /imu/data copied unchanged.
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2, PointField

src, dst = sys.argv[1:3]
IN_DT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"), ("ring", "<u2"), ("t", "<f8")])
FIELDS = [("x", 0, PointField.FLOAT32), ("y", 4, PointField.FLOAT32), ("z", 8, PointField.FLOAT32),
          ("intensity", 12, PointField.FLOAT32), ("ring", 16, PointField.UINT16), ("time", 20, PointField.FLOAT32)]
OUT_DT = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "time"],
                   "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"],
                   "offsets": [0, 4, 8, 12, 16, 20], "itemsize": 24})

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=src), rosbag2_py.ConverterOptions("", ""))
reader.set_filter(rosbag2_py.StorageFilter(topics=["/vanjee_points719e_merged", "/imu/data"]))
writer = rosbag2_py.SequentialWriter()
writer.open(rosbag2_py.StorageOptions(uri=dst, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
writer.create_topic(rosbag2_py.TopicMetadata(id=0, name="/lidar_points", type="sensor_msgs/msg/PointCloud2",
                                             serialization_format="cdr"))
writer.create_topic(rosbag2_py.TopicMetadata(id=1, name="/imu/data", type="sensor_msgs/msg/Imu",
                                             serialization_format="cdr"))
cloud_cls = get_message("sensor_msgs/msg/PointCloud2")
n = 0
while reader.has_next():
    topic, data, stamp = reader.read_next()
    if topic == "/imu/data":
        writer.write(topic, data, stamp)
        continue
    m = deserialize_message(data, cloud_cls)
    pts = np.frombuffer(bytes(m.data), dtype=IN_DT, count=m.width)
    pts = pts[np.isfinite(pts["x"]) & np.isfinite(pts["y"]) & np.isfinite(pts["z"])]
    out = np.zeros(len(pts), dtype=OUT_DT)
    out["x"], out["y"], out["z"], out["intensity"] = pts["x"], pts["y"], pts["z"], pts["i"]
    out["ring"] = pts["ring"] - 1  # rings are 1..4 in the bag; LIO-SAM expects 0-based
    t0 = float(pts["t"].min())  # LIO-SAM wants header.stamp = scan start and per-point time >= 0
    out["time"] = (pts["t"] - t0).astype(np.float32)
    o = PointCloud2()
    o.header = m.header
    o.header.stamp.sec = int(t0)
    o.header.stamp.nanosec = int(round((t0 - int(t0)) * 1e9))
    o.height, o.width = 1, len(out)
    o.fields = [PointField(name=nm, offset=off, datatype=dt, count=1) for nm, off, dt in FIELDS]
    o.is_bigendian, o.point_step, o.row_step, o.is_dense = False, 24, 24 * len(out), True
    o.data = out.tobytes()
    writer.write("/lidar_points", serialize_message(o), stamp)
    n += 1
print(f"{n} clouds -> {dst}")

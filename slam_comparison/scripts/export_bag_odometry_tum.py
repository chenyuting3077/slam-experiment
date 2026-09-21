#!/usr/bin/env python3
"""Dump a nav_msgs/Odometry topic from a rosbag2 as a TUM trajectory.

usage: export_bag_odometry_tum.py BAG_DIR TOPIC out_tum.txt
"""
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, topic, out = sys.argv[1:4]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
cls = get_message('nav_msgs/msg/Odometry')
n = 0
with open(out, 'w') as f:
    while r.has_next():
        _, d, _ = r.read_next()
        m = deserialize_message(d, cls)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        f.write('%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n' % (t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        n += 1
print(f'{n} poses -> {out}')

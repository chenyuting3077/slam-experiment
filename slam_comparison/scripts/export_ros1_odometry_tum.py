#!/usr/bin/env python3
"""Dump nav_msgs/Odometry from a ROS1 bag (e.g. FAST_LIO's /Odometry) as a TUM trajectory.

usage: export_ros1_odometry_tum.py in.bag TOPIC out_tum.txt
"""
import sys

from rosbags.highlevel import AnyReader
from pathlib import Path

bag, topic, out = sys.argv[1:4]
n = 0
with AnyReader([Path(bag)]) as r, open(out, 'w') as f:
    for c, _, raw in r.messages(connections=[x for x in r.connections if x.topic == topic]):
        m = r.deserialize(raw, c.msgtype)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        f.write('%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n' % (t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        n += 1
print(f'{n} poses -> {out}')

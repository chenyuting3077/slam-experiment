#!/usr/bin/env python3
"""Subscribe to a nav_msgs/Odometry topic and append every message to a TUM file, flushing each line.

Unlike `ros2 bag record`, nothing is lost if the process/container is killed without a clean shutdown.

usage: dump_odometry_tum.py TOPIC OUT_TUM
"""
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

topic, out = sys.argv[1:3]
rclpy.init()
node = rclpy.create_node("dump_odometry_tum")
f = open(out, "w", buffering=1)


def cb(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    f.write("%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))


node.create_subscription(Odometry, topic, cb, QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT))
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

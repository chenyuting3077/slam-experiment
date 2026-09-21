#!/usr/bin/env python3
"""Livox Mid360 part of a Compal AMR lab mcap -> ROS1 bag for the official hku-mars/FAST_LIO (mid360.yaml).

usage: convert_lab_livox_ros1.py SRC.mcap DST.bag
 /livox/lidar: livox_ros_driver2/CustomMsg (ROS2) -> livox_ros_driver/CustomMsg (ROS1), same fields, frame livox_frame
 /livox/imu:   copied unchanged (still in g; FAST_LIO rescales acceleration by its own gravity estimate), frame livox_frame
"""
import struct
import sys
from pathlib import Path

import numpy as np
from mcap.reader import make_reader
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

sys.path.insert(0, str(Path(__file__).parent))
from livox_cdr import parse_custom_msg  # noqa: E402

CUSTOM_MSG_DEF = "Header header\nuint64 timebase\nuint32 point_num\nuint8 lidar_id\nuint8[3] rsvd\nCustomPoint[] points\n"
CUSTOM_POINT_DEF = "uint32 offset_time\nfloat32 x\nfloat32 y\nfloat32 z\nuint8 reflectivity\nuint8 tag\nuint8 line\n"
P1_DT = np.dtype([('offset_time', '<u4'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                  ('reflectivity', 'u1'), ('tag', 'u1'), ('line', 'u1')])
FRAME = b"livox_frame"

src, dst = sys.argv[1:3]
ts2 = get_typestore(Stores.ROS2_HUMBLE)
ts1 = get_typestore(Stores.ROS1_NOETIC)
ts1.register(get_types_from_msg(CUSTOM_MSG_DEF, 'livox_ros_driver/msg/CustomMsg'))
ts1.register(get_types_from_msg(CUSTOM_POINT_DEF, 'livox_ros_driver/msg/CustomPoint'))
Imu1, Header1, Time1 = ts1.types['sensor_msgs/msg/Imu'], ts1.types['std_msgs/msg/Header'], ts1.types['builtin_interfaces/msg/Time']

n_l = n_i = 0
with open(src, 'rb') as f, Writer(dst) as w:
    c_l = w.add_connection('/livox/lidar', 'livox_ros_driver/msg/CustomMsg', typestore=ts1)
    c_i = w.add_connection('/livox/imu', 'sensor_msgs/msg/Imu', typestore=ts1)
    for schema, channel, m in make_reader(f).iter_messages(topics=['/livox/lidar', '/livox/imu']):
        if channel.topic == '/livox/lidar':
            sec, nsec, _, timebase, lidar_id, pts = parse_custom_msg(m.data)
            p1 = np.empty(len(pts), dtype=P1_DT)
            for k in P1_DT.names:
                p1[k] = pts[k]
            raw = (struct.pack('<III', 0, sec, nsec) + struct.pack('<I', len(FRAME)) + FRAME
                   + struct.pack('<QIB3x', timebase, len(pts), lidar_id) + struct.pack('<I', len(pts)) + p1.tobytes())
            w.write(c_l, sec * 10**9 + nsec, raw)
            n_l += 1
        else:
            i2 = ts2.deserialize_cdr(m.data, 'sensor_msgs/msg/Imu')
            i1 = Imu1(header=Header1(seq=0, stamp=Time1(sec=i2.header.stamp.sec, nanosec=i2.header.stamp.nanosec), frame_id='livox_frame'),
                      orientation=i2.orientation, orientation_covariance=np.array(i2.orientation_covariance, dtype=np.float64),
                      angular_velocity=i2.angular_velocity, angular_velocity_covariance=np.array(i2.angular_velocity_covariance, dtype=np.float64),
                      linear_acceleration=i2.linear_acceleration, linear_acceleration_covariance=np.array(i2.linear_acceleration_covariance, dtype=np.float64))
            w.write(c_i, i2.header.stamp.sec * 10**9 + i2.header.stamp.nanosec, ts1.serialize_ros1(i1, 'sensor_msgs/msg/Imu'))
            n_i += 1
print(f'{n_l} lidar msgs, {n_i} imu msgs -> {dst}')

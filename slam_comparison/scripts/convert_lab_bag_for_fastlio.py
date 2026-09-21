#!/usr/bin/env python3
"""Convert a ROS2/mcap Compal AMR bag into a ROS1 bag for official FAST_LIO.

usage: convert_lab_bag_for_fastlio.py SRC.mcap DST.bag

- /vanjee_points719e_merged -> /velodyne_points, repacked into x,y,z,intensity,
  time(f32 seconds relative to the header stamp),ring(u16) so FAST_LIO's
  velodyne_handler uses per-point offset times. NaN points are dropped.
- /imu/data -> /imu/data, passthrough.
"""
import sys

import numpy as np
from mcap_ros2.reader import read_ros2_messages
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

SRC, DST = sys.argv[1], sys.argv[2]

ts = get_typestore(Stores.ROS1_NOETIC)
PointCloud2 = ts.types["sensor_msgs/msg/PointCloud2"]
PointField = ts.types["sensor_msgs/msg/PointField"]
Imu = ts.types["sensor_msgs/msg/Imu"]
Header = ts.types["std_msgs/msg/Header"]
Time = ts.types["builtin_interfaces/msg/Time"]
Vector3 = ts.types["geometry_msgs/msg/Vector3"]
Quaternion = ts.types["geometry_msgs/msg/Quaternion"]

IN_DT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"),
                  ("ring", "<u2"), ("t", "<f8")])
OUT_DT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"),
                   ("time", "<f4"), ("ring", "<u2")])
assert IN_DT.itemsize == 26 and OUT_DT.itemsize == 22

FIELDS_OUT = [
    PointField(name="x", offset=0, datatype=7, count=1),
    PointField(name="y", offset=4, datatype=7, count=1),
    PointField(name="z", offset=8, datatype=7, count=1),
    PointField(name="intensity", offset=12, datatype=7, count=1),
    PointField(name="time", offset=16, datatype=7, count=1),
    PointField(name="ring", offset=20, datatype=4, count=1),
]


def mk_header(sec, nsec, frame_id):
    return Header(seq=0, stamp=Time(sec=sec, nanosec=nsec), frame_id=frame_id)


def to_ns(sec, nsec):
    return sec * 1_000_000_000 + nsec


n_cloud = n_imu = n_in = n_out = 0

with Writer(DST) as writer:
    conn_cloud = writer.add_connection("/velodyne_points", PointCloud2.__msgtype__, typestore=ts)
    conn_imu = writer.add_connection("/imu/data", Imu.__msgtype__, typestore=ts)

    for msg in read_ros2_messages(SRC, topics=["/vanjee_points719e_merged", "/imu/data"]):
        m = msg.ros_msg
        sec, nsec = m.header.stamp.sec, m.header.stamp.nanosec
        if msg.channel.topic == "/vanjee_points719e_merged":
            assert m.point_step == 26
            pts = np.frombuffer(bytes(m.data), dtype=IN_DT, count=m.width)
            n_in += len(pts)
            ok = np.isfinite(pts["x"]) & np.isfinite(pts["y"]) & np.isfinite(pts["z"])
            pts = pts[ok]
            out = np.empty(len(pts), dtype=OUT_DT)
            out["x"], out["y"], out["z"], out["i"] = pts["x"], pts["y"], pts["z"], pts["i"]
            out["time"] = (pts["t"] - (sec + nsec * 1e-9)).astype(np.float32)
            out["ring"] = pts["ring"]
            n_out += len(out)
            cloud = PointCloud2(
                header=mk_header(sec, nsec, "base_link"), height=1, width=len(out),
                fields=FIELDS_OUT, is_bigendian=False, point_step=22,
                row_step=22 * len(out), data=np.frombuffer(out.tobytes(), dtype=np.uint8),
                is_dense=True)
            writer.write(conn_cloud, to_ns(sec, nsec), ts.serialize_ros1(cloud, PointCloud2.__msgtype__))
            n_cloud += 1
        else:
            imu = Imu(
                header=mk_header(sec, nsec, "imu_link"),
                orientation=Quaternion(x=m.orientation.x, y=m.orientation.y, z=m.orientation.z, w=m.orientation.w),
                orientation_covariance=np.array(m.orientation_covariance, dtype=np.float64),
                angular_velocity=Vector3(x=m.angular_velocity.x, y=m.angular_velocity.y, z=m.angular_velocity.z),
                angular_velocity_covariance=np.array(m.angular_velocity_covariance, dtype=np.float64),
                linear_acceleration=Vector3(x=m.linear_acceleration.x, y=m.linear_acceleration.y, z=m.linear_acceleration.z),
                linear_acceleration_covariance=np.array(m.linear_acceleration_covariance, dtype=np.float64))
            writer.write(conn_imu, to_ns(sec, nsec), ts.serialize_ros1(imu, Imu.__msgtype__))
            n_imu += 1

print(f"wrote {DST}: {n_cloud} clouds ({n_out}/{n_in} points kept), {n_imu} imu msgs")

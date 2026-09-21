#!/usr/bin/env python3
"""Livox Mid360 part of a Compal AMR lab bag -> ROS2 bag for Cartographer / RTAB-Map / LIO-SAM.

usage: convert_lab_livox_ros2.py SRC_BAG DST_BAG [--imu-orientation-from-accel]
 /livox/lidar (livox_ros_driver2/CustomMsg) -> /livox/points (PointCloud2, Velodyne PointXYZIRT layout:
     x y z intensity f32, ring u16 = Livox `line`, time f32 = offset_time in seconds), frame livox_frame.
     Points are filtered like official FAST_LIO's Livox handler: tag & 0x30 in (0x00, 0x10), finite, non-zero,
     and sorted by time (Cartographer CHECKs that no point is later than the last point of the cloud).
 /livox/imu  -> /livox/imu, linear acceleration converted from g to m/s^2 (the Livox IMU reports g), frame livox_frame.
 /odometry, /tf_static copied; one extra static transform livox_link -> livox_frame (identity) is added because the
     cloud is stamped livox_link but the IMU livox_frame (same physical sensor; the bag has no TF for livox_frame).
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).parent))
from livox_cdr import parse_custom_msg  # noqa: E402

G = 9.80665
OUT_DT = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "time"],
                   "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"],
                   "offsets": [0, 4, 8, 12, 16, 20], "itemsize": 24})
FIELDS = [("x", 0, PointField.FLOAT32), ("y", 4, PointField.FLOAT32), ("z", 8, PointField.FLOAT32),
          ("intensity", 12, PointField.FLOAT32), ("ring", 16, PointField.UINT16), ("time", 20, PointField.FLOAT32)]


def quat_from_accel(ax, ay, az):
    roll = np.arctan2(ay, az)
    pitch = np.arctan2(-ax, np.hypot(ay, az))
    cr, sr, cp, sp = np.cos(roll / 2), np.sin(roll / 2), np.cos(pitch / 2), np.sin(pitch / 2)
    return sr * cp, cr * sp, -sr * sp, cr * cp  # yaw = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--imu-orientation-from-accel", action="store_true",
                    help="fill Imu.orientation with a gravity-derived roll/pitch (yaw 0); off by default")
    args = ap.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.src), rosbag2_py.ConverterOptions("", ""))
    topics = {t.name: t for t in reader.get_all_topics_and_types()}
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.dst, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    writer.create_topic(rosbag2_py.TopicMetadata(id=0, name="/livox/points", type="sensor_msgs/msg/PointCloud2",
                                                 serialization_format="cdr"))
    for name in ("/livox/imu", "/odometry", "/tf_static"):
        writer.create_topic(topics[name])

    n_cloud = n_imu = 0
    tf_written = False
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == "/livox/lidar":
            sec, nsec, _, _, _, pts = parse_custom_msg(data)
            ok = ((pts["tag"] & 0x30) == 0) | ((pts["tag"] & 0x30) == 0x10)
            ok &= np.isfinite(pts["x"]) & np.isfinite(pts["y"]) & np.isfinite(pts["z"])
            ok &= (pts["x"] != 0) | (pts["y"] != 0) | (pts["z"] != 0)
            pts = pts[ok]
            pts = pts[np.argsort(pts["offset_time"], kind="stable")]  # Cartographer wants the last point to carry the max time
            out = np.zeros(len(pts), dtype=OUT_DT)
            out["x"], out["y"], out["z"] = pts["x"], pts["y"], pts["z"]
            out["intensity"] = pts["reflectivity"]
            out["ring"] = pts["line"]
            out["time"] = pts["offset_time"] * 1e-9
            o = PointCloud2()
            o.header.stamp.sec, o.header.stamp.nanosec, o.header.frame_id = sec, nsec, "livox_frame"
            o.height, o.width = 1, len(out)
            o.fields = [PointField(name=nm, offset=off, datatype=dt, count=1) for nm, off, dt in FIELDS]
            o.is_bigendian, o.point_step, o.row_step, o.is_dense = False, 24, 24 * len(out), True
            o.data = out.tobytes()
            writer.write("/livox/points", serialize_message(o), stamp)
            n_cloud += 1
        elif topic == "/livox/imu":
            m = deserialize_message(data, Imu)
            ax, ay, az = m.linear_acceleration.x * G, m.linear_acceleration.y * G, m.linear_acceleration.z * G
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = ax, ay, az
            if args.imu_orientation_from_accel:
                qx, qy, qz, qw = quat_from_accel(ax, ay, az)
                m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = qx, qy, qz, qw
            writer.write(topic, serialize_message(m), stamp)
            n_imu += 1
        else:
            if topic == "/tf_static" and not tf_written:
                t = TransformStamped()
                t.header.frame_id, t.child_frame_id = "livox_link", "livox_frame"
                t.transform.rotation.w = 1.0
                writer.write("/tf_static", serialize_message(TFMessage(transforms=[t])), stamp)
                tf_written = True
            writer.write(topic, data, stamp)
    print(f"{n_cloud} clouds, {n_imu} imu msgs -> {args.dst}")


if __name__ == "__main__":
    main()

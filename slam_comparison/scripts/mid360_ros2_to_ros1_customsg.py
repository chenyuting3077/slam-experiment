#!/usr/bin/env python3
"""Convert the merged Mid360 ROS2 bag's /livox/points (PointCloud2) into
genuine livox_ros_driver/CustomMsg messages, and copy /livox/imu through
unchanged, writing a ROS1 bag for the official hku-mars/FAST_LIO container
(its mid360.yaml expects real CustomMsg on lidar_type=1, not PointCloud2).

Field mapping per point (matches livox_ros_driver2's own PointCloud2
converter, which is exactly where our x/y/z/t/intensity/tag/line fields
came from in the first place):
  PointCloud2 field -> CustomPoint field
  x, y, z            -> x, y, z
  t (uint32, ns)      -> offset_time (uint32, ns) -- direct copy, same units
  intensity (float, already 0-255) -> reflectivity (uint8)
  tag (uint8)         -> tag
  line (uint8)        -> line
timebase per message = header.stamp as nanoseconds (matches livox_ros_driver2
convention: CustomMsg.header.stamp is the first point's time, timebase is
also that same instant, and each point's offset_time is already relative to
it).
"""
import argparse
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Writer as Ros1Writer
from rosbags.rosbag2 import Reader as Ros2Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

CUSTOM_MSG_DEF = """
Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
"""

CUSTOM_POINT_DEF = """
uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_in', help='merged ROS2 bag dir (with /livox/points + /livox/imu)')
    ap.add_argument('--out', required=True, help='output ROS1 .bag path')
    args = ap.parse_args()

    ts2 = get_typestore(Stores.ROS2_HUMBLE)
    ts1 = get_typestore(Stores.ROS1_NOETIC)
    ts1.register(get_types_from_msg(CUSTOM_MSG_DEF, 'livox_ros_driver/msg/CustomMsg'))
    ts1.register(get_types_from_msg(CUSTOM_POINT_DEF, 'livox_ros_driver/msg/CustomPoint'))

    out_path = Path(args.out)
    if out_path.exists():
        raise SystemExit(f"{out_path} already exists -- remove it first")

    Header1 = ts1.types['std_msgs/msg/Header']
    Time1 = ts1.types['builtin_interfaces/msg/Time']

    def ros1_header(ros2_header):
        return Header1(
            seq=0,
            stamp=Time1(sec=ros2_header.stamp.sec, nanosec=ros2_header.stamp.nanosec),
            frame_id=ros2_header.frame_id,
        )

    n_lidar, n_imu = 0, 0
    with Ros2Reader(Path(args.bag_in)) as reader2, Ros1Writer(out_path) as writer1:
        conns2 = {c.topic: c for c in reader2.connections}
        conn_lidar = writer1.add_connection('/livox/lidar', 'livox_ros_driver/msg/CustomMsg', typestore=ts1)
        conn_imu = writer1.add_connection('/livox/imu', 'sensor_msgs/msg/Imu', typestore=ts1)

        want = [c for c in reader2.connections if c.topic in ('/livox/points', '/livox/imu')]
        for conn, timestamp, rawdata in reader2.messages(connections=want):
            if conn.topic == '/livox/imu':
                msg = ts2.deserialize_cdr(rawdata, conn.msgtype)
                imu1 = ts1.types['sensor_msgs/msg/Imu'](
                    header=ros1_header(msg.header),
                    orientation=msg.orientation,
                    orientation_covariance=msg.orientation_covariance,
                    angular_velocity=msg.angular_velocity,
                    angular_velocity_covariance=msg.angular_velocity_covariance,
                    linear_acceleration=msg.linear_acceleration,
                    linear_acceleration_covariance=msg.linear_acceleration_covariance,
                )
                raw1 = ts1.serialize_ros1(imu1, 'sensor_msgs/msg/Imu')
                writer1.write(conn_imu, timestamp, raw1)
                n_imu += 1
                continue

            msg = ts2.deserialize_cdr(rawdata, conn.msgtype)
            data = np.frombuffer(msg.data, dtype=np.uint8)
            pts = data.reshape(-1, msg.point_step)
            x = np.frombuffer(pts[:, 0:4].tobytes(), dtype=np.float32)
            y = np.frombuffer(pts[:, 4:8].tobytes(), dtype=np.float32)
            z = np.frombuffer(pts[:, 8:12].tobytes(), dtype=np.float32)
            t = np.frombuffer(pts[:, 12:16].tobytes(), dtype=np.uint32)
            intensity = np.frombuffer(pts[:, 16:20].tobytes(), dtype=np.float32)
            tag = pts[:, 20]
            line = pts[:, 21]
            reflectivity = np.clip(intensity, 0, 255).astype(np.uint8)

            points = [
                ts1.types['livox_ros_driver/msg/CustomPoint'](
                    offset_time=int(t[i]), x=float(x[i]), y=float(y[i]), z=float(z[i]),
                    reflectivity=int(reflectivity[i]), tag=int(tag[i]), line=int(line[i]),
                )
                for i in range(len(x))
            ]
            custom = ts1.types['livox_ros_driver/msg/CustomMsg'](
                header=ros1_header(msg.header),
                timebase=timestamp,
                point_num=len(points),
                lidar_id=0,
                rsvd=np.array([0, 0, 0], dtype=np.uint8),
                points=points,
            )
            raw1 = ts1.serialize_ros1(custom, 'livox_ros_driver/msg/CustomMsg')
            writer1.write(conn_lidar, timestamp, raw1)
            n_lidar += 1

    print(f"Wrote {n_lidar} CustomMsg + {n_imu} Imu messages to {out_path}")


if __name__ == '__main__':
    main()

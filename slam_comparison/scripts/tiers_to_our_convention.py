#!/usr/bin/env python3
"""Convert a TIERS multi_modal_lidar_dataset ROS1 bag's Mid360 topics into
the same convention already used for the Zenodo Mid360 datasets in this
repo, so all existing configs/launch files/patches work unchanged:
  - topics renamed: /mid360/livox/lidar -> /livox/points, /mid360/livox/imu -> /livox/imu
  - point field layout: x,y,z,t(uint32 ns offset-from-scan-start),intensity,tag,line
    (TIERS ships x,y,z,intensity,tag,line,timestamp(float64 absolute ns) --
    different order AND semantics for the time field)
  - frame_id unified to "livox_frame" for both topics (TIERS uses
    "mid360_frame" for the cloud but "livox_frame" for the IMU)
"""
import argparse
import struct
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import Stores, get_typestore


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_in')
    ap.add_argument('--out', required=True)
    ap.add_argument('--keep-topics', nargs='*', default=['/gnss_pose', '/vrpn_client_node/unitree_b1/pose'],
                     help='non-Mid360 topics to pass through unchanged (e.g. ground truth)')
    args = ap.parse_args()

    ts = get_typestore(Stores.ROS1_NOETIC)
    out_path = Path(args.out)
    if out_path.exists():
        raise SystemExit(f"{out_path} already exists -- remove it first")

    PointCloud2 = ts.types['sensor_msgs/msg/PointCloud2']
    PointField = ts.types['sensor_msgs/msg/PointField']
    Header = ts.types['std_msgs/msg/Header']

    n_lidar, n_imu, n_other = 0, 0, 0
    with Reader(Path(args.bag_in)) as reader, Writer(out_path) as writer:
        conn_map = {}
        for conn in reader.connections:
            if conn.topic not in args.keep_topics:
                continue
            conn_map[conn.topic] = writer.add_connection(conn.topic, conn.msgtype, typestore=ts)

        conn_lidar = writer.add_connection('/livox/points', 'sensor_msgs/msg/PointCloud2', typestore=ts)
        conn_imu = writer.add_connection('/livox/imu', 'sensor_msgs/msg/Imu', typestore=ts)

        for conn, timestamp, rawdata in reader.messages():
            if conn.topic == '/mid360/livox/imu':
                msg = ts.deserialize_ros1(rawdata, conn.msgtype)
                msg = ts.types['sensor_msgs/msg/Imu'](
                    header=Header(seq=0, stamp=msg.header.stamp, frame_id='livox_frame'),
                    orientation=msg.orientation,
                    orientation_covariance=msg.orientation_covariance,
                    angular_velocity=msg.angular_velocity,
                    angular_velocity_covariance=msg.angular_velocity_covariance,
                    linear_acceleration=msg.linear_acceleration,
                    linear_acceleration_covariance=msg.linear_acceleration_covariance,
                )
                writer.write(conn_imu, timestamp, ts.serialize_ros1(msg, 'sensor_msgs/msg/Imu'))
                n_imu += 1
                continue

            if conn.topic == '/mid360/livox/lidar':
                msg = ts.deserialize_ros1(rawdata, conn.msgtype)
                data = np.frombuffer(msg.data, dtype=np.uint8)
                pts = data.reshape(-1, msg.point_step)
                x = np.frombuffer(pts[:, 0:4].tobytes(), dtype=np.float32)
                y = np.frombuffer(pts[:, 4:8].tobytes(), dtype=np.float32)
                z = np.frombuffer(pts[:, 8:12].tobytes(), dtype=np.float32)
                intensity = np.frombuffer(pts[:, 12:16].tobytes(), dtype=np.float32)
                tag = pts[:, 16]
                line = pts[:, 17]
                abs_ns = np.frombuffer(pts[:, 18:26].tobytes(), dtype=np.float64)
                stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                offset_ns = np.clip((abs_ns * 1e-9 - stamp_s) * 1e9, 0, None).astype(np.uint32)

                n = len(x)
                out = np.zeros((n, 22), dtype=np.uint8)
                out[:, 0:4] = np.frombuffer(x.tobytes(), dtype=np.uint8).reshape(n, 4)
                out[:, 4:8] = np.frombuffer(y.tobytes(), dtype=np.uint8).reshape(n, 4)
                out[:, 8:12] = np.frombuffer(z.tobytes(), dtype=np.uint8).reshape(n, 4)
                out[:, 12:16] = np.frombuffer(offset_ns.tobytes(), dtype=np.uint8).reshape(n, 4)
                out[:, 16:20] = np.frombuffer(intensity.tobytes(), dtype=np.uint8).reshape(n, 4)
                out[:, 20] = tag
                out[:, 21] = line

                fields = [
                    PointField(name='x', offset=0, datatype=7, count=1),
                    PointField(name='y', offset=4, datatype=7, count=1),
                    PointField(name='z', offset=8, datatype=7, count=1),
                    PointField(name='t', offset=12, datatype=6, count=1),
                    PointField(name='intensity', offset=16, datatype=7, count=1),
                    PointField(name='tag', offset=20, datatype=2, count=1),
                    PointField(name='line', offset=21, datatype=2, count=1),
                ]
                new_msg = PointCloud2(
                    header=Header(seq=0, stamp=msg.header.stamp, frame_id='livox_frame'),
                    height=1, width=n, fields=fields, is_bigendian=False,
                    point_step=22, row_step=22 * n, data=out.reshape(-1), is_dense=True,
                )
                writer.write(conn_lidar, timestamp, ts.serialize_ros1(new_msg, 'sensor_msgs/msg/PointCloud2'))
                n_lidar += 1
                continue

            if conn.topic in conn_map:
                writer.write(conn_map[conn.topic], timestamp, rawdata)
                n_other += 1

    print(f"Wrote {n_lidar} lidar + {n_imu} imu + {n_other} other messages to {out_path}")


if __name__ == '__main__':
    main()

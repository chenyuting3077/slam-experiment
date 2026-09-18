#!/usr/bin/env python3
"""Rewrite a converted ROS2 bag, fixing PointCloud2.row_step == 0 messages.

The LIO-SAM sample ROS1 bags publish PointCloud2 with row_step left at 0
(a quirk of the original Velodyne driver). rosbags-convert carries that 0
through unchanged. Cartographer doesn't validate row_step, but RTAB-Map's
icp_odometry asserts data.size() == row_step * height and aborts. Since
data.size() == width * point_step here, row_step is just recomputed.
"""
import sys
from pathlib import Path

from rosbags.rosbag2 import Writer
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def main(src: str, dst: str, version: int = 9) -> None:
    src_path = Path(src)
    dst_path = Path(dst)
    typestore = get_typestore(Stores.ROS2_JAZZY)

    with AnyReader([src_path]) as reader, Writer(dst_path, version=version) as writer:
        conn_map = {}
        for conn in reader.connections:
            conn_map[conn.id] = writer.add_connection(
                conn.topic, conn.msgtype, typestore=typestore)

        fixed_count = 0
        total = 0
        for conn, timestamp, rawdata in reader.messages():
            total += 1
            out_conn = conn_map[conn.id]
            if conn.msgtype == 'sensor_msgs/msg/PointCloud2':
                msg = reader.deserialize(rawdata, conn.msgtype)
                expected_row_step = msg.width * msg.point_step
                if msg.row_step != expected_row_step:
                    msg.row_step = expected_row_step
                    fixed_count += 1
                    rawdata = typestore.serialize_cdr(msg, conn.msgtype)
            writer.write(out_conn, timestamp, rawdata)

        print(f'Total messages: {total}, PointCloud2 row_step fixed: {fixed_count}')


if __name__ == '__main__':
    version = int(sys.argv[3]) if len(sys.argv) > 3 else 9
    main(sys.argv[1], sys.argv[2], version)

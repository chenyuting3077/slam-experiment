#!/usr/bin/env python3
"""Rewrite a bag renaming /points_raw -> /points2 and /imu_correct -> /imu.

cartographer_offline_node's ROS2 port doesn't honor -r remap args when
matching its hardcoded expected sensor topics ("points2", "imu") against the
bag's actual topic names (confirmed via its own "Expected resolved topic
... not found in bag file(s)" diagnostic, and an empty resulting pbstream --
0 submaps, 0 constraints). Renaming the topics in the bag itself sidesteps
the broken remap-resolution path entirely.
"""
import sys
from pathlib import Path

from rosbags.rosbag2 import Writer
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

RENAMES = {
    '/points_raw': '/points2',
    '/imu_correct': '/imu',
}


def main(src: str, dst: str) -> None:
    src_path = Path(src)
    dst_path = Path(dst)
    typestore = get_typestore(Stores.ROS2_JAZZY)

    with AnyReader([src_path]) as reader, Writer(dst_path, version=9) as writer:
        conn_map = {}
        for conn in reader.connections:
            new_topic = RENAMES.get(conn.topic, conn.topic)
            conn_map[conn.id] = writer.add_connection(
                new_topic, conn.msgtype, typestore=typestore)

        total = 0
        for conn, timestamp, rawdata in reader.messages():
            total += 1
            writer.write(conn_map[conn.id], timestamp, rawdata)

        print(f'Total messages copied: {total}')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])

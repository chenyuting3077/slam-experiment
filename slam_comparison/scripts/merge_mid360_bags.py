#!/usr/bin/env python3
"""Concatenate the two consecutive rosbag2 parts of the Zenodo Mid360
"outdoor_hard_01" sequence into one continuous bag. Copies raw serialized
bytes without deserializing.

Only copies /livox/points (PointCloud2) and /livox/imu -- the three ROS2
consumers here (Cartographer, RTAB-Map, the LIO-SAM Mid360 fork) all use
the plain PointCloud2 topic, not the raw livox_ros_driver2/CustomMsg one,
which would need that third-party message type registered to write out
(and we don't need it for anything in this bag).
"""
import argparse
from pathlib import Path

from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys import Stores, get_typestore


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('parts', nargs='+', help='bag directories, in time order')
    ap.add_argument('--out', required=True)
    ap.add_argument('--topics', nargs='+', default=['/livox/points', '/livox/imu'])
    args = ap.parse_args()

    ts = get_typestore(Stores.ROS2_HUMBLE)
    out_path = Path(args.out)
    if out_path.exists():
        raise SystemExit(f"{out_path} already exists -- remove it first")

    with Writer(out_path, version=5) as writer:
        conn_map = {}
        total = 0
        for part in args.parts:
            with Reader(Path(part)) as reader:
                wanted = [c for c in reader.connections if c.topic in args.topics]
                for conn in wanted:
                    key = (conn.topic, conn.msgtype)
                    if key not in conn_map:
                        conn_map[key] = writer.add_connection(
                            conn.topic, conn.msgtype, typestore=ts,
                            serialization_format=conn.ext.serialization_format,
                            offered_qos_profiles=conn.ext.offered_qos_profiles or (),
                        )
                for conn, timestamp, rawdata in reader.messages(connections=wanted):
                    writer.write(conn_map[(conn.topic, conn.msgtype)], timestamp, rawdata)
                    total += 1
            print(f"[{part}] merged.")
        print(f"Wrote {total} messages to {out_path}")


if __name__ == '__main__':
    main()

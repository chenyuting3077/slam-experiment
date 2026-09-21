#!/usr/bin/env python3
"""Export optimized node poses from a Cartographer .pbstream as a TUM file.

usage: export_cartographer_pbstream_trajectory.py PBPY_DIR in.pbstream out_tum.txt
PBPY_DIR: dir holding python protobuf modules compiled from cartographer's .proto files.
"""
import gzip
import struct
import sys

sys.path.insert(0, sys.argv[1])
from cartographer.mapping.proto import serialization_pb2  # noqa: E402

UNIX_EPOCH_TICKS = 621355968000000000  # cartographer time = 100ns ticks since 0001-01-01


def records(path):
    with open(path, 'rb') as f:
        f.read(8)  # magic number
        while True:
            hdr = f.read(8)
            if len(hdr) < 8:
                return
            size = struct.unpack('<Q', hdr)[0]
            yield gzip.decompress(f.read(size))


def main():
    rows = []
    for i, raw in enumerate(records(sys.argv[2])):
        if i == 0:
            continue  # SerializationHeader
        msg = serialization_pb2.SerializedData()
        msg.ParseFromString(raw)
        if msg.HasField('pose_graph'):
            for traj in msg.pose_graph.trajectory:
                for n in traj.node:
                    t = (n.timestamp - UNIX_EPOCH_TICKS) / 1e7
                    p, q = n.pose.translation, n.pose.rotation
                    rows.append((traj.trajectory_id, t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    rows.sort(key=lambda r: (r[0], r[1]))
    with open(sys.argv[3], 'w') as out:
        for r in rows:
            out.write('%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n' % r[1:])
    print(f'{len(rows)} poses -> {sys.argv[3]}')


if __name__ == '__main__':
    main()

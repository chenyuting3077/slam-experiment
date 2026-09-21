#!/usr/bin/env python3
"""Copy the first N seconds of selected topics from a rosbag2 into a new bag."""
import argparse

import rosbag2_py


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--seconds', type=float, required=True)
    ap.add_argument('--topics', nargs='+', required=True)
    ap.add_argument('--storage', default='mcap')
    args = ap.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.src), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=args.topics))

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.dst, storage_id=args.storage),
                rosbag2_py.ConverterOptions('', ''))
    for t in reader.get_all_topics_and_types():
        if t.name in args.topics:
            writer.create_topic(t)

    start = None
    counts = {}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if start is None:
            start = stamp
        if (stamp - start) / 1e9 > args.seconds:
            break
        writer.write(topic, data, stamp)
        counts[topic] = counts.get(topic, 0) + 1
    for k, v in sorted(counts.items()):
        print(f'{k}: {v}')


if __name__ == '__main__':
    main()

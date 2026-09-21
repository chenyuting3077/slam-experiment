#!/usr/bin/env python3
"""Merge rosbag2 bags into one, interleaved by timestamp (Cartographer's offline node treats several bags as several trajectories).

usage: merge_bags.py OUT_BAG IN_BAG [IN_BAG ...]   (topic names must be distinct across inputs, except /tf_static which is taken from the first)
"""
import heapq
import sys

import rosbag2_py


def open_reader(uri):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri), rosbag2_py.ConverterOptions("", ""))
    return r


def stream(idx, reader, skip):
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in skip:
            yield stamp, idx, topic, data


out, ins = sys.argv[1], sys.argv[2:]
readers = [open_reader(u) for u in ins]
writer = rosbag2_py.SequentialWriter()
writer.open(rosbag2_py.StorageOptions(uri=out, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
seen = set()
skips = []
for r in readers:
    skip = set()
    for t in r.get_all_topics_and_types():
        if t.name in seen:
            skip.add(t.name)
        else:
            seen.add(t.name)
            writer.create_topic(t)
    skips.append(skip)
n = 0
for stamp, _, topic, data in heapq.merge(*[stream(i, r, skips[i]) for i, r in enumerate(readers)]):
    writer.write(topic, data, stamp)
    n += 1
print(f"{n} messages -> {out}")

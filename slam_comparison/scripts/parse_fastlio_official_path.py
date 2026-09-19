#!/usr/bin/env python3
"""Parse a `rostopic echo /path -n 1` capture (nav_msgs/Path, YAML-style
rostopic echo output) from the official hku-mars/FAST_LIO into a TUM-format
trajectory file. /path republishes its full accumulated history on every
message, so one capture near the end of a run gives the whole trajectory --
FAST_LIO has no rosbag-record/save-trajectory mechanism of its own.
"""
import re
import sys


def main(path_capture: str, out_path: str) -> None:
    with open(path_capture) as f:
        content = f.read()

    blocks = content.split('  - \n')[1:]
    out = []
    for b in blocks:
        secs = re.search(r'secs:\s*(\d+)', b)
        nsecs = re.search(r'nsecs:\s*(\d+)', b)
        pos = re.search(
            r'position:\s*\n\s*x:\s*(-?[\d.eE+-]+)\s*\n\s*y:\s*(-?[\d.eE+-]+)\s*\n\s*z:\s*(-?[\d.eE+-]+)', b)
        ori = re.search(
            r'orientation:\s*\n\s*x:\s*(-?[\d.eE+-]+)\s*\n\s*y:\s*(-?[\d.eE+-]+)\s*\n\s*z:\s*(-?[\d.eE+-]+)\s*\n\s*w:\s*(-?[\d.eE+-]+)', b)
        if not (secs and nsecs and pos and ori):
            continue
        t = int(secs.group(1)) + int(nsecs.group(1)) / 1e9
        px, py, pz = pos.groups()
        qx, qy, qz, qw = ori.groups()
        out.append(f"{t:.6f} {px} {py} {pz} {qx} {qy} {qz} {qw}")

    with open(out_path, 'w') as f:
        f.write('\n'.join(out) + '\n')
    print(f"Wrote {len(out)} poses to {out_path}")


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])

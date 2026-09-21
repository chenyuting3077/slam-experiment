"""Parse a ROS2 CDR-serialized livox_ros_driver2/msg/CustomMsg without building per-point Python objects."""
import struct

import numpy as np

# offset_time u32, x/y/z f32, reflectivity/tag/line u8, 1 byte of CDR padding before the next element
POINT_DT = np.dtype([('offset_time', '<u4'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                     ('reflectivity', 'u1'), ('tag', 'u1'), ('line', 'u1'), ('pad', 'u1')])


def _align(pos, n):
    return (pos + n - 1) // n * n


def parse_custom_msg(raw):
    """Return (sec, nanosec, frame_id, timebase_ns, lidar_id, points structured array)."""
    b = memoryview(raw)[4:]  # drop the 4-byte encapsulation header; CDR alignment is relative to what follows
    sec, nsec, slen = struct.unpack_from('<iII', b, 0)
    frame_id = bytes(b[12:12 + slen - 1]).decode()
    pos = _align(12 + slen, 8)
    timebase, point_num, lidar_id = struct.unpack_from('<QIB', b, pos)
    pos += 8 + 4 + 1 + 3  # timebase, point_num, lidar_id, rsvd[3]
    pos = _align(pos, 4)
    (n,) = struct.unpack_from('<I', b, pos)
    pos += 4
    if n != point_num:
        raise ValueError(f'point_num {point_num} != sequence length {n}')
    need = n * POINT_DT.itemsize
    avail = len(b) - pos
    if avail not in (need, need - 1):
        raise ValueError(f'unexpected payload size: {avail} bytes for {n} points')
    buf = bytes(b[pos:pos + avail]) + (b'\0' if avail == need - 1 else b'')
    return sec, nsec, frame_id, timebase, lidar_id, np.frombuffer(buf, dtype=POINT_DT, count=n)

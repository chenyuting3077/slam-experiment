#!/usr/bin/env python3
"""Incrementally replay a system's trajectory + accumulated point cloud in
an Open3D window, so you can watch the map build up (and drift/misalign)
scan by scan instead of only seeing the final static result.

Works uniformly across all four systems -- they all just need a TUM-format
trajectory file. Point cloud content always comes from the same source
(/points_raw in the Jazzy-converted bag), so the only thing differing
between systems is the pose trajectory applied to it -- an apples-to-apples
comparison of "how each system's estimated poses place the same raw scans".

Usage:
  python3 incremental_playback_viewer.py <trajectory.txt> [--bag PATH]
      [--stride N] [--voxel SIZE] [--point-stride N] [--speed FPS]

Controls once the window opens:
  Space       pause / resume
  Right arrow step forward one scan while paused
  Q / Esc     quit
"""
import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np
import open3d as o3d
from rosbags.highlevel import AnyReader

DEFAULT_BAG = '/home/allen/slam-experiment/data/campus_dataset_ros2'


def load_tum(path):
    times, poses = [], []
    with open(path) as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            t = float(parts[0])
            x, y, z, qx, qy, qz, qw = (float(v) for v in parts[1:8])
            n = qx * qx + qy * qy + qz * qz + qw * qw
            s = 2.0 / n
            rot = np.array([
                [1 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw), s * (qx * qz + qy * qw)],
                [s * (qx * qy + qz * qw), 1 - s * (qx * qx + qz * qz), s * (qy * qz - qx * qw)],
                [s * (qx * qz - qy * qw), s * (qy * qz + qx * qw), 1 - s * (qx * qx + qy * qy)],
            ])
            transform = np.eye(4)
            transform[:3, :3] = rot
            transform[:3, 3] = [x, y, z]
            times.append(t)
            poses.append(transform)
    return np.array(times), poses


def load_scans(bag_path, point_stride):
    """Load every /points_raw scan's (timestamp, xyz) up front."""
    scans = []
    with AnyReader([Path(bag_path)]) as reader:
        conns = [c for c in reader.connections if c.topic == '/points_raw']
        for conn, timestamp, rawdata in reader.messages(connections=conns):
            msg = reader.deserialize(rawdata, conn.msgtype)
            data = np.frombuffer(msg.data, dtype=np.uint8)
            pts = data.reshape(-1, msg.point_step)
            xyz = np.frombuffer(pts[:, 0:12].tobytes(), dtype=np.float32).reshape(-1, 3)
            xyz = xyz[::point_stride]
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            scans.append((timestamp / 1e9, xyz))
    return scans


def nearest_pose(times, poses, t, max_dt=0.5):
    i = np.searchsorted(times, t)
    if i == 0 or i >= len(times):
        return None
    if abs(times[i] - t) > abs(times[i - 1] - t):
        i -= 1
    if abs(times[i] - t) > max_dt:
        return None
    return poses[i]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('trajectory')
    ap.add_argument('--bag', default=DEFAULT_BAG)
    ap.add_argument('--stride', type=int, default=1, help='use every Nth scan')
    ap.add_argument('--voxel', type=float, default=0.15, help='accumulated-cloud voxel size (0=off)')
    ap.add_argument('--point-stride', type=int, default=4, help='subsample points within each scan')
    ap.add_argument('--speed', type=float, default=8.0, help='scans per second during playback')
    args = ap.parse_args()

    label = Path(args.trajectory).stem
    print(f"[{label}] Loading trajectory...")
    times, poses = load_tum(args.trajectory)
    print(f"[{label}] {len(times)} poses. Loading scans from {args.bag} ...")
    scans = load_scans(args.bag, args.point_stride)
    print(f"[{label}] {len(scans)} scans loaded.")

    state = {'paused': False, 'step': False, 'quit': False, 'idx': 0}

    def toggle_pause(vis):
        state['paused'] = not state['paused']
        return False

    def step_once(vis):
        state['step'] = True
        return False

    def quit_now(vis):
        state['quit'] = True
        return False

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name=f"Incremental playback: {label}", width=1280, height=800)
    vis.register_key_callback(32, toggle_pause)   # space
    vis.register_key_callback(262, step_once)     # right arrow
    vis.register_key_callback(81, quit_now)       # Q
    vis.register_key_callback(256, quit_now)      # Esc

    accumulated = o3d.geometry.PointCloud()
    traj_line = o3d.geometry.LineSet()
    traj_points = []
    vis.add_geometry(accumulated)
    vis.add_geometry(traj_line)

    render_opt = vis.get_render_option()
    render_opt.point_size = 1.5
    render_opt.background_color = np.array([0.05, 0.05, 0.05])

    accumulated_xyz = []
    first = True

    def advance():
        nonlocal accumulated_xyz, first
        idx = state['idx']
        if idx >= len(scans):
            return False
        t, xyz = scans[idx]
        pose = nearest_pose(times, poses, t)
        state['idx'] = idx + args.stride
        if pose is None or len(xyz) == 0:
            return True
        xyz_h = np.hstack([xyz, np.ones((len(xyz), 1), dtype=np.float32)])
        world = (pose @ xyz_h.T).T[:, :3]
        accumulated_xyz.append(world.astype(np.float32))
        traj_points.append(pose[:3, 3].copy())

        merged = np.vstack(accumulated_xyz)
        accumulated.points = o3d.utility.Vector3dVector(merged)
        z = merged[:, 2]
        zmin, zmax = z.min(), z.max() + 1e-6
        colors = np.zeros((len(merged), 3))
        colors[:, 0] = (z - zmin) / (zmax - zmin)
        colors[:, 2] = 1.0 - colors[:, 0]
        accumulated.colors = o3d.utility.Vector3dVector(colors)
        if args.voxel > 0 and len(accumulated_xyz) % 20 == 0:
            down = accumulated.voxel_down_sample(args.voxel)
            accumulated.points = down.points
            accumulated.colors = down.colors
            accumulated_xyz = [np.asarray(accumulated.points, dtype=np.float32)]

        traj_line.points = o3d.utility.Vector3dVector(np.array(traj_points))
        if len(traj_points) > 1:
            lines = [[i, i + 1] for i in range(len(traj_points) - 1)]
            traj_line.lines = o3d.utility.Vector2iVector(lines)
            traj_line.colors = o3d.utility.Vector3dVector(
                [[1, 1, 0] for _ in lines])

        vis.update_geometry(accumulated)
        vis.update_geometry(traj_line)
        if first:
            vis.reset_view_point(True)
            first = False
        return True

    print("Controls: Space=pause/resume, Right-arrow=step, Q/Esc=quit")
    period = 1.0 / max(args.speed, 0.1)
    last = time.time()
    while not state['quit']:
        if not vis.poll_events():
            break
        vis.update_renderer()
        now = time.time()
        if state['step']:
            advance()
            state['step'] = False
            last = now
        elif not state['paused'] and now - last >= period:
            if not advance():
                print(f"[{label}] Done -- {len(traj_points)} poses, "
                      f"{sum(len(a) for a in accumulated_xyz)} points shown.")
                state['paused'] = True
            last = now
    vis.destroy_window()


if __name__ == '__main__':
    main()

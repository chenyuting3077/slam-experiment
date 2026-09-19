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
      [--cam-height M]

The camera is fixed, straight down, centered on the whole trajectory's
bounding-box center, set once before playback starts and never moved --
so you can watch the trajectory and map grow within a stable frame
instead of the view chasing/spinning around the current point.

To watch all four systems side by side at once, just launch four of these
in parallel (they free-run at the same --speed, so they stay roughly in
sync):
  for f in cartographer rtabmap liosam fastlio_official; do
    python3 incremental_playback_viewer.py outputs/trajectories/${f}_campus.txt &
  done

Controls once a window is focused:
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
    """Returns times, translations (N,3), quaternions (N,4) as (x,y,z,w)."""
    times, trans, quats = [], [], []
    with open(path) as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            t = float(parts[0])
            x, y, z, qx, qy, qz, qw = (float(v) for v in parts[1:8])
            times.append(t)
            trans.append([x, y, z])
            quats.append([qx, qy, qz, qw])
    return np.array(times), np.array(trans), np.array(quats)


def quat_to_matrix(q):
    qx, qy, qz, qw = q
    n = qx * qx + qy * qy + qz * qz + qw * qw
    s = 2.0 / n
    return np.array([
        [1 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw), s * (qx * qz + qy * qw)],
        [s * (qx * qy + qz * qw), 1 - s * (qx * qx + qz * qz), s * (qy * qz - qx * qw)],
        [s * (qx * qz - qy * qw), s * (qy * qz + qx * qw), 1 - s * (qx * qx + qy * qy)],
    ])


def slerp(q0, q1, alpha):
    dot = np.dot(q0, q1)
    if dot < 0:
        q1 = -q1
        dot = -dot
    dot = min(dot, 1.0)
    if dot > 0.9995:
        result = q0 + alpha * (q1 - q0)
        return result / np.linalg.norm(result)
    theta0 = np.arccos(dot)
    theta = theta0 * alpha
    q2 = q1 - q0 * dot
    q2 /= np.linalg.norm(q2)
    return q0 * np.cos(theta) + q2 * np.sin(theta)


def interpolate_pose(times, trans, quats, t):
    """Linear/slerp interpolation between the two bracketing keyframes --
    holding "nearest" instead causes visible camera stutter/jumps for
    systems with sparse keyframes (RTAB-Map, LIO-SAM, FAST-LIO2 all have
    far fewer poses than there are scans; only Cartographer is ~1:1)."""
    i = np.searchsorted(times, t)
    if i <= 0:
        i = 1
    if i >= len(times):
        i = len(times) - 1
    if t < times[0] - 2.0 or t > times[-1] + 2.0:
        return None  # well outside the trajectory's time range altogether
    t0, t1 = times[i - 1], times[i]
    if t1 - t0 < 1e-9:
        alpha = 0.0
    else:
        alpha = np.clip((t - t0) / (t1 - t0), 0.0, 1.0)
    position = trans[i - 1] * (1 - alpha) + trans[i] * alpha
    rot = quat_to_matrix(slerp(quats[i - 1], quats[i], alpha))
    transform = np.eye(4)
    transform[:3, :3] = rot
    transform[:3, 3] = position
    return transform


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


def load_gps(bag_path, topic='/gps/fix'):
    """Returns times, lats, lons, alts for every NavSatFix on `topic` --
    used as an independent (non-LiDAR) ground-truth reference, since the
    Campus bag actually does carry raw GPS even though the SLAM systems
    themselves don't use it."""
    times, lats, lons, alts = [], [], [], []
    with AnyReader([Path(bag_path)]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        for conn, timestamp, rawdata in reader.messages(connections=conns):
            msg = reader.deserialize(rawdata, conn.msgtype)
            times.append(timestamp / 1e9)
            lats.append(msg.latitude)
            lons.append(msg.longitude)
            alts.append(msg.altitude)
    return np.array(times), np.array(lats), np.array(lons), np.array(alts)


def gps_to_local_xy(lats, lons, lat0=None, lon0=None):
    """Equirectangular approximation -- plenty accurate over a ~1km loop."""
    if lat0 is None:
        lat0, lon0 = lats[0], lons[0]
    earth_r = 6378137.0
    east = np.radians(lons - lon0) * earth_r * np.cos(np.radians(lat0))
    north = np.radians(lats - lat0) * earth_r
    return east, north


def align_rigid_2d(p, q):
    """Best-fit rotation+translation (no scale) mapping 2D points p onto q,
    via Procrustes/Kabsch on their centroids -- used to bring GPS (in its
    own ENU frame) into a SLAM system's own, arbitrarily-rotated frame."""
    pc, qc = p - p.mean(axis=0), q - q.mean(axis=0)
    h = pc.T @ qc
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = vt.T @ u.T
    t = q.mean(axis=0) - r @ p.mean(axis=0)
    return r, t


def look_at_extrinsic(eye, target, up_world=np.array([0., 0., 1.])):
    """World-to-camera 4x4 (OpenCV/Open3D convention: +Z forward into the
    scene, +X right, +Y down)."""
    z_cam = target - eye
    norm = np.linalg.norm(z_cam)
    if norm < 1e-6:
        z_cam = np.array([1., 0., 0.])
    else:
        z_cam = z_cam / norm
    if abs(np.dot(z_cam, up_world)) > 0.999:
        up_world = np.array([0., 1., 0.])
    x_cam = np.cross(z_cam, up_world)
    x_cam /= np.linalg.norm(x_cam)
    y_cam = np.cross(z_cam, x_cam)
    r_c2w = np.stack([x_cam, y_cam, z_cam], axis=1)
    r_w2c = r_c2w.T
    extrinsic = np.eye(4)
    extrinsic[:3, :3] = r_w2c
    extrinsic[:3, 3] = -r_w2c @ eye
    return extrinsic


def cylinder_segment(p0, p1, radius, resolution=8):
    """A short cylinder mesh from p0 to p1 -- used to draw the trajectory as
    an actual tube instead of a GL line, since aliased line width is capped
    at a small hardware maximum (looked no thicker past ~10px in testing)
    regardless of what render_option.line_width is set to."""
    p0 = np.asarray(p0, dtype=np.float64)
    p1 = np.asarray(p1, dtype=np.float64)
    vec = p1 - p0
    height = np.linalg.norm(vec)
    if height < 1e-6:
        return None
    cyl = o3d.geometry.TriangleMesh.create_cylinder(
        radius=radius, height=height, resolution=resolution, split=1)
    axis = vec / height
    z = np.array([0., 0., 1.])
    v = np.cross(z, axis)
    c = float(np.dot(z, axis))
    if np.linalg.norm(v) < 1e-8:
        rot = np.eye(3) if c > 0 else -np.eye(3)
    else:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rot = np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))
    cyl.rotate(rot, center=(0, 0, 0))
    cyl.translate((p0 + p1) / 2.0)
    return cyl


def height_rainbow(z, zmin, zmax, darken=0.55):
    """RViz-style AxisColor rainbow: hue sweeps red -> yellow -> green ->
    cyan -> blue as height increases (matches the LIO-SAM README demo gif's
    look), instead of a plain two-color blend. Scaled down by `darken` so
    the white trajectory line reads clearly against the point cloud."""
    span = max(zmax - zmin, 1e-6)
    t = np.clip((z - zmin) / span, 0.0, 1.0)
    hue = t * (240.0 / 360.0)  # low=red(0) -> yellow -> green -> blue(high)
    h6 = hue * 6.0
    i = np.floor(h6).astype(np.int32) % 6
    f = h6 - np.floor(h6)
    colors = np.zeros((len(z), 3))
    for k, (r, g, b) in enumerate([
        (1, f, 0), (1 - f, 1, 0), (0, 1, f),
        (0, 1 - f, 1), (f, 0, 1), (1, 0, 1 - f),
    ]):
        mask = i == k
        colors[mask, 0] = r[mask] if hasattr(r, '__len__') else r
        colors[mask, 1] = g[mask] if hasattr(g, '__len__') else g
        colors[mask, 2] = b[mask] if hasattr(b, '__len__') else b
    return colors * darken


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('trajectory')
    ap.add_argument('--bag', default=DEFAULT_BAG)
    ap.add_argument('--stride', type=int, default=1, help='use every Nth scan')
    ap.add_argument('--voxel', type=float, default=0.5, help='accumulated-cloud voxel size (0=off)')
    ap.add_argument('--voxel-every', type=int, default=15,
                     help='re-voxel-merge the accumulated cloud every N scans -- doing this on '
                          'every single scan (1) declutters best but re-downsamples the whole, '
                          'ever-growing cloud each time, which gets too slow for a full run')
    ap.add_argument('--point-stride', type=int, default=8, help='subsample points within each scan')
    ap.add_argument('--point-size', type=float, default=1.0, help='rendered point size on screen')
    ap.add_argument('--speed', type=float, default=5120.0, help='scans per second during playback (~512x the sensor\'s native 10Hz)')
    ap.add_argument('--cam-height', type=float, default=22.0,
                     help='minimum camera height above the map (m); actually raised as '
                          'needed to fit the whole trajectory in frame')
    ap.add_argument('--win-width', type=int, default=960)
    ap.add_argument('--win-height', type=int, default=720)
    ap.add_argument('--win-x', type=int, default=50)
    ap.add_argument('--win-y', type=int, default=50)
    ap.add_argument('--color-span', type=float, default=11.0,
                     help='height range (m) mapped across the rainbow, starting from the '
                          'floor level (trajectory z-min - floor-margin)')
    ap.add_argument('--floor-margin', type=float, default=1.0,
                     help='hide points more than this far (m) below the trajectory\'s OWN '
                          'lowest pose -- relative to each system\'s own z, not a fixed world '
                          'height, since a system with unconstrained z drift (no loop closure) '
                          'can sink/rise by tens of meters and a fixed cutoff would wrongly '
                          'chop out real structure. Set very large to disable.')
    ap.add_argument('--traj-radius', type=float, default=2.0,
                     help='radius (m) of the tube drawn for the trajectory -- a real cylinder '
                          'mesh, not a GL line, since line width caps out at a small hardware max')
    ap.add_argument('--traj-seg-dist', type=float, default=0.3,
                     help='only add a new trajectory tube segment once the pose has moved at '
                          'least this far (m) -- keeps segment count reasonable')
    args = ap.parse_args()

    label = Path(args.trajectory).stem
    print(f"[{label}] Loading trajectory...")
    times, trans, quats = load_tum(args.trajectory)
    traj_z_min = float(trans[:, 2].min())
    traj_z_max = float(trans[:, 2].max())
    floor = traj_z_min - args.floor_margin
    color_zmin = floor
    color_zmax = max(floor + args.color_span, traj_z_max + args.floor_margin)
    traj_lift_z = color_zmax + 2.0  # draw the trajectory tube/marker flat up here so
                                     # they never end up underneath point-cloud structure
    print(f"[{label}] trajectory z=[{traj_z_min:.2f}, {traj_z_max:.2f}] -> floor={floor:.2f}, "
          f"color range [{color_zmin:.2f}, {color_zmax:.2f}], traj drawn at z={traj_lift_z:.2f}")
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
    vis.create_window(window_name=f"Incremental playback: {label}",
                       width=args.win_width, height=args.win_height,
                       left=args.win_x, top=args.win_y)
    vis.register_key_callback(32, toggle_pause)   # space
    vis.register_key_callback(262, step_once)     # right arrow
    vis.register_key_callback(81, quit_now)       # Q
    vis.register_key_callback(256, quit_now)      # Esc

    accumulated = o3d.geometry.PointCloud()
    traj_mesh = o3d.geometry.TriangleMesh()
    current_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.6)
    current_marker.paint_uniform_color([1.0, 0.15, 0.15])
    traj_points = []
    last_seg_point = None
    vis.add_geometry(accumulated)
    vis.add_geometry(traj_mesh)
    vis.add_geometry(current_marker)

    render_opt = vis.get_render_option()
    render_opt.point_size = args.point_size
    render_opt.background_color = np.array([0.0, 0.0, 0.0])
    marker_center = np.zeros(3)

    accumulated_xyz = []

    # Frame the camera on the whole trajectory's bounding-box center, high
    # enough to fit its whole extent (plus a fixed pad for structure that
    # sticks out past the trajectory itself, e.g. a building set back from
    # the path), once before playback starts -- and never touch it again,
    # so the view stays put while the map/trajectory grow inside it.
    map_min, map_max = trans.min(axis=0), trans.max(axis=0)
    map_center = (map_min + map_max) / 2.0
    half_extent = (map_max[:2] - map_min[:2]) / 2.0 + 50.0  # +50m pad
    vis.poll_events()
    vis.update_renderer()
    cam_params = vis.get_view_control().convert_to_pinhole_camera_parameters()
    fx, fy = cam_params.intrinsic.intrinsic_matrix[0, 0], cam_params.intrinsic.intrinsic_matrix[1, 1]
    w, h = cam_params.intrinsic.width, cam_params.intrinsic.height
    margin = 1.15
    height_for_x = half_extent[0] * margin * fx / (w / 2.0)
    height_for_y = half_extent[1] * margin * fy / (h / 2.0)
    init_height = max(args.cam_height, height_for_x, height_for_y, 20.0)
    cam_params.extrinsic = look_at_extrinsic(
        map_center + np.array([0., 0., init_height]), map_center,
        up_world=np.array([0., 1., 0.]))
    vis.get_view_control().convert_from_pinhole_camera_parameters(cam_params, allow_arbitrary=True)

    def advance():
        nonlocal accumulated_xyz, marker_center, last_seg_point, traj_mesh
        idx = state['idx']
        if idx >= len(scans):
            return False
        t, xyz = scans[idx]
        pose = interpolate_pose(times, trans, quats, t)
        state['idx'] = idx + args.stride
        if pose is None or len(xyz) == 0:
            return True
        xyz_h = np.hstack([xyz, np.ones((len(xyz), 1), dtype=np.float32)])
        world = (pose @ xyz_h.T).T[:, :3]
        world = world[world[:, 2] > floor]
        if len(world) == 0:
            return True
        accumulated_xyz.append(world.astype(np.float32))
        traj_points.append(pose[:3, 3].copy())

        merged = np.vstack(accumulated_xyz)
        accumulated.points = o3d.utility.Vector3dVector(merged)
        z = merged[:, 2]
        accumulated.colors = o3d.utility.Vector3dVector(
            height_rainbow(z, color_zmin, color_zmax))
        if args.voxel > 0 and len(accumulated_xyz) % args.voxel_every == 0:
            down = accumulated.voxel_down_sample(args.voxel)
            accumulated.points = down.points
            accumulated.colors = down.colors
            accumulated_xyz = [np.asarray(accumulated.points, dtype=np.float32)]

        current_pos = np.array([pose[0, 3], pose[1, 3], traj_lift_z])
        if last_seg_point is None:
            last_seg_point = current_pos.copy()
        elif np.linalg.norm(current_pos - last_seg_point) >= args.traj_seg_dist:
            seg = cylinder_segment(last_seg_point, current_pos, args.traj_radius)
            if seg is not None:
                seg.paint_uniform_color([1.0, 1.0, 1.0])
                traj_mesh += seg
                vis.update_geometry(traj_mesh)
            last_seg_point = current_pos.copy()

        marker_pos = np.array([pose[0, 3], pose[1, 3], traj_lift_z])
        current_marker.translate(marker_pos - marker_center)
        marker_center = marker_pos.copy()

        vis.update_geometry(accumulated)
        vis.update_geometry(current_marker)
        return True

    print("Controls: Space=pause/resume, Right-arrow=step, Q/Esc=quit")
    period = 1.0 / max(args.speed, 0.1)
    last = time.time()
    while not state['quit']:
        if not vis.poll_events():
            break
        # Re-lock the camera every frame -- Open3D's default mouse
        # drag/scroll handlers are still live and would otherwise let an
        # accidental scroll pan/zoom the "fixed" view away.
        vis.get_view_control().convert_from_pinhole_camera_parameters(cam_params, allow_arbitrary=True)
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

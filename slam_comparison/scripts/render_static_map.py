#!/usr/bin/env python3
"""Render a system's full trajectory + already-accumulated point cloud as a
single static top-down image -- no scan-by-scan playback, no re-reading the
raw bag. Loads the point cloud a system already exported (outputs/pointclouds
or outputs/liosam_campus/GlobalMap.pcd) instead of rebuilding it from scans,
so this is fast and just needs one render.

Usage:
  python3 render_static_map.py outputs/trajectories/liosam_campus.txt \
      --cloud outputs/liosam_campus/GlobalMap.pcd
"""
import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

from incremental_playback_viewer import (
    load_tum, height_rainbow, cylinder_segment, look_at_extrinsic,
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('trajectory')
    ap.add_argument('--cloud', required=True, help='already-exported .pcd/.ply point cloud')
    ap.add_argument('--out', default=None, help='PNG output path (default: outputs/<label>_map.png)')
    ap.add_argument('--traj-radius', type=float, default=2.0)
    ap.add_argument('--traj-seg-dist', type=float, default=1.0,
                     help='coarser than the live viewer -- this is a one-shot static render')
    ap.add_argument('--color-span', type=float, default=11.0,
                     help='height range (m) mapped across the rainbow, starting from the '
                          'floor level (trajectory z-min - floor-margin)')
    ap.add_argument('--floor-margin', type=float, default=1.0,
                     help='hide points more than this far (m) below the trajectory\'s OWN '
                          'lowest pose -- relative to each system\'s own z, not a fixed world '
                          'height, since a system with unconstrained z drift (no loop closure) '
                          'can sink/rise by tens of meters and a fixed cutoff would wrongly '
                          'chop out real structure')
    ap.add_argument('--win-width', type=int, default=1280)
    ap.add_argument('--win-height', type=int, default=960)
    ap.add_argument('--cam-height', type=float, default=22.0)
    ap.add_argument('--point-size', type=float, default=1.0)
    ap.add_argument('--traj-lift-margin', type=float, default=2.0,
                     help='draw the trajectory tube flat at (point-cloud z-max + this margin) '
                          'instead of its real height, so it never ends up underneath/behind '
                          'point-cloud structure from the top-down view. Only affects the '
                          'tube\'s drawn Z, not the real pose.')
    args = ap.parse_args()

    label = Path(args.trajectory).stem
    out_path = args.out or f'/home/allen/slam-experiment/outputs/{label}_map.png'

    print(f"[{label}] Loading trajectory...")
    times, trans, quats = load_tum(args.trajectory)
    traj_z_min = float(trans[:, 2].min())
    floor = traj_z_min - args.floor_margin
    print(f"[{label}] trajectory z-min={traj_z_min:.2f} -> floor={floor:.2f}")

    print(f"[{label}] Loading point cloud from {args.cloud} ...")
    cloud = o3d.io.read_point_cloud(args.cloud)
    pts = np.asarray(cloud.points)
    pts = pts[np.isfinite(pts).all(axis=1)]
    pts = pts[pts[:, 2] > floor]
    print(f"[{label}] {len(pts)} points after floor cutoff.")
    # Normalize color to THIS system's own filtered point range, not a fixed
    # span -- each system's z is on a different scale (some drift tens of
    # meters, some stay within a couple), so a shared absolute scale would
    # make most of them look like a single flat color. Normalizing per
    # system means every render uses the full rainbow and is comparable in
    # *relative* height variation, even though the same color no longer
    # means the same absolute height across the four systems.
    color_zmin = floor
    color_zmax = float(pts[:, 2].max()) if len(pts) else floor + args.color_span
    if color_zmax <= color_zmin:
        color_zmax = color_zmin + 1e-3
    print(f"[{label}] color range [{color_zmin:.2f}, {color_zmax:.2f}] (normalized)")
    colors = height_rainbow(pts[:, 2], color_zmin, color_zmax)
    render_cloud = o3d.geometry.PointCloud()
    render_cloud.points = o3d.utility.Vector3dVector(pts)
    render_cloud.colors = o3d.utility.Vector3dVector(colors)

    traj_lift_z = float(pts[:, 2].max()) + args.traj_lift_margin if len(pts) else 0.0
    print(f"[{label}] Drawing trajectory tube flat at z={traj_lift_z:.2f} "
          f"({len(trans)} poses) so it stays on top of the cloud...")
    traj_mesh = o3d.geometry.TriangleMesh()
    last = None
    for p in trans:
        p_flat = np.array([p[0], p[1], traj_lift_z])
        if last is None:
            last = p_flat
            continue
        if np.linalg.norm(p_flat - last) >= args.traj_seg_dist:
            seg = cylinder_segment(last, p_flat, args.traj_radius)
            if seg is not None:
                seg.paint_uniform_color([1.0, 1.0, 1.0])
                traj_mesh += seg
            last = p_flat

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=f"Map: {label}", width=args.win_width,
                       height=args.win_height, visible=True)
    vis.add_geometry(render_cloud)
    vis.add_geometry(traj_mesh)
    ro = vis.get_render_option()
    ro.point_size = args.point_size
    ro.background_color = np.array([0.0, 0.0, 0.0])

    map_min, map_max = trans.min(axis=0), trans.max(axis=0)
    map_center = (map_min + map_max) / 2.0
    map_radius = float(np.linalg.norm((map_max - map_min)[:2])) / 2.0
    init_height = max(args.cam_height, map_radius * 1.6, 20.0)
    vis.poll_events()
    vis.update_renderer()
    cam = vis.get_view_control().convert_to_pinhole_camera_parameters()
    cam.extrinsic = look_at_extrinsic(
        map_center + np.array([0., 0., init_height]), map_center,
        up_world=np.array([0., 1., 0.]))
    vis.get_view_control().convert_from_pinhole_camera_parameters(cam, allow_arbitrary=True)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(out_path, do_render=True)
    vis.destroy_window()
    print(f"[{label}] Saved {out_path}")


if __name__ == '__main__':
    main()

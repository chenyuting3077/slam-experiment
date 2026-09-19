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
from scipy.ndimage import median_filter

from incremental_playback_viewer import (
    load_tum, height_rainbow, cylinder_segment, look_at_extrinsic,
    interpolate_pose, load_gps, gps_to_local_xy, align_rigid_2d, DEFAULT_BAG,
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
    ap.add_argument('--bag', default=DEFAULT_BAG)
    ap.add_argument('--no-gps', action='store_true',
                     help='skip overlaying the /gps/fix ground-truth track')
    ap.add_argument('--gps-radius', type=float, default=1.5)
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

    gps_mesh = None
    gps_xy_aligned = None
    if not args.no_gps:
        print(f"[{label}] Loading /gps/fix from {args.bag} for ground-truth overlay...")
        gps_times, lats, lons, alts = load_gps(args.bag)
        if len(gps_times) > 3:
            east, north = gps_to_local_xy(lats, lons)
            gps_local = np.stack([east, north], axis=1)
            slam_xy_at_gps, gps_valid = [], []
            for i, t in enumerate(gps_times):
                pose = interpolate_pose(times, trans, quats, t)
                if pose is not None:
                    slam_xy_at_gps.append(pose[:2, 3])
                    gps_valid.append(i)
            if len(gps_valid) >= 3:
                p = gps_local[gps_valid]
                q = np.array(slam_xy_at_gps)
                r, t_off = align_rigid_2d(p, q)
                gps_xy_aligned = (r @ gps_local.T).T + t_off
                print(f"[{label}] Aligned {len(gps_valid)}/{len(gps_times)} GPS fixes to "
                      f"this system's frame (rigid 2D fit).")
                # Raw single-antenna GPS near buildings throws occasional huge
                # multipath spikes (tens of meters, one fix at a time) -- a
                # short median filter kills those without smearing the real
                # loop shape, since a spike is a single outlier sample, not a
                # sustained trend. Fit alignment above used the raw fixes;
                # this filtering is display-only.
                gps_xy_smooth = np.stack([
                    median_filter(gps_xy_aligned[:, 0], size=9, mode='nearest'),
                    median_filter(gps_xy_aligned[:, 1], size=9, mode='nearest'),
                ], axis=1)
                # Break the drawn line (instead of bridging with a straight
                # segment) wherever consecutive fixes still jump further than
                # plausible walking/vehicle speed allows -- leftover multipath
                # excursions the median filter didn't fully absorb, otherwise
                # each one draws a long, misleading straight streak.
                max_jump = 8.0
                gps_mesh = o3d.geometry.TriangleMesh()
                last_g = None
                for xy in gps_xy_smooth:
                    g_flat = np.array([xy[0], xy[1], traj_lift_z])
                    if last_g is None:
                        last_g = g_flat
                        continue
                    if np.linalg.norm(g_flat - last_g) > max_jump:
                        last_g = g_flat
                        continue
                    if np.linalg.norm(g_flat - last_g) >= args.traj_seg_dist:
                        seg = cylinder_segment(last_g, g_flat, args.gps_radius)
                        if seg is not None:
                            seg.paint_uniform_color([1.0, 1.0, 0.0])  # yellow, distinct from white
                            gps_mesh += seg
                        last_g = g_flat
            else:
                print(f"[{label}] Not enough overlapping GPS/trajectory time range -- skipping.")
        else:
            print(f"[{label}] No usable /gps/fix messages -- skipping.")

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=f"Map: {label}", width=args.win_width,
                       height=args.win_height, visible=True)
    vis.add_geometry(render_cloud)
    vis.add_geometry(traj_mesh)
    if gps_mesh is not None:
        vis.add_geometry(gps_mesh)
    ro = vis.get_render_option()
    ro.point_size = args.point_size
    ro.background_color = np.array([0.0, 0.0, 0.0])

    # Frame on the union of the trajectory, the point cloud, AND the GPS
    # overlay's own extent -- buildings/trees can stick out past the
    # trajectory's bounding box, and cropping them was exactly the earlier
    # "PNG isn't complete" complaint.
    xy_min = np.minimum(trans[:, :2].min(axis=0), pts[:, :2].min(axis=0)) if len(pts) else trans[:, :2].min(axis=0)
    xy_max = np.maximum(trans[:, :2].max(axis=0), pts[:, :2].max(axis=0)) if len(pts) else trans[:, :2].max(axis=0)
    if gps_xy_aligned is not None:
        xy_min = np.minimum(xy_min, gps_xy_aligned.min(axis=0))
        xy_max = np.maximum(xy_max, gps_xy_aligned.max(axis=0))
    map_center_xy = (xy_min + xy_max) / 2.0
    map_center = np.array([map_center_xy[0], map_center_xy[1], float(trans[:, 2].mean())])
    half_extent = (xy_max - xy_min) / 2.0

    vis.poll_events()
    vis.update_renderer()
    cam = vis.get_view_control().convert_to_pinhole_camera_parameters()
    fx, fy = cam.intrinsic.intrinsic_matrix[0, 0], cam.intrinsic.intrinsic_matrix[1, 1]
    w, h = cam.intrinsic.width, cam.intrinsic.height
    margin = 1.15  # a bit of headroom past the exact fit
    height_for_x = half_extent[0] * margin * fx / (w / 2.0)
    height_for_y = half_extent[1] * margin * fy / (h / 2.0)
    init_height = max(args.cam_height, height_for_x, height_for_y, 20.0)
    print(f"[{label}] framing: xy half-extent={half_extent}, camera height={init_height:.1f}m")
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

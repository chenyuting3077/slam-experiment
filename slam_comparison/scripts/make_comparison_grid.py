#!/usr/bin/env python3
"""Combine four systems' static map PNGs (from render_static_map.py) into
one 2x2 grid image with labels. A tile whose file is missing, or that's
explicitly listed in --failed, is rendered as a labeled "FAILURE" panel
instead of being silently dropped -- so a failed system still gets a slot
in the figure rather than disappearing from the comparison.

Usage:
  python3 make_comparison_grid.py
  python3 make_comparison_grid.py --set mid360
"""
import argparse
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageOps

TILE_SETS = {
    'campus': [
        ('Cartographer 3D', 'outputs/cartographer_campus_map.png', None),
        ('RTAB-Map (pure ICP)', 'outputs/rtabmap_campus_map.png', None),
        ('LIO-SAM', 'outputs/liosam_campus_map.png', None),
        ('FAST-LIO2', 'outputs/fastlio_official_campus_map.png', None),
    ],
    'mid360': [
        ('Cartographer 3D', 'outputs/cartographer_mid360_map.png',
         'FAILURE\ndiverges within ~0.1s of start\n(pose extrapolator, not fixed by\ntuning imu_gravity_time_constant)'),
        ('RTAB-Map (pure ICP)', 'outputs/rtabmap_mid360_map.png', None),
        ('LIO-SAM', 'outputs/liosam_mid360_map.png', None),
        ('FAST-LIO2', 'outputs/fastlio_mid360_map.png', None),
    ],
    'mid360_kidnap': [
        ('Cartographer 3D', 'outputs/cartographer_mid360_kidnap_map.png',
         'FAILURE\nsame instant divergence\nas outdoor_hard_01'),
        ('RTAB-Map (pure ICP)', 'outputs/rtabmap_mid360_kidnap_map.png', None),
        ('LIO-SAM', 'outputs/liosam_mid360_kidnap_map.png', None),
        ('FAST-LIO2', 'outputs/fastlio_mid360_kidnap_map.png', None),
    ],
    'tiers_cut1': [
        ('Cartographer 3D', 'outputs/cartographer_tiers_cut1_map.png',
         'FAILURE\nsame instant divergence\n(3rd dataset, 2nd source, same bug)'),
        ('RTAB-Map (pure ICP)', 'outputs/rtabmap_tiers_cut1_map.png', None),
        ('LIO-SAM', 'outputs/liosam_tiers_cut1_map.png', None),
        ('FAST-LIO2', 'outputs/fastlio_tiers_cut1_map.png', None),
    ],
    'tiers_cut0': [
        ('Cartographer 3D', 'outputs/cartographer_tiers_cut0_map.png',
         'FAILURE\nsame instant divergence\n(4th dataset, 2nd source, same bug)'),
        ('RTAB-Map (pure ICP)', 'outputs/rtabmap_tiers_cut0_map.png', None),
        ('LIO-SAM', 'outputs/liosam_tiers_cut0_map.png', None),
        ('FAST-LIO2', 'outputs/fastlio_tiers_cut0_map.png', None),
    ],
    'tiers_indoor1': [
        ('Cartographer 3D', 'outputs/cartographer_tiers_indoor1_map.png',
         'FAILURE\nsame instant divergence\n(5th dataset, indoor scene too)'),
        ('RTAB-Map (pure ICP)', 'outputs/rtabmap_tiers_indoor1_map.png', None),
        ('LIO-SAM', 'outputs/liosam_tiers_indoor1_map.png', None),
        ('FAST-LIO2', 'outputs/fastlio_tiers_indoor1_map.png', None),
    ],
    'tiers_indoor2': [
        ('Cartographer 3D', 'outputs/cartographer_tiers_indoor2_map.png',
         'FAILURE\nsame instant divergence\n(6th dataset, same bug)'),
        ('RTAB-Map (pure ICP)', 'outputs/rtabmap_tiers_indoor2_map.png', None),
        ('LIO-SAM', 'outputs/liosam_tiers_indoor2_map.png', None),
        ('FAST-LIO2', 'outputs/fastlio_tiers_indoor2_map.png', None),
    ],
    'garden': [
        ('Cartographer 3D', 'outputs/cartographer_garden_map.png',
         'FAILURE\nhard crash (SIGABRT) at t=27s:\nimu_tracker.cc gravity CHECK failed\nduring a real sharp turn\nsee CARTOGRAPHER_FAILURE.md'),
        ('RTAB-Map (pure ICP)', 'outputs/rtabmap_garden_map.png',
         'FAILURE\nICP lost tracking ~15s in\n(same sharp turn), never recovered\n-- only 1 keyframe for the whole run'),
        ('LIO-SAM', 'outputs/liosam_garden_map.png', None),
        ('FAST-LIO2', 'outputs/fastlio_garden_map.png', None),
    ],
    'park': [
        ('Cartographer 3D', 'outputs/cartographer_park_map.png',
         'FAILURE\nsilent divergence (no crash this time)\n/tf shows tens of millions of meters\nsame bug, 560s dataset'),
        ('RTAB-Map (pure ICP)', 'outputs/rtabmap_park_map.png', None),
        ('LIO-SAM', 'outputs/liosam_park_map.png', None),
        ('FAST-LIO2', 'outputs/fastlio_park_map.png', None),
    ],
    'rotation': [
        ('Cartographer 3D', 'outputs/cartographer_rotation_map.png',
         'FAILURE\nhard crash (SIGABRT):\nimu_tracker.cc CHECK_GT(z,0) failed\n-- pure fast rotation alone (3.7 rad/s)\nis enough, no big linear accel needed'),
        ('RTAB-Map (pure ICP)', 'outputs/rtabmap_rotation_map.png',
         'FAILURE\nICP rotation limit exceeded\nimmediately (fast in-place spin)\n-- only 1 keyframe for the whole run'),
        ('LIO-SAM', 'outputs/liosam_rotation_map.png', None),
        ('FAST-LIO2', 'outputs/fastlio_rotation_map.png', None),
    ],
    'legkilo_corridor': [
        ('Cartographer 3D (+ real leg odom)', 'outputs/cartographer_legkilo_corridor_map.png',
         'FAILURE\nsilent divergence to ~8600m\neven WITH real /state_SDK odometry\n(~1000x smaller than without odom,\nbut still wrong -- see CARTOGRAPHER_FAILURE.md)'),
        ('RTAB-Map (pure ICP)', 'outputs/rtabmap_legkilo_corridor_map.png', None),
        ('LIO-SAM', 'outputs/liosam_legkilo_corridor_map.png', None),
        ('FAST-LIO2 (official)', 'outputs/fastlio_legkilo_corridor_map.png',
         'FAILURE\nATE 272m -- classic long feature-poor\ncorridor degeneracy, front-end ESKF\nhas no loop closure to correct it'),
        ('FAST_LIO_SAM', 'outputs/fastlio_sam_legkilo_corridor_map.png',
         'FAILURE\nATE 61062m -- same front-end\ndegeneracy as FAST-LIO2, made\ncatastrophically worse by the backend'),
    ],
    'compal_amr': [
        ('Cartographer 3D', 'outputs/compal_amr_cartographer/cartographer_compal_amr_map.png', None),
        ('RTAB-Map (external odom)', 'outputs/compal_amr_rtabmap/rtabmap_compal_amr_map.png', None),
        ('FAST-LIO2 (official)', 'outputs/compal_amr_fastlio2/fastlio2_compal_amr_map.png', None),
        ('LIO-SAM (diverges after ~17/30 keyframes)', 'outputs/compal_amr_liosam/liosam_compal_amr_map.png', None),
    ],
    'compal_amr_full': [
        ('Cartographer 3D (end-to-end 4.4cm)', 'docs/images/compal_amr/full_809s_individual/cartographer_compal_amr_full_map.png', None),
        ('RTAB-Map (end-to-end 1.5cm)', 'docs/images/compal_amr/full_809s_individual/rtabmap_compal_amr_full_map.png', None),
        ('FAST-LIO2 (diverges from ~200 s)', 'docs/images/compal_amr/full_809s_individual/fastlio2_compal_amr_full_map.png', None),
        ('LIO-SAM (diverges from the start)', 'docs/images/compal_amr/full_809s_individual/liosam_compal_amr_full_map.png', None),
    ],
    'compal_amr_2026_09_21': [
        ('Cartographer 3D (end-to-end 2.64cm)', 'docs/images/compal_amr/2026_09_21_individual/cartographer_compal_amr_2026_09_21_map.png', None),
        ('RTAB-Map (end-to-end 1.25m)', 'docs/images/compal_amr/2026_09_21_individual/rtabmap_compal_amr_2026_09_21_map.png', None),
        ('FAST-LIO2 (end-to-end 1.10m)', 'docs/images/compal_amr/2026_09_21_individual/fastlio2_compal_amr_2026_09_21_map.png', None),
        ('LIO-SAM (diverges from ~keyframe 8/106)', 'docs/images/compal_amr/2026_09_21_individual/liosam_compal_amr_2026_09_21_map.png', None),
    ],
}
DEFAULT_TILES = TILE_SETS['campus']


def failure_tile(size, reason):
    img = Image.new('RGB', size, (20, 0, 0))
    draw = ImageDraw.Draw(img)
    try:
        font_big = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 48)
        font_small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 22)
    except OSError:
        font_big = font_small = ImageFont.load_default()
    lines = reason.split('\n')
    big, rest = lines[0], lines[1:]
    tw = draw.textlength(big, font=font_big)
    cy = size[1] / 2 - 60
    draw.text((size[0] / 2 - tw / 2, cy), big, fill=(220, 60, 60), font=font_big)
    cy += 70
    for line in rest:
        tw = draw.textlength(line, font=font_small)
        draw.text((size[0] / 2 - tw / 2, cy), line, fill=(200, 150, 150), font=font_small)
        cy += 28
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--set', default='campus', choices=list(TILE_SETS))
    ap.add_argument('--out', default=None)
    ap.add_argument('--label-height', type=int, default=50)
    args = ap.parse_args()

    entries = TILE_SETS[args.set]
    out_path = args.out or f'/home/allen/slam-experiment/outputs/comparison_map_grid_{args.set}.png'

    # Tile size = largest existing tile; smaller ones are centered on black.
    sizes = [Image.open(path).size for _, path, _ in entries if Path(path).exists()]
    if not sizes:
        raise SystemExit("No tile images exist -- nothing to build a grid from.")
    ref_size = (max(w for w, _ in sizes), max(h for _, h in sizes))
    tile_w_ref = ref_size[0]

    tiles = []
    for label, path, reason in entries:
        if reason is not None or not Path(path).exists():
            img = failure_tile(ref_size, reason or 'FAILURE\n(no map produced)')
        else:
            img = ImageOps.pad(Image.open(path).convert('RGB'), (tile_w_ref, Image.open(path).size[1]), color=(0, 0, 0))
        tiles.append((label, img))

    tile_w = ref_size[0]
    label_h = args.label_height
    num_rows = (len(tiles) + 1) // 2
    # Each row is as tall as its tallest tile (tiles are centered on black in their cell).
    row_h = [max(t[1].size[1] for t in tiles[r * 2:r * 2 + 2]) for r in range(num_rows)]
    row_y = [sum(row_h[:r]) + label_h * r for r in range(num_rows)]
    grid_w, grid_h = tile_w * 2, sum(row_h) + label_h * num_rows

    grid = Image.new('RGB', (grid_w, grid_h), (0, 0, 0))
    draw = ImageDraw.Draw(grid)
    try:
        font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 28)
    except OSError:
        font = ImageFont.load_default()

    for i, (label, img) in enumerate(tiles):
        col, row = i % 2, i // 2
        x, y = col * tile_w, row_y[row]
        draw.rectangle([x, y, x + tile_w, y + label_h], fill=(30, 30, 30))
        text_w = draw.textlength(label, font=font)
        draw.text((x + (tile_w - text_w) / 2, y + (label_h - 28) / 2), label,
                   fill=(255, 255, 255), font=font)
        grid.paste(img, (x, y + label_h + (row_h[row] - img.size[1]) // 2))

    grid.save(out_path)
    print(f"Saved {out_path}")


if __name__ == '__main__':
    main()

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

from PIL import Image, ImageDraw, ImageFont

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

    # Reference size from the first tile that actually exists.
    ref_size = None
    for _, path, _ in entries:
        if Path(path).exists():
            ref_size = Image.open(path).size
            break
    if ref_size is None:
        raise SystemExit("No tile images exist -- nothing to build a grid from.")

    tiles = []
    for label, path, reason in entries:
        if reason is not None or not Path(path).exists():
            img = failure_tile(ref_size, reason or 'FAILURE\n(no map produced)')
        else:
            img = Image.open(path).convert('RGB')
        tiles.append((label, img))

    tile_w, tile_h = ref_size
    label_h = args.label_height
    grid_w, grid_h = tile_w * 2, (tile_h + label_h) * 2

    grid = Image.new('RGB', (grid_w, grid_h), (0, 0, 0))
    draw = ImageDraw.Draw(grid)
    try:
        font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 28)
    except OSError:
        font = ImageFont.load_default()

    for i, (label, img) in enumerate(tiles):
        col, row = i % 2, i // 2
        x, y = col * tile_w, row * (tile_h + label_h)
        draw.rectangle([x, y, x + tile_w, y + label_h], fill=(30, 30, 30))
        text_w = draw.textlength(label, font=font)
        draw.text((x + (tile_w - text_w) / 2, y + (label_h - 28) / 2), label,
                   fill=(255, 255, 255), font=font)
        grid.paste(img, (x, y + label_h))

    grid.save(out_path)
    print(f"Saved {out_path}")


if __name__ == '__main__':
    main()

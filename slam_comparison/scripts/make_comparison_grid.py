#!/usr/bin/env python3
"""Combine the four systems' static map PNGs (from render_static_map.py)
into one 2x2 grid image with labels, for a single side-by-side figure.

Usage:
  python3 make_comparison_grid.py
"""
import argparse
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

DEFAULT_TILES = [
    ('Cartographer 3D', 'outputs/cartographer_campus_map.png'),
    ('RTAB-Map (pure ICP)', 'outputs/rtabmap_campus_map.png'),
    ('LIO-SAM', 'outputs/liosam_campus_map.png'),
    ('FAST-LIO2', 'outputs/fastlio_official_campus_map.png'),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/home/allen/slam-experiment/outputs/comparison_map_grid.png')
    ap.add_argument('--label-height', type=int, default=50)
    args = ap.parse_args()

    tiles = [(label, Image.open(path).convert('RGB')) for label, path in DEFAULT_TILES]
    tile_w, tile_h = tiles[0][1].size
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

    grid.save(args.out)
    print(f"Saved {args.out}")


if __name__ == '__main__':
    main()

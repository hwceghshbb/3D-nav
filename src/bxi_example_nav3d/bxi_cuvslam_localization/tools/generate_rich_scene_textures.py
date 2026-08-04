#!/usr/bin/env python3
"""Generate deterministic, feature-rich textures for the cuVSLAM test scene."""

import argparse
from pathlib import Path
import random

from PIL import Image, ImageDraw, ImageFont


PALETTES = (
    ((235, 238, 232), (31, 91, 138), (205, 70, 49), (47, 126, 88)),
    ((228, 232, 240), (103, 55, 138), (224, 151, 36), (24, 111, 132)),
    ((239, 233, 219), (42, 67, 101), (176, 62, 74), (70, 130, 128)),
    ((225, 237, 232), (112, 65, 45), (40, 100, 156), (213, 130, 45)),
)


def make_texture(index, output):
    rng = random.Random(9000 + index)
    background, *accents = PALETTES[index % len(PALETTES)]
    image = Image.new("RGB", (1024, 512), background)
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default(size=28)

    for cell_y in range(4):
        for cell_x in range(8):
            x0, y0 = cell_x * 128, cell_y * 128
            color = accents[(cell_x + 2 * cell_y + index) % len(accents)]
            inset = rng.randint(10, 28)
            shape = (cell_x * 3 + cell_y + index) % 4
            box = (x0 + inset, y0 + inset, x0 + 128 - inset, y0 + 128 - inset)
            if shape == 0:
                draw.rectangle(box, fill=color, outline=(20, 20, 20), width=4)
            elif shape == 1:
                draw.ellipse(box, fill=color, outline=(20, 20, 20), width=4)
            elif shape == 2:
                draw.polygon(
                    ((x0 + 64, y0 + 8), (x0 + 120, y0 + 112), (x0 + 8, y0 + 112)),
                    fill=color,
                    outline=(20, 20, 20),
                )
            else:
                draw.line((x0 + 15, y0 + 105, x0 + 110, y0 + 20), fill=color, width=18)
                draw.line((x0 + 15, y0 + 20, x0 + 110, y0 + 105), fill=(20, 20, 20), width=7)
            draw.text(
                (x0 + 8, y0 + 8),
                f"{chr(65 + index)}{cell_y * 8 + cell_x:02d}",
                fill=(10, 10, 10),
                font=font,
            )

    for _ in range(90):
        x, y = rng.randrange(1024), rng.randrange(512)
        radius = rng.randrange(2, 7)
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=accents[rng.randrange(3)])

    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    for index in range(4):
        make_texture(index, args.output_dir / f"rich_wall_{index}.png")


if __name__ == "__main__":
    main()

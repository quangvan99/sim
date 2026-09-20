"""Generate a small Nav2 PGM map from the simulator's JSON obstacle map."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def generate(source: Path, image: Path, yaml_path: Path) -> None:
    data = json.loads(source.read_text())
    resolution = float(data["resolution_m"])
    bounds = data["bounds"]
    width = round((bounds["max_x"] - bounds["min_x"]) / resolution)
    height = round((bounds["max_y"] - bounds["min_y"]) / resolution)
    pixels = [100] * (width * height)

    for obstacle in data["obstacles"]:
        radius = float(obstacle["radius"])
        center_x = float(obstacle["x"])
        center_y = float(obstacle["y"])
        for row in range(height):
            y = bounds["min_y"] + (row + 0.5) * resolution
            for column in range(width):
                x = bounds["min_x"] + (column + 0.5) * resolution
                if math.hypot(x - center_x, y - center_y) <= radius:
                    pixels[(height - 1 - row) * width + column] = 0

    image.parent.mkdir(parents=True, exist_ok=True)
    image.write_text(
        "P2\n"
        f"{width} {height}\n"
        "100\n"
        + "\n".join(
            " ".join(str(value) for value in pixels[row : row + width])
            for row in range(0, len(pixels), width)
        )
        + "\n"
    )
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.write_text(
        f"image: {image.name}\n"
        f"resolution: {resolution}\n"
        f"origin: [{bounds['min_x']}, {bounds['min_y']}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
        "mode: trinary\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("image", type=Path)
    parser.add_argument("yaml", type=Path)
    args = parser.parse_args()
    generate(args.source, args.image, args.yaml)


if __name__ == "__main__":
    main()

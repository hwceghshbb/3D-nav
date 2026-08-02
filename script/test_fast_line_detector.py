#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PKG_SRC = ROOT / "src" / "bxi_example_py_elf3"
if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))

from bxi_example_py_elf3.fast_line_detector import FastLineDetector


def make_frame(index: int, width: int, height: int, dropout: bool = False) -> np.ndarray:
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:] = (35, 90, 35)
    y = np.arange(height, dtype=np.float32)
    center = width * 0.5 + 0.10 * width * ((y - height * 0.5) / height) ** 2 * width / 160.0
    center += 0.04 * width * np.sin(index * 0.03)
    half_width = 0.20 * width + 0.10 * (height - y)
    left = np.clip((center - half_width).astype(np.int32), 0, width - 1)
    right = np.clip((center + half_width).astype(np.int32), 0, width - 1)
    polygon = np.column_stack((np.r_[left, right[::-1]], np.r_[y.astype(np.int32), y[::-1].astype(np.int32)]))
    cv2.fillPoly(image, [polygon], (25, 25, 190))
    if dropout:
        cv2.rectangle(image, (0, int(height * 0.45)), (width, int(height * 0.65)), (35, 90, 35), -1)
    noise = np.random.default_rng(index).normal(0.0, 3.0, image.shape).astype(np.int16)
    return np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--output", type=Path, default=Path("/tmp/fast_line_detector_test"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    detector = FastLineDetector()
    controls = []
    confidences = []
    for index in range(args.frames):
        detection = detector.detect(make_frame(index, 160, 96, dropout=35 <= index <= 38))
        controls.append(float(detection["control_error"]))
        confidences.append(float(detection["confidence"]))
        if index in (0, 35, 38, args.frames - 1):
            frame = make_frame(index, 160, 96, dropout=35 <= index <= 38)
            mask = detection["mask"]
            overlay = frame.copy()
            points = np.asarray(detection["curve_points"], dtype=np.int32)
            if len(points) > 1:
                cv2.polylines(overlay, [points], False, (0, 255, 0), 2)
            cv2.imwrite(str(args.output / f"frame_{index:04d}.png"), overlay)
            cv2.imwrite(str(args.output / f"mask_{index:04d}.png"), mask)

    assert max(abs(value) for value in controls) <= 1.0
    assert min(confidences[39:]) >= 0.0
    print(f"frames={args.frames} control_range=[{min(controls):+.3f}, {max(controls):+.3f}]")
    print(f"dropout_confidence={confidences[35]:.3f}->{confidences[38]:.3f}")
    print(f"saved={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

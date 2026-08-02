#!/usr/bin/env python3
import argparse
import os
import sys

import cv2
import numpy as np


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG_SRC = os.path.join(REPO_ROOT, "src", "bxi_example_py_elf3")
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)

from bxi_example_py_elf3.traditional_line_detector import TraditionalLineDetector
from bxi_example_py_elf3.ufldv2_line_detector import UFLDv2LineDetector


def parse_crop(value):
    if not value:
        return None
    parts = [int(part.strip()) for part in value.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("crop must be x,y,w,h")
    x, y, w, h = parts
    if w <= 0 or h <= 0:
        raise argparse.ArgumentTypeError("crop width/height must be positive")
    return x, y, w, h


def draw_detection(image, detection):
    overlay = image.copy()
    height, width = overlay.shape[:2]
    roi_top = detection["roi_top"]
    roi_bottom = detection["roi_bottom"]
    roi_left = detection["roi_left"]
    roi_right = detection["roi_right"]

    def points_in_roi(points):
        return [
            (int(point[0]), int(point[1]))
            for point in points
            if len(point) >= 2
            if roi_left <= int(point[0]) <= roi_right and roi_top <= int(point[1]) <= roi_bottom
        ]

    cv2.rectangle(overlay, (roi_left, roi_top), (roi_right, roi_bottom), (50, 180, 50), 1)

    for points in detection["candidate_curves"]:
        if isinstance(points, dict):
            points = points.get("points", [])
        points = points_in_roi(points)
        if len(points) >= 2:
            cv2.polylines(overlay, [np.asarray(points, dtype=np.int32)], False, (120, 120, 120), 1, cv2.LINE_AA)

    for key, color, thickness in (
        ("left_curve_points", (255, 160, 0), 2),
        ("right_curve_points", (0, 220, 255), 2),
        ("center_curve_points", (0, 255, 0), 3),
    ):
        points = points_in_roi(detection[key])
        if len(points) >= 2:
            cv2.polylines(overlay, [np.asarray(points, dtype=np.int32)], False, color, thickness, cv2.LINE_AA)

    center_x = int(width * 0.5)
    cv2.line(overlay, (center_x, 0), (center_x, height - 1), (80, 80, 255), 1)
    raw_center_x = int(np.clip(detection["raw_lane_center"], 0, width - 1))
    cv2.line(overlay, (raw_center_x, int(roi_top)), (raw_center_x, int(roi_bottom)), (120, 220, 220), 1)
    target_x, target_y = detection["target_point"]
    cv2.circle(overlay, (int(target_x), int(target_y)), 6, (80, 255, 80), -1)

    status = (
        f"ctrl={detection['control_error']:+.3f} "
        f"offset={detection['multi_point_offset']:+.3f} "
        f"head={detection['heading_error']:+.3f} "
        f"confidence={detection['confidence']:.2f}"
    )
    cv2.putText(overlay, status, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(overlay, status, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (245, 245, 245), 1, cv2.LINE_AA)
    return overlay


def main():
    parser = argparse.ArgumentParser(description="Run lane detector on one local image.")
    parser.add_argument("--image", required=True, help="Input image path.")
    parser.add_argument("--output", default="/tmp/line_detector_result.png", help="Output visualization path.")
    parser.add_argument("--detector", choices=("traditional", "ufldv2"), default="ufldv2")
    parser.add_argument("--model", default="/home/hwc/下载/ufldv2_tusimple_res18_320x800.onnx")
    parser.add_argument("--dataset", default="auto", help="UFLDv2 preset: auto, tusimple, culane, curvelanes.")
    parser.add_argument("--crop", type=parse_crop, default=None, help="Optional crop x,y,w,h before detection.")
    args = parser.parse_args()

    image = cv2.imread(args.image)
    if image is None:
        raise SystemExit(f"failed to read image: {args.image}")

    if args.crop is not None:
        x, y, w, h = args.crop
        image = image[y : y + h, x : x + w]
        if image.size == 0:
            raise SystemExit(f"empty crop: {args.crop}")

    if args.detector == "ufldv2":
        detector = UFLDv2LineDetector(args.model, image.shape[1], image.shape[0], dataset=args.dataset)
    else:
        detector = TraditionalLineDetector(image.shape[1], image.shape[0])

    detection = detector.detect(image)
    result = draw_detection(image, detection)
    cv2.imwrite(args.output, result)
    print(f"output: {args.output}")
    print(f"image: {image.shape[1]}x{image.shape[0]}")
    print(f"detector: {args.detector}")
    print(f"confidence: {detection['confidence']:.3f}")
    print(f"offset: {detection['multi_point_offset']:+.3f}")
    print(f"heading: {detection['heading_error']:+.3f}")
    print(f"control_error: {detection['control_error']:+.3f}")
    print(f"candidate_curves: {len(detection['candidate_curves'])}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import argparse
import os
import time

import cv2
import numpy as np
import onnxruntime as ort


IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def _static_dim(value, fallback):
    if isinstance(value, int) and value > 0:
        return value
    return fallback


def preprocess_bgr(frame, width, height):
    image = cv2.resize(frame, (width, height), interpolation=cv2.INTER_LINEAR)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    image = (image - IMAGENET_MEAN) / IMAGENET_STD
    image = np.transpose(image, (2, 0, 1))[None, ...]
    return np.ascontiguousarray(image, dtype=np.float32)


def main():
    parser = argparse.ArgumentParser(description="Quick UFLD v2 ONNX smoke/perf test.")
    parser.add_argument("--model", required=True, help="Path to ufldv2_*.onnx")
    parser.add_argument(
        "--video",
        default="third_party/Ultra-Fast-Lane-Detection-v2/example.mp4",
        help="Video/image stream used for testing.",
    )
    parser.add_argument("--frames", type=int, default=200)
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=320)
    args = parser.parse_args()

    if not os.path.exists(args.model):
        raise SystemExit(f"model file not found: {args.model}")

    sess = ort.InferenceSession(args.model, providers=["CPUExecutionProvider"])
    input_meta = sess.get_inputs()[0]
    input_name = input_meta.name
    input_shape = input_meta.shape
    height = _static_dim(input_shape[2] if len(input_shape) >= 4 else None, args.height)
    width = _static_dim(input_shape[3] if len(input_shape) >= 4 else None, args.width)

    print(f"model: {args.model}")
    print(f"input: name={input_name} shape={input_shape} using={height}x{width}")
    print("outputs:")
    for output in sess.get_outputs():
        print(f"  {output.name}: {output.shape}")

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise RuntimeError(f"failed to open video: {args.video}")

    inference_times = []
    frames = 0
    last_outputs = None
    while frames < args.frames:
        ok, frame = cap.read()
        if not ok:
            break
        tensor = preprocess_bgr(frame, width, height)
        start = time.perf_counter()
        last_outputs = sess.run(None, {input_name: tensor})
        inference_times.append((time.perf_counter() - start) * 1000.0)
        frames += 1

    if frames == 0:
        raise RuntimeError("no frames were processed")

    print(f"frames: {frames}")
    print(f"avg_infer_ms: {np.mean(inference_times):.2f}")
    print(f"p95_infer_ms: {np.percentile(inference_times, 95):.2f}")
    print(f"fps: {1000.0 / np.mean(inference_times):.2f}")
    print("last output tensors:")
    for output_meta, output_value in zip(sess.get_outputs(), last_outputs):
        print(f"  {output_meta.name}: shape={output_value.shape} dtype={output_value.dtype}")


if __name__ == "__main__":
    main()

"""Low-latency, temporal-stable visual line detector for high-speed tracking."""

from __future__ import annotations

from dataclasses import dataclass
import argparse
import time

import cv2
import numpy as np


@dataclass(frozen=True)
class FastLineConfig:
    line_color: str = "red"
    roi_top_ratio: float = 0.28
    roi_bottom_ratio: float = 0.98
    row_count: int = 18
    red_hue_low: int = 0
    red_hue_high: int = 12
    red_hue_wrap_low: int = 168
    min_saturation: int = 55
    min_value: int = 35
    min_run_pixels: int = 3
    morph_kernel: int = 3
    fit_degree: int = 2
    temporal_alpha: float = 0.55
    prediction_gain: float = 0.08
    max_jump_norm: float = 0.22
    lookahead_ratio: float = 0.70
    heading_span_ratio: float = 0.30
    max_lost_frames: int = 8


class FastLineDetector:
    """Fast centerline detector with bounded temporal prediction.

    The detector expects a BGR image. It is intentionally stateless with
    respect to ROS and returns scalar errors that can be published directly
    by an existing controller.
    """

    def __init__(self, config: FastLineConfig | None = None) -> None:
        self.config = config or FastLineConfig()
        self._curve: np.ndarray | None = None
        self._curve_velocity = np.zeros(3, dtype=np.float32)
        self._last_control = 0.0
        self._last_time = 0.0
        self._lost_frames = 0
        self._last_shape: tuple[int, int] | None = None

    def reset(self) -> None:
        self._curve = None
        self._curve_velocity[:] = 0.0
        self._last_control = 0.0
        self._last_time = 0.0
        self._lost_frames = 0

    def detect(self, image: np.ndarray) -> dict[str, object]:
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("FastLineDetector expects a BGR HxWx3 image")
        height, width = image.shape[:2]
        self._last_shape = (height, width)
        now = time.monotonic()
        dt = 1.0 / 30.0 if self._last_time == 0.0 else float(np.clip(now - self._last_time, 1e-3, 0.2))
        self._last_time = now

        mask = self._make_line_mask(image)
        roi_top = int(height * self.config.roi_top_ratio)
        roi_bottom = int(height * self.config.roi_bottom_ratio)
        rows = np.linspace(roi_top, roi_bottom, self.config.row_count, dtype=np.int32)
        observations = self._scan_rows(mask, rows, width)
        raw_curve = self._fit_curve(observations, height, width)

        predicted_curve = None if self._curve is None else self._curve + self._curve_velocity * dt
        if raw_curve is None:
            self._lost_frames += 1
            if predicted_curve is not None and self._lost_frames <= self.config.max_lost_frames:
                self._curve = predicted_curve
            confidence = max(0.0, 0.45 * (1.0 - self._lost_frames / (self.config.max_lost_frames + 1.0)))
            detected = False
        else:
            self._lost_frames = 0
            if predicted_curve is not None:
                delta = np.clip(raw_curve - predicted_curve, -self.config.max_jump_norm, self.config.max_jump_norm)
                raw_curve = predicted_curve + delta
            previous = self._curve if self._curve is not None else raw_curve
            alpha = float(np.clip(self.config.temporal_alpha, 0.05, 1.0))
            filtered = previous + alpha * (raw_curve - previous)
            self._curve_velocity = (filtered - previous) / dt
            self._curve = filtered
            fit_error = self._curve_fit_error(observations, filtered, height, width)
            coverage = min(1.0, len(observations) / max(5.0, self.config.row_count * 0.65))
            confidence = float(np.clip(coverage * np.exp(-5.0 * fit_error), 0.0, 1.0))
            detected = True

        if self._curve is None:
            center = width * 0.5
            heading = 0.0
            control = self._last_control * 0.85
            curve_points = np.empty((0, 2), dtype=np.float32)
        else:
            center, heading, control, curve_points = self._control_from_curve(self._curve, height, width)
            if not detected:
                control = 0.85 * self._last_control + 0.15 * control
            max_control = 0.35 + 0.65 * confidence
            control = float(np.clip(control, -max_control, max_control))
            self._last_control = control

        return {
            "control_error": float(control),
            "offset_norm": float(np.clip((center - width * 0.5) / max(width * 0.5, 1.0), -1.0, 1.0)),
            "heading_error": float(np.clip(heading, -1.0, 1.0)),
            "lane_center": float(center),
            "confidence": float(confidence),
            "detected": detected,
            "lost_frames": self._lost_frames,
            "points": np.asarray(observations, dtype=np.float32),
            "curve_points": curve_points,
            "mask": mask,
            "roi_top": roi_top,
            "roi_bottom": roi_bottom,
        }

    def _make_line_mask(self, image: np.ndarray) -> np.ndarray:
        if self.config.line_color.lower() == "white":
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mask = np.where(
                (gray >= self.config.min_value)
                & (hsv[:, :, 1] <= 90)
                & (gray >= np.percentile(gray, 65)),
                255,
                0,
            ).astype(np.uint8)
        elif self.config.line_color.lower() == "auto":
            red = self._make_red_mask(image)
            white = self._make_white_mask(image)
            mask = red if int(np.count_nonzero(red)) >= int(np.count_nonzero(white)) else white
        else:
            mask = self._make_red_mask(image)
        return self._clean_mask(mask)

    def _make_red_mask(self, image: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        low = cv2.inRange(
            hsv,
            (self.config.red_hue_low, self.config.min_saturation, self.config.min_value),
            (self.config.red_hue_high, 255, 255),
        )
        high = cv2.inRange(
            hsv,
            (self.config.red_hue_wrap_low, self.config.min_saturation, self.config.min_value),
            (179, 255, 255),
        )
        mask = cv2.bitwise_or(low, high)
        return self._clean_mask(mask)

    def _make_white_mask(self, image: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        threshold = max(self.config.min_value, int(np.percentile(gray, 65)))
        return np.where(
            (gray >= threshold) & (hsv[:, :, 1] <= 90), 255, 0
        ).astype(np.uint8)

    def _clean_mask(self, mask: np.ndarray) -> np.ndarray:
        kernel_size = max(1, int(self.config.morph_kernel))
        if kernel_size > 1:
            kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _scan_rows(self, mask: np.ndarray, rows: np.ndarray, width: int) -> list[tuple[float, float]]:
        observations: list[tuple[float, float]] = []
        expected = width * 0.5 if self._curve is None else self._curve[0] * width + width * 0.5
        for row in rows:
            band = mask[max(0, row - 1):min(mask.shape[0], row + 2)]
            signal = np.mean(band > 0, axis=0) > 0.35
            transitions = np.diff(np.pad(signal.astype(np.uint8), (1, 1)))
            starts = np.flatnonzero(transitions == 1)
            ends = np.flatnonzero(transitions == 255) - 1
            if len(starts) == 0 or len(ends) == 0:
                continue
            candidates = [(int(s), int(e)) for s, e in zip(starts, ends) if e - s + 1 >= self.config.min_run_pixels]
            if not candidates:
                continue
            start, end = min(candidates, key=lambda pair: abs(0.5 * (pair[0] + pair[1]) - expected))
            center = 0.5 * (start + end)
            observations.append((center, float(row)))
            if self._curve is not None:
                expected = self._curve[0] * width + width * 0.5
        return observations

    def _fit_curve(self, observations: list[tuple[float, float]], height: int, width: int) -> np.ndarray | None:
        if len(observations) < max(4, self.config.fit_degree + 2):
            return None
        points = np.asarray(observations, dtype=np.float32)
        y = (points[:, 1] - height * 0.5) / max(height * 0.5, 1.0)
        x = (points[:, 0] - width * 0.5) / max(width * 0.5, 1.0)
        coeff = np.polyfit(y, x, self.config.fit_degree)
        if not np.all(np.isfinite(coeff)):
            return None
        return coeff.astype(np.float32)

    def _curve_fit_error(self, observations: list[tuple[float, float]], curve: np.ndarray, height: int, width: int) -> float:
        points = np.asarray(observations, dtype=np.float32)
        y = (points[:, 1] - height * 0.5) / max(height * 0.5, 1.0)
        x = (points[:, 0] - width * 0.5) / max(width * 0.5, 1.0)
        return float(np.mean(np.abs(x - np.polyval(curve, y))))

    def _control_from_curve(self, curve: np.ndarray, height: int, width: int):
        look_y = 2.0 * self.config.lookahead_ratio - 1.0
        near_y = look_y + self.config.heading_span_ratio
        far_y = look_y - self.config.heading_span_ratio
        look_x = float(np.polyval(curve, look_y))
        near_x = float(np.polyval(curve, near_y))
        far_x = float(np.polyval(curve, far_y))
        heading = float(np.clip(np.arctan2(near_x - far_x, near_y - far_y), -1.0, 1.0))
        control = float(np.clip(0.72 * look_x + 0.28 * heading, -1.0, 1.0))
        ys = np.linspace(-1.0, 1.0, 16)
        xs = np.polyval(curve, ys)
        curve_points = np.column_stack((xs * width * 0.5 + width * 0.5, ys * height * 0.5 + height * 0.5))
        return look_x * width * 0.5 + width * 0.5, heading, control, curve_points.astype(np.float32)


def main() -> None:
    parser = argparse.ArgumentParser(description="Fast multi-row red-track detector")
    parser.add_argument("--rows", type=int, default=18)
    parser.add_argument("--alpha", type=float, default=0.55)
    args = parser.parse_args()
    detector = FastLineDetector(FastLineConfig(row_count=args.rows, temporal_alpha=args.alpha))
    print(f"ready: rows={detector.config.row_count} temporal_alpha={detector.config.temporal_alpha}")

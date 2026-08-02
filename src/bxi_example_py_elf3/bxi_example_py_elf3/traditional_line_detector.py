import cv2
import numpy as np


class TraditionalLineDetector:
    """Temporal multi-line runway detector with locked lane boundaries."""

    def __init__(self, image_width=640, image_height=360):
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.lane_width = self.image_width * 0.32
        self.left_curve = None
        self.right_curve = None
        self.center_curve = None
        self.last_center = self.image_width * 0.5
        self.last_heading = 0.0
        self.last_control = 0.0
        self.lost_frames = 0
        self.partial_detection = False
        self.initialized = False
        self.max_boundary_jump = 0.055
        self.max_center_jump = 0.035
        self.max_width_change = 0.22
        self.history_alpha = 0.22

    def detect(self, image):
        height, width = image.shape[:2]
        self.image_width = width
        self.image_height = height
        roi_top = int(height * 0.38)
        roi_bottom = int(height * 0.98)
        mask = self.make_line_mask(image, roi_top, roi_bottom)
        candidates = self.detect_line_candidates(mask, roi_top, roi_bottom)
        fit = self.select_lane(candidates, width, height, roi_top, roi_bottom)
        debug = self.make_debug_image(image, mask, candidates, fit, roi_top, roi_bottom)

        center_curve = fit["center_curve"]
        near_y = int(height * 0.80)
        far_y = int(height * 0.56)
        near_center = float(np.polyval(center_curve, near_y))
        far_center = float(np.polyval(center_curve, far_y))
        slope = (near_center - far_center) / max(float(near_y - far_y), 1.0)
        offset = (near_center - width * 0.5) / max(width * 0.5, 1.0)
        heading = float(np.clip(slope * 5.0, -1.0, 1.0))
        control = float(np.clip(0.72 * offset + 0.48 * heading, -1.0, 1.0))
        self.last_center = self.rate_limit(self.last_center, near_center, width * 0.08)
        self.last_heading = 0.72 * self.last_heading + 0.28 * heading
        self.last_control = 0.72 * self.last_control + 0.28 * control

        left_near = float(np.polyval(fit["left_curve"], near_y))
        right_near = float(np.polyval(fit["right_curve"], near_y))
        half_width = max((right_near - left_near) * 0.5, 1.0)
        robot_center = width * 0.5
        margin = min(robot_center - left_near, right_near - robot_center) / half_width
        mode = 0.0
        if self.lost_frames > 0 or self.partial_detection:
            mode = 3.0
        elif margin < 0.18:
            mode = 2.0
        elif margin < 0.35:
            mode = 1.0

        confidence = fit["confidence"]
        if self.lost_frames > 0:
            confidence *= max(0.0, 1.0 - 0.13 * self.lost_frames)

        return {
            "roi_top": roi_top,
            "roi_bottom": roi_bottom,
            "roi_left": 0,
            "roi_right": width - 1,
            "left_points": self.curve_points(fit["left_curve"], roi_top, roi_bottom),
            "right_points": self.curve_points(fit["right_curve"], roi_top, roi_bottom),
            "candidate_curves": candidates,
            "left_curve_points": self.curve_points(fit["left_curve"], roi_top, roi_bottom),
            "right_curve_points": self.curve_points(fit["right_curve"], roi_top, roi_bottom),
            "center_curve_points": self.curve_points(center_curve, roi_top, roi_bottom),
            "lookahead_y": near_y,
            "raw_lane_center": near_center,
            "lane_center": self.last_center,
            "target_point": (int(np.clip(self.last_center, 0, width - 1)), near_y),
            "offset_norm": float(np.clip((self.last_center - width * 0.5) / max(width * 0.5, 1.0), -1.0, 1.0)),
            "multi_point_offset": float(np.clip(self.last_control, -1.0, 1.0)),
            "heading_error": float(np.clip(self.last_heading, -1.0, 1.0)),
            "control_error": float(np.clip(self.last_control, -1.0, 1.0)),
            "boundary_margin": float(np.clip(margin, -1.0, 1.0)),
            "lane_width_norm": float(np.clip((right_near - left_near) / max(width, 1), 0.0, 2.0)),
            "mode": mode,
            "confidence": float(np.clip(confidence, 0.0, 1.0)),
            "debug_mask": debug,
        }

    def make_line_mask(self, image, roi_top, roi_bottom):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
        bright = (value > 150) & (saturation < 135)
        local = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            31,
            -5,
        ) > 0
        mask = np.where(bright | (local & (saturation < 155) & (value > 110)), 255, 0).astype(np.uint8)
        mask[:roi_top, :] = 0
        mask[roi_bottom:, :] = 0
        mask = cv2.medianBlur(mask, 3)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 11)), iterations=1)
        return mask

    def detect_line_candidates(self, mask, roi_top, roi_bottom):
        height, width = mask.shape[:2]
        bands = np.linspace(roi_bottom - 1, roi_top, 14).astype(int)
        observations = []
        for index in range(len(bands) - 1):
            y0, y1 = sorted((int(bands[index + 1]), int(bands[index])))
            y0 = max(roi_top, y0)
            y1 = min(roi_bottom, max(y0 + 1, y1))
            band = mask[y0:y1]
            score = np.mean(band, axis=0) / 255.0
            score = cv2.GaussianBlur(score.reshape(1, -1).astype(np.float32), (1, 0), 0).ravel()
            threshold = max(0.10, float(np.percentile(score, 78)))
            runs = self.runs(score > threshold)
            points = []
            for start, end in runs:
                if end - start < max(2, int(width * 0.006)):
                    continue
                weights = score[start:end] + 1.0e-3
                x = float(np.average(np.arange(start, end), weights=weights))
                points.append((x, float((y0 + y1) * 0.5), float(score[start:end].mean())))
            observations.append(points)

        tracks = []
        for points in observations:
            used = set()
            for track in tracks:
                expected = track[-1][0]
                choices = [(abs(point[0] - expected), idx, point) for idx, point in enumerate(points) if idx not in used]
                if not choices:
                    continue
                distance, idx, point = min(choices)
                limit = max(width * 0.075, abs(track[-1][0] - track[-2][0]) * 2.5 if len(track) > 1 else width * 0.04)
                if distance <= limit:
                    track.append(point)
                    used.add(idx)
            for idx, point in enumerate(points):
                if idx not in used:
                    tracks.append([point])

        curves = []
        for track in tracks:
            if len(track) < 4:
                continue
            points = np.asarray(track, dtype=np.float64)
            degree = 2 if len(track) >= 7 else 1
            coeff = np.polyfit(points[:, 1], points[:, 0], degree)
            if degree == 1:
                coeff = np.asarray([0.0, coeff[0], coeff[1]])
            curves.append({"coeff": coeff, "points": points, "quality": self.track_quality(points, width)})
        return curves

    def select_lane(self, candidates, width, height, roi_top, roi_bottom):
        self.partial_detection = False
        if self.lost_frames >= 12:
            self.initialized = False
            self.left_curve = None
            self.right_curve = None
            self.center_curve = None
        if not candidates:
            self.lost_frames += 1
            return self.history_fit(width, roi_top, roi_bottom, 0.0)

        near_y = float(height * 0.80)
        evaluated = []
        relaxed_evaluated = []
        for left_index, left in enumerate(candidates):
            left_x = float(np.polyval(left["coeff"], near_y))
            for right_index, right in enumerate(candidates):
                if left_index == right_index:
                    continue
                right_x = float(np.polyval(right["coeff"], near_y))
                if right_x <= left_x:
                    continue
                lane_width = right_x - left_x
                if lane_width < width * 0.08 or lane_width > width * 0.98:
                    continue
                center = (left_x + right_x) * 0.5
                score = left["quality"] + right["quality"]
                score -= abs(lane_width - self.lane_width) / max(width * 0.25, 1.0)
                score -= 0.35 * abs(center - width * 0.5) / max(width * 0.5, 1.0)
                if self.initialized:
                    previous_left = float(np.polyval(self.left_curve, near_y))
                    previous_right = float(np.polyval(self.right_curve, near_y))
                    previous_center = 0.5 * (previous_left + previous_right)
                    center_jump = abs(center - previous_center) / max(width, 1.0)
                    width_jump = abs(lane_width - self.lane_width) / max(self.lane_width, 1.0)
                    if width_jump <= 0.24 and center_jump <= 0.18:
                        relaxed_score = (
                            left["quality"]
                            + right["quality"]
                            - 5.0 * center_jump
                            - 1.2 * width_jump
                        )
                        relaxed_evaluated.append(
                            (relaxed_score, left, right, lane_width)
                        )
                    if abs(left_x - previous_left) > width * self.max_boundary_jump:
                        continue
                    if abs(right_x - previous_right) > width * self.max_boundary_jump:
                        continue
                    if abs(center - previous_center) > width * self.max_center_jump:
                        continue
                    score -= abs(left_x - previous_left) / max(width * self.max_boundary_jump, 1.0)
                    score -= abs(right_x - previous_right) / max(width * self.max_boundary_jump, 1.0)
                    score -= 1.5 * abs(center - previous_center) / max(width * self.max_center_jump, 1.0)
                else:
                    score -= abs(center - width * 0.5) / max(width * 0.5, 1.0)
                evaluated.append((score, left, right, lane_width))

        if not evaluated and self.initialized and len(candidates) == 1:
            single = candidates[0]
            near_x = float(np.polyval(single["coeff"], near_y))
            previous_left = float(np.polyval(self.left_curve, near_y))
            previous_right = float(np.polyval(self.right_curve, near_y))
            if abs(near_x - previous_left) <= width * self.max_boundary_jump:
                left_coeff = single["coeff"]
                right_coeff = self.right_curve + (left_coeff - self.left_curve)
            elif abs(near_x - previous_right) <= width * self.max_boundary_jump:
                right_coeff = single["coeff"]
                left_coeff = self.left_curve + (right_coeff - self.right_curve)
            else:
                left_coeff = None
                right_coeff = None
            if left_coeff is not None and right_coeff is not None:
                # A single visible boundary is not enough to move the lane
                # center. Keep the last complete-lane estimate until both
                # boundaries are observed again.
                self.lost_frames = 0
                self.partial_detection = True
                return {
                    "left_curve": self.left_curve,
                    "right_curve": self.right_curve,
                    "center_curve": self.center_curve,
                    "confidence": 0.18 * single["quality"],
                }

        if not evaluated and self.initialized and relaxed_evaluated:
            _score, left, right, lane_width = max(
                relaxed_evaluated, key=lambda item: item[0]
            )
            left_coeff = self.smooth_coeff(self.left_curve, left["coeff"], 0.12)
            right_coeff = self.smooth_coeff(self.right_curve, right["coeff"], 0.12)
            self.left_curve = left_coeff
            self.right_curve = right_coeff
            self.center_curve = (left_coeff + right_coeff) * 0.5
            self.lane_width = 0.85 * self.lane_width + 0.15 * lane_width
            self.lost_frames = 0
            return {
                "left_curve": left_coeff,
                "right_curve": right_coeff,
                "center_curve": self.center_curve,
                "confidence": 0.38 * min(left["quality"], right["quality"]),
            }

        if not evaluated:
            self.lost_frames += 1
            return self.history_fit(width, roi_top, roi_bottom, 0.0)

        _score, left, right, lane_width = max(evaluated, key=lambda item: item[0])
        left_coeff = left["coeff"]
        right_coeff = right["coeff"]
        if self.initialized:
            old_width = max(self.lane_width, 1.0)
            width_change = abs(lane_width - old_width) / old_width
            if width_change > self.max_width_change:
                self.lost_frames += 1
                return self.history_fit(width, roi_top, roi_bottom, 0.0)
            left_coeff = self.smooth_coeff(self.left_curve, left_coeff, self.history_alpha)
            right_coeff = self.smooth_coeff(self.right_curve, right_coeff, self.history_alpha)
        self.left_curve = left_coeff
        self.right_curve = right_coeff
        self.center_curve = (left_coeff + right_coeff) * 0.5
        if not self.initialized:
            self.lane_width = lane_width
        else:
            self.lane_width = 0.82 * self.lane_width + 0.18 * lane_width
        self.initialized = True
        self.lost_frames = 0
        quality = min(1.0, 0.5 * left["quality"] + 0.5 * right["quality"])
        return {"left_curve": left_coeff, "right_curve": right_coeff, "center_curve": self.center_curve, "confidence": quality}

    def history_fit(self, width, roi_top, roi_bottom, confidence):
        if self.left_curve is None or self.right_curve is None:
            left = np.asarray([0.0, 0.0, width * 0.5 - self.lane_width * 0.5])
            right = np.asarray([0.0, 0.0, width * 0.5 + self.lane_width * 0.5])
            center = (left + right) * 0.5
            return {"left_curve": left, "right_curve": right, "center_curve": center, "confidence": confidence}
        return {"left_curve": self.left_curve, "right_curve": self.right_curve, "center_curve": self.center_curve, "confidence": confidence}

    def track_quality(self, points, width):
        coverage = min(1.0, len(points) / 10.0)
        vertical = min(1.0, (points[:, 1].max() - points[:, 1].min()) / max(self.image_height * 0.45, 1.0))
        density = min(1.0, float(points[:, 2].mean()) * 2.0)
        return float(np.clip(0.40 * coverage + 0.40 * vertical + 0.20 * density, 0.0, 1.0))

    def smooth_coeff(self, old, new, alpha):
        return (1.0 - alpha) * old + alpha * new

    def rate_limit(self, old, new, limit):
        return old + float(np.clip(new - old, -limit, limit))

    def curve_points(self, coeff, top, bottom):
        ys = np.linspace(top, bottom, 16)
        return [(int(np.clip(np.polyval(coeff, y), 0, self.image_width - 1)), int(y)) for y in ys]

    def runs(self, values):
        padded = np.concatenate(([False], values, [False]))
        changes = np.flatnonzero(padded[1:] != padded[:-1])
        return list(zip(changes[::2], changes[1::2]))

    def make_debug_image(self, image, mask, candidates, fit, roi_top, roi_bottom):
        debug = image.copy()
        debug[mask > 0] = (80, 180, 80)
        for candidate in candidates:
            points = candidate["points"]
            for x, y, _score in points:
                cv2.circle(debug, (int(x), int(y)), 3, (0, 180, 255), -1)
        for coeff, color in ((fit["left_curve"], (0, 0, 255)), (fit["right_curve"], (255, 0, 0)), (fit["center_curve"], (0, 255, 255))):
            points = self.curve_points(coeff, roi_top, roi_bottom)
            for first, second in zip(points[:-1], points[1:]):
                cv2.line(debug, first, second, color, 3)
        cv2.line(debug, (int(self.last_center), int(self.image_height * 0.80)), (int(self.last_center), self.image_height - 1), (255, 255, 0), 2)
        return debug

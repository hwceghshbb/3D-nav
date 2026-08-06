from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .core import stamp_to_nanoseconds


def quaternion_matrix(quaternion):
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.linalg.norm([x, y, z, w]))
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotation_error_deg(left, right):
    cosine = (float(np.trace(left.T @ right)) - 1.0) * 0.5
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_matrix(rotation):
    return math.atan2(rotation[1, 0], rotation[0, 0])


def pose_from_message(message):
    pose = message.pose.pose
    return (
        np.asarray([pose.position.x, pose.position.y, pose.position.z]),
        quaternion_matrix(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        ),
    )


def percentile(values, quantile):
    return float(np.percentile(np.asarray(values), quantile)) if values else None


class LocalizationBenchmark(Node):
    def __init__(self):
        super().__init__("localization_benchmark")
        self.declare_parameter("backend_name", "localization")
        self.declare_parameter("estimate_topic", "/nav/odom")
        self.declare_parameter("truth_topic", "/simulation/odom")
        self.declare_parameter("output_path", "/tmp/localization_benchmark.json")
        self.declare_parameter("max_stamp_delta_ms", 35.0)
        self.declare_parameter("warmup_sec", 3.0)
        self.declare_parameter("report_interval_sec", 5.0)
        self.declare_parameter("max_samples", 20000)

        max_samples = int(self.get_parameter("max_samples").value)
        self.truth = deque(maxlen=max_samples * 2)
        self.samples = deque(maxlen=max_samples)
        self.arrival_intervals_ms = deque(maxlen=max_samples)
        self.last_arrival_ns = None
        self.first_pair_stamp_ns = None
        self.anchor_rotation = None
        self.anchor_translation = None
        self.dropped_stamp_pairs = 0

        self.create_subscription(
            Odometry,
            str(self.get_parameter("truth_topic").value),
            self.truth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("estimate_topic").value),
            self.estimate_callback,
            qos_profile_sensor_data,
        )
        interval = max(
            1.0, float(self.get_parameter("report_interval_sec").value)
        )
        self.create_timer(interval, self.report)
        self.get_logger().info(
            "Benchmarking %s: estimate=%s truth=%s"
            % (
                self.get_parameter("backend_name").value,
                self.get_parameter("estimate_topic").value,
                self.get_parameter("truth_topic").value,
            )
        )

    def truth_callback(self, message):
        self.truth.append(
            (stamp_to_nanoseconds(message.header.stamp), *pose_from_message(message))
        )

    def estimate_callback(self, message):
        if not self.truth:
            return
        stamp_ns = stamp_to_nanoseconds(message.header.stamp)
        truth_stamp_ns, truth_position, truth_rotation = min(
            self.truth, key=lambda item: abs(item[0] - stamp_ns)
        )
        stamp_delta_ms = abs(truth_stamp_ns - stamp_ns) / 1e6
        if stamp_delta_ms > float(self.get_parameter("max_stamp_delta_ms").value):
            self.dropped_stamp_pairs += 1
            return

        now_ns = time.monotonic_ns()
        if self.last_arrival_ns is not None:
            self.arrival_intervals_ms.append((now_ns - self.last_arrival_ns) / 1e6)
        self.last_arrival_ns = now_ns
        if self.first_pair_stamp_ns is None:
            self.first_pair_stamp_ns = stamp_ns

        warmup_ns = int(float(self.get_parameter("warmup_sec").value) * 1e9)
        if stamp_ns - self.first_pair_stamp_ns < warmup_ns:
            return

        estimate_position, estimate_rotation = pose_from_message(message)
        if self.anchor_rotation is None:
            self.anchor_rotation = truth_rotation @ estimate_rotation.T
            self.anchor_translation = (
                truth_position - self.anchor_rotation @ estimate_position
            )
        anchored_position = self.anchor_rotation @ estimate_position + self.anchor_translation
        anchored_rotation = self.anchor_rotation @ estimate_rotation
        self.samples.append(
            {
                "stamp_ns": stamp_ns,
                "stamp_delta_ms": stamp_delta_ms,
                "estimate_position": estimate_position,
                "estimate_rotation": estimate_rotation,
                "truth_position": truth_position,
                "truth_rotation": truth_rotation,
                "anchored_translation_error_m": float(
                    np.linalg.norm(anchored_position - truth_position)
                ),
                "anchored_rotation_error_deg": rotation_error_deg(
                    anchored_rotation, truth_rotation
                ),
            }
        )

    def compute_summary(self):
        received_samples = list(self.samples)
        samples_by_stamp = {}
        out_of_order_samples = 0
        previous_stamp = None
        for sample in received_samples:
            stamp = sample["stamp_ns"]
            if previous_stamp is not None and stamp < previous_stamp:
                out_of_order_samples += 1
            previous_stamp = stamp
            samples_by_stamp[stamp] = sample
        samples = [samples_by_stamp[stamp] for stamp in sorted(samples_by_stamp)]
        intervals = list(self.arrival_intervals_ms)
        summary = {
            "backend": str(self.get_parameter("backend_name").value),
            "estimate_topic": str(self.get_parameter("estimate_topic").value),
            "truth_topic": str(self.get_parameter("truth_topic").value),
            "matched_samples": len(samples),
            "duplicate_stamp_samples": len(received_samples) - len(samples),
            "out_of_order_samples": out_of_order_samples,
            "dropped_stamp_pairs": self.dropped_stamp_pairs,
            "rate_hz_median": (
                1000.0 / percentile(intervals, 50) if intervals else None
            ),
            "rate_hz_p05": (
                1000.0 / percentile(intervals, 95) if intervals else None
            ),
            "stamp_delta_ms_p95": percentile(
                [sample["stamp_delta_ms"] for sample in samples], 95
            ),
        }
        if not samples:
            return summary

        anchored_translation = [
            sample["anchored_translation_error_m"] for sample in samples
        ]
        anchored_rotation = [
            sample["anchored_rotation_error_deg"] for sample in samples
        ]
        anchored_yaw = []
        anchored_tilt = []
        for sample in samples:
            anchored_estimate = self.anchor_rotation @ sample["estimate_rotation"]
            relative_error = sample["truth_rotation"].T @ anchored_estimate
            yaw_error = abs(math.degrees(wrap_angle(yaw_from_matrix(relative_error))))
            anchored_yaw.append(yaw_error)
            anchored_tilt.append(
                math.sqrt(max(0.0, sample["anchored_rotation_error_deg"] ** 2 - yaw_error ** 2))
            )
        summary.update(
            {
                "duration_sec": (
                    samples[-1]["stamp_ns"] - samples[0]["stamp_ns"]
                )
                / 1e9,
                "anchored_translation_rmse_m": float(
                    math.sqrt(np.mean(np.square(anchored_translation)))
                ),
                "anchored_translation_p95_m": percentile(anchored_translation, 95),
                "anchored_translation_max_m": max(anchored_translation),
                "anchored_rotation_rmse_deg": float(
                    math.sqrt(np.mean(np.square(anchored_rotation)))
                ),
                "anchored_rotation_p95_deg": percentile(anchored_rotation, 95),
                "anchored_yaw_rmse_deg": float(
                    math.sqrt(np.mean(np.square(anchored_yaw)))
                ),
                "anchored_yaw_p95_deg": percentile(anchored_yaw, 95),
                "anchored_tilt_rmse_deg": float(
                    math.sqrt(np.mean(np.square(anchored_tilt)))
                ),
            }
        )

        estimate = np.stack([sample["estimate_position"] for sample in samples])
        truth = np.stack([sample["truth_position"] for sample in samples])
        estimate_center = estimate.mean(axis=0)
        truth_center = truth.mean(axis=0)
        covariance = (estimate - estimate_center).T @ (truth - truth_center)
        u_matrix, _, vt_matrix = np.linalg.svd(covariance)
        correction = np.eye(3)
        if np.linalg.det(vt_matrix.T @ u_matrix.T) < 0.0:
            correction[2, 2] = -1.0
        align_rotation = vt_matrix.T @ correction @ u_matrix.T
        align_translation = truth_center - align_rotation @ estimate_center
        aligned = (align_rotation @ estimate.T).T + align_translation
        ate = np.linalg.norm(aligned - truth, axis=1)
        summary["se3_ate_rmse_m"] = float(math.sqrt(np.mean(np.square(ate))))
        summary["se3_ate_p95_m"] = percentile(ate.tolist(), 95)
        truth_steps = np.linalg.norm(np.diff(truth, axis=0), axis=1)
        estimate_steps = np.linalg.norm(np.diff(estimate, axis=0), axis=1)
        jump_threshold = np.maximum(0.25, truth_steps + 0.20)
        jump_mask = estimate_steps > jump_threshold
        summary["translation_jump_count"] = int(np.count_nonzero(jump_mask))
        summary["translation_step_p95_m"] = percentile(estimate_steps.tolist(), 95)
        summary["translation_step_max_m"] = float(estimate_steps.max())
        summary["path_length_truth_m"] = float(truth_steps.sum())
        summary["path_length_estimate_m"] = float(estimate_steps.sum())
        summary["path_length_estimate_without_jumps_m"] = float(
            estimate_steps[~jump_mask].sum()
        )

        estimate_rotations = [sample["estimate_rotation"] for sample in samples]
        truth_rotations = [sample["truth_rotation"] for sample in samples]
        relative_translation_errors = []
        relative_rotation_errors = []
        relative_yaw_errors = []
        for index in range(1, len(samples)):
            estimate_delta = estimate_rotations[index - 1].T @ (
                estimate[index] - estimate[index - 1]
            )
            truth_delta = truth_rotations[index - 1].T @ (
                truth[index] - truth[index - 1]
            )
            relative_translation_errors.append(
                float(np.linalg.norm(estimate_delta - truth_delta))
            )
            estimate_delta_rotation = (
                estimate_rotations[index - 1].T @ estimate_rotations[index]
            )
            truth_delta_rotation = truth_rotations[index - 1].T @ truth_rotations[index]
            rotation_error = truth_delta_rotation.T @ estimate_delta_rotation
            relative_rotation_errors.append(rotation_error_deg(np.eye(3), rotation_error))
            relative_yaw_errors.append(
                abs(math.degrees(wrap_angle(yaw_from_matrix(rotation_error))))
            )
        summary["frame_rpe_translation_rmse_m"] = float(
            math.sqrt(np.mean(np.square(relative_translation_errors)))
        )
        summary["frame_rpe_rotation_rmse_deg"] = float(
            math.sqrt(np.mean(np.square(relative_rotation_errors)))
        )
        summary["frame_rpe_yaw_rmse_deg"] = float(
            math.sqrt(np.mean(np.square(relative_yaw_errors)))
        )
        return summary

    def report(self):
        summary = self.compute_summary()
        self.get_logger().info(
            "%s benchmark: samples=%d rate=%sHz anchored_rmse=%sm ATE=%sm"
            % (
                summary["backend"],
                summary["matched_samples"],
                "%.1f" % summary["rate_hz_median"]
                if summary["rate_hz_median"] is not None
                else "n/a",
                "%.3f" % summary.get("anchored_translation_rmse_m", math.nan),
                "%.3f" % summary.get("se3_ate_rmse_m", math.nan),
            )
        )
        self.write_summary(summary)

    def write_summary(self, summary=None):
        summary = summary or self.compute_summary()
        summary["written_at_utc"] = datetime.now(timezone.utc).isoformat()
        path = Path(str(self.get_parameter("output_path").value)).expanduser()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(path.suffix + ".tmp")
            temporary.write_text(
                json.dumps(summary, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            temporary.replace(path)
        except OSError as error:
            self.get_logger().error(f"Cannot write benchmark result: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationBenchmark()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.write_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

from __future__ import annotations

from array import array as byte_array
from collections import deque
from collections.abc import Mapping
import math
from threading import Lock
import time

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu

from bxi_example_py_elf3.mod_api import NodeBuildContext


_DEFAULTS: dict[str, object] = {
    "serial": "",
    "depth_w": 480,
    "depth_h": 270,
    "depth_fps": 60,
    "color_w": 848,
    "color_h": 480,
    "color_fps": 30,
    "enable_color": False,
    "enable_ir": False,
    "enable_imu": False,
    "publish_rate_hz": 60,
    "out_w": 64,
    "out_h": 36,
    "hfov": 89.24,
    "vfov": 58.06,
    "publish_full": True,
    "publish_origin_camera": True,
    "origin_out_w": 36,
    "origin_out_h": 48,
    "origin_hfov": 45.2,
    "origin_vfov": 58.0616969,
    "origin_min_dist": 0.2,
    "origin_max_dist": 3.0,
    "decimation": 1,
    "min_dist": 0.2,
    "max_dist": 2.5,
    "spat_alpha": 0.45,
    "spat_delta": 20.0,
    "spat_holes": 2,
    "temp_holes": 4,
    "temp_alpha": 0.45,
    "temp_delta": 20.0,
    "hole1": 1,
    "hole2": 2,
    "frame_depth": "camera_depth_optical_frame",
    "frame_color": "camera_color_optical_frame",
    "topic_depth": "/camera/depth/image_raw",
    "topic_depth_info": "/camera/depth/camera_info",
    "topic_small": "/camera/depth/image_64x36",
    "topic_small_info": "/camera/depth/camera_info_64x36",
    "topic_origin": "/camera/depth/image_36x48",
    "topic_origin_info": "/camera/depth/camera_info_36x48",
    "topic_color": "/camera/color/image_raw",
    "topic_ir1": "/camera/infra1/image_raw",
    "topic_ir2": "/camera/infra2/image_raw",
    "topic_imu": "/camera/imu",
}


def _round_positive(value: float) -> int:
    return int(math.floor(value + 0.5))


def _clamp_roi(
    width: int, height: int, max_width: int, max_height: int
) -> tuple[int, int]:
    return max(1, min(width, max_width)), max(1, min(height, max_height))


def _choose_fov_crop(
    width: int,
    height: int,
    fx: float,
    fy: float,
    out_width: int,
    out_height: int,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
) -> tuple[int, int, int, int]:
    horizontal_fov = math.radians(horizontal_fov_deg)
    vertical_fov = math.radians(vertical_fov_deg)
    output_ratio = float(out_width) / float(out_height)

    target_width = _round_positive(2.0 * fx * math.tan(horizontal_fov / 2.0))
    target_height = _round_positive(2.0 * fy * math.tan(vertical_fov / 2.0))

    width_a, height_a = _clamp_roi(
        _round_positive(target_height * output_ratio),
        target_height,
        width,
        height,
    )
    width_b, height_b = _clamp_roi(
        target_width,
        _round_positive(target_width / output_ratio),
        width,
        height,
    )

    def fov_error(crop_width: int, crop_height: int) -> float:
        actual_horizontal = 2.0 * math.atan(float(crop_width) / (2.0 * fx))
        actual_vertical = 2.0 * math.atan(float(crop_height) / (2.0 * fy))
        return abs(actual_horizontal - horizontal_fov) + abs(
            actual_vertical - vertical_fov
        )

    if fov_error(width_a, height_a) <= fov_error(width_b, height_b):
        crop_width, crop_height = width_a, height_a
    else:
        crop_width, crop_height = width_b, height_b
    return (
        (width - crop_width) // 2,
        (height - crop_height) // 2,
        crop_width,
        crop_height,
    )


def _resize_nearest(image: np.ndarray, width: int, height: int) -> np.ndarray:
    source_height, source_width = image.shape[:2]
    source_x = np.minimum(
        np.floor(np.arange(width) * (float(source_width) / float(width))).astype(
            np.intp
        ),
        source_width - 1,
    )
    source_y = np.minimum(
        np.floor(np.arange(height) * (float(source_height) / float(height))).astype(
            np.intp
        ),
        source_height - 1,
    )
    return np.ascontiguousarray(image[np.ix_(source_y, source_x)])


def _clamp_depth(image: np.ndarray, minimum: int, maximum: int) -> np.ndarray:
    result = np.ascontiguousarray(image, dtype=np.uint16).copy()
    valid = result != 0
    result[valid] = np.clip(result[valid], minimum, maximum)
    return result


class RealSenseDepthPublisher(Node):
    def __init__(
        self,
        node_name: str,
        params: Mapping[str, object],
    ) -> None:
        super().__init__(node_name)
        unknown = set(params) - set(_DEFAULTS)
        if unknown:
            super().destroy_node()
            raise ValueError(f"unknown RealSense node params: {sorted(unknown)}")

        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            super().destroy_node()
            raise RuntimeError(
                "pyrealsense2 is required by com.bxi.normal_depth; "
                "install the Python RealSense bindings"
            ) from exc

        self._rs = rs
        self._pipeline = rs.pipeline()
        self._pipeline_started = False
        self._frame_lock = Lock()
        self._latest_frameset = None
        self._motion_samples: deque[tuple[object, float, float, float]] = deque(
            maxlen=2048
        )
        self._last_log_times: dict[str, float] = {}

        try:
            self._read_parameters(params)
            self._create_publishers()
            profile = self._start_pipeline()
            self._initialize_depth_processing(profile)
            self._timer = self.create_timer(
                1.0 / float(self.publish_rate_hz),
                self._publish_latest,
            )
        except BaseException:
            self._stop_pipeline()
            super().destroy_node()
            raise

    def _read_parameters(self, params: Mapping[str, object]) -> None:
        for name, default in _DEFAULTS.items():
            self.declare_parameter(name, params.get(name, default))
            value = self.get_parameter(name).value
            if isinstance(default, bool):
                if not isinstance(value, bool):
                    raise ValueError(f"RealSense param '{name}' must be a boolean")
            elif isinstance(default, int):
                if isinstance(value, bool) or not isinstance(value, int):
                    raise ValueError(f"RealSense param '{name}' must be an integer")
            elif isinstance(default, float):
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise ValueError(f"RealSense param '{name}' must be a number")
                value = float(value)
            elif not isinstance(value, str):
                raise ValueError(f"RealSense param '{name}' must be a string")
            setattr(self, name, value)

        for name in (
            "depth_w",
            "depth_h",
            "depth_fps",
            "color_w",
            "color_h",
            "color_fps",
            "publish_rate_hz",
            "out_w",
            "out_h",
            "origin_out_w",
            "origin_out_h",
            "decimation",
        ):
            if getattr(self, name) <= 0:
                raise ValueError(f"RealSense param '{name}' must be greater than zero")
        for minimum_name, maximum_name in (
            ("min_dist", "max_dist"),
            ("origin_min_dist", "origin_max_dist"),
        ):
            if getattr(self, minimum_name) < 0.0 or getattr(self, maximum_name) <= 0.0:
                raise ValueError(
                    f"RealSense params '{minimum_name}/{maximum_name}' are invalid"
                )

    def _create_publishers(self) -> None:
        self._pub_depth = self.create_publisher(Image, self.topic_depth, 1)
        self._pub_depth_info = self.create_publisher(
            CameraInfo, self.topic_depth_info, 1
        )
        self._pub_small = self.create_publisher(Image, self.topic_small, 1)
        self._pub_small_info = self.create_publisher(
            CameraInfo, self.topic_small_info, 1
        )
        self._pub_origin = None
        self._pub_origin_info = None
        if self.publish_origin_camera:
            self._pub_origin = self.create_publisher(Image, self.topic_origin, 1)
            self._pub_origin_info = self.create_publisher(
                CameraInfo, self.topic_origin_info, 1
            )

        self._pub_color = (
            self.create_publisher(Image, self.topic_color, 1)
            if self.enable_color
            else None
        )
        self._pub_ir1 = (
            self.create_publisher(Image, self.topic_ir1, 1) if self.enable_ir else None
        )
        self._pub_ir2 = (
            self.create_publisher(Image, self.topic_ir2, 1) if self.enable_ir else None
        )
        self._pub_imu = (
            self.create_publisher(Imu, self.topic_imu, 100) if self.enable_imu else None
        )

    def _start_pipeline(self):
        rs = self._rs
        config = rs.config()
        if self.serial:
            config.enable_device(self.serial)
        config.enable_stream(
            rs.stream.depth,
            self.depth_w,
            self.depth_h,
            rs.format.z16,
            self.depth_fps,
        )
        if self.enable_color:
            config.enable_stream(
                rs.stream.color,
                self.color_w,
                self.color_h,
                rs.format.bgr8,
                self.color_fps,
            )
        if self.enable_ir:
            config.enable_stream(
                rs.stream.infrared,
                1,
                self.depth_w,
                self.depth_h,
                rs.format.y8,
                self.depth_fps,
            )
            config.enable_stream(
                rs.stream.infrared,
                2,
                self.depth_w,
                self.depth_h,
                rs.format.y8,
                self.depth_fps,
            )
        if self.enable_imu:
            config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)
            config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)

        try:
            profile = self._pipeline.start(config, self._on_realsense_frame)
        except Exception as exc:
            raise RuntimeError(f"RealSense start failed: {exc}") from exc
        self._pipeline_started = True
        return profile

    def _initialize_depth_processing(self, profile) -> None:
        rs = self._rs
        device = profile.get_device()
        depth_sensor = device.first_depth_sensor()
        self._depth_scale = float(depth_sensor.get_depth_scale())
        self._min_units, self._max_units = self._distance_units(
            self.min_dist, self.max_dist
        )
        self._origin_min_units, self._origin_max_units = self._distance_units(
            self.origin_min_dist, self.origin_max_dist
        )
        self._configure_device(device)

        depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        self._intrinsics = depth_profile.get_intrinsics()
        self._log_intrinsics(self._intrinsics)

        self._decimation_filter = rs.decimation_filter()
        self._decimation_filter.set_option(
            rs.option.filter_magnitude, float(self.decimation)
        )
        self._spatial_filter = rs.spatial_filter()
        self._spatial_filter.set_option(rs.option.filter_smooth_alpha, self.spat_alpha)
        self._spatial_filter.set_option(rs.option.filter_smooth_delta, self.spat_delta)
        self._spatial_filter.set_option(rs.option.holes_fill, float(self.spat_holes))
        self._temporal_filter = rs.temporal_filter()
        self._temporal_filter.set_option(rs.option.filter_smooth_alpha, self.temp_alpha)
        self._temporal_filter.set_option(rs.option.filter_smooth_delta, self.temp_delta)
        self._temporal_filter.set_option(rs.option.holes_fill, float(self.temp_holes))
        self._hole_filter_1 = rs.hole_filling_filter(int(self.hole1))
        self._hole_filter_2 = rs.hole_filling_filter(int(self.hole2))

    def _distance_units(self, minimum: float, maximum: float) -> tuple[int, int]:
        minimum_units = min(65535, max(0, _round_positive(minimum / self._depth_scale)))
        maximum_units = min(65535, max(1, _round_positive(maximum / self._depth_scale)))
        return (
            (minimum_units, maximum_units)
            if minimum_units <= maximum_units
            else (maximum_units, minimum_units)
        )

    def _configure_device(self, device) -> None:
        rs = self._rs
        for sensor in device.query_sensors():
            try:
                if sensor.supports(rs.option.emitter_enabled):
                    sensor.set_option(rs.option.emitter_enabled, 1.0)
                if sensor.supports(rs.option.laser_power):
                    option_range = sensor.get_option_range(rs.option.laser_power)
                    sensor.set_option(rs.option.laser_power, option_range.max)
                if sensor.supports(rs.option.visual_preset):
                    sensor.set_option(
                        rs.option.visual_preset,
                        float(int(rs.rs400_visual_preset.high_accuracy)),
                    )
            except Exception as exc:
                self.get_logger().warning(
                    f"cannot configure RealSense sensor options: {exc}"
                )

    def _log_intrinsics(self, intrinsics) -> None:
        horizontal_fov = math.degrees(
            2.0 * math.atan(float(intrinsics.width) / (2.0 * intrinsics.fx))
        )
        vertical_fov = math.degrees(
            2.0 * math.atan(float(intrinsics.height) / (2.0 * intrinsics.fy))
        )
        self.get_logger().info(
            "RealSense depth intrinsics: "
            f"size={intrinsics.width}x{intrinsics.height}, "
            f"fx={intrinsics.fx:.3f}, fy={intrinsics.fy:.3f}, "
            f"cx={intrinsics.ppx:.3f}, cy={intrinsics.ppy:.3f}, "
            f"model={intrinsics.model}, HFOV={horizontal_fov:.2f}, "
            f"VFOV={vertical_fov:.2f}"
        )

    def _on_realsense_frame(self, frame) -> None:
        try:
            if frame.is_frameset():
                with self._frame_lock:
                    self._latest_frameset = frame.as_frameset()
                return
            if frame.is_motion_frame():
                motion_frame = frame.as_motion_frame()
                motion = motion_frame.get_motion_data()
                stream_type = motion_frame.get_profile().stream_type()
                with self._frame_lock:
                    self._motion_samples.append(
                        (stream_type, float(motion.x), float(motion.y), float(motion.z))
                    )
        except Exception as exc:
            self._log_throttled("callback", f"RealSense callback failed: {exc}")

    def _publish_latest(self) -> None:
        with self._frame_lock:
            frameset = self._latest_frameset
            self._latest_frameset = None
            motion_samples = tuple(self._motion_samples)
            self._motion_samples.clear()
        if frameset is not None:
            try:
                self._publish_frameset(frameset)
            except Exception as exc:
                self._log_throttled("frameset", f"RealSense frame failed: {exc}")
        if motion_samples and self._pub_imu is not None:
            self._publish_motion(motion_samples)

    def _publish_frameset(self, frameset) -> None:
        stamp = self.get_clock().now().to_msg()
        depth_frame = frameset.get_depth_frame()
        if depth_frame:
            self._publish_depth(depth_frame, stamp)
        if self._pub_color is not None:
            color_frame = frameset.get_color_frame()
            if color_frame:
                color = np.asanyarray(color_frame.get_data())
                self._pub_color.publish(
                    self._image_message(color, "bgr8", self.frame_color, stamp)
                )
        if self._pub_ir1 is not None and self._pub_ir2 is not None:
            ir1_frame = frameset.get_infrared_frame(1)
            ir2_frame = frameset.get_infrared_frame(2)
            if ir1_frame:
                ir1 = np.asanyarray(ir1_frame.get_data())
                self._pub_ir1.publish(
                    self._image_message(ir1, "mono8", self.frame_depth, stamp)
                )
            if ir2_frame:
                ir2 = np.asanyarray(ir2_frame.get_data())
                self._pub_ir2.publish(
                    self._image_message(ir2, "mono8", self.frame_depth, stamp)
                )

    def _publish_depth(self, depth_frame, stamp) -> None:
        frame = depth_frame
        if self.decimation > 1:
            frame = self._decimation_filter.process(frame)
        frame = self._spatial_filter.process(frame)
        frame = self._temporal_filter.process(frame)
        frame = self._hole_filter_1.process(frame)
        frame = self._hole_filter_2.process(frame)
        depth = np.asanyarray(frame.get_data())
        height, width = depth.shape

        intrinsics = self._intrinsics
        fx_full = intrinsics.fx * (float(width) / float(intrinsics.width))
        fy_full = intrinsics.fy * (float(height) / float(intrinsics.height))
        cx_full = intrinsics.ppx * (float(width) / float(intrinsics.width))
        cy_full = intrinsics.ppy * (float(height) / float(intrinsics.height))

        if self.publish_full:
            full = _clamp_depth(depth, self._min_units, self._max_units)
            image = self._image_message(full, "16UC1", self.frame_depth, stamp)
            self._pub_depth.publish(image)
            self._pub_depth_info.publish(
                self._camera_info(image, fx_full, fy_full, cx_full, cy_full)
            )

        crop = _choose_fov_crop(
            width,
            height,
            fx_full,
            fy_full,
            self.out_w,
            self.out_h,
            self.hfov,
            self.vfov,
        )
        small, small_calibration = self._project_depth(
            depth,
            crop,
            self.out_w,
            self.out_h,
            fx_full,
            fy_full,
            cx_full,
            cy_full,
            self._min_units,
            self._max_units,
        )
        small_image = self._image_message(small, "16UC1", self.frame_depth, stamp)
        self._pub_small.publish(small_image)
        self._pub_small_info.publish(self._camera_info(small_image, *small_calibration))
        self._log_projection("small", crop, small_calibration, self.out_w, self.out_h)

        if self._pub_origin is not None and self._pub_origin_info is not None:
            origin_crop = _choose_fov_crop(
                width,
                height,
                fx_full,
                fy_full,
                self.origin_out_w,
                self.origin_out_h,
                self.origin_hfov,
                self.origin_vfov,
            )
            origin, origin_calibration = self._project_depth(
                depth,
                origin_crop,
                self.origin_out_w,
                self.origin_out_h,
                fx_full,
                fy_full,
                cx_full,
                cy_full,
                self._origin_min_units,
                self._origin_max_units,
            )
            origin_image = self._image_message(origin, "16UC1", self.frame_depth, stamp)
            self._pub_origin.publish(origin_image)
            self._pub_origin_info.publish(
                self._camera_info(origin_image, *origin_calibration)
            )
            self._log_projection(
                "origin pre-rotation",
                origin_crop,
                origin_calibration,
                self.origin_out_w,
                self.origin_out_h,
            )

    @staticmethod
    def _project_depth(
        depth: np.ndarray,
        crop: tuple[int, int, int, int],
        out_width: int,
        out_height: int,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        minimum: int,
        maximum: int,
    ) -> tuple[np.ndarray, tuple[float, float, float, float]]:
        x, y, width, height = crop
        resized = _resize_nearest(
            depth[y : y + height, x : x + width], out_width, out_height
        )
        projected = _clamp_depth(resized, minimum, maximum)
        scale_x = float(out_width) / float(width)
        scale_y = float(out_height) / float(height)
        return projected, (
            fx * scale_x,
            fy * scale_y,
            (cx - x) * scale_x,
            (cy - y) * scale_y,
        )

    def _publish_motion(
        self,
        samples: tuple[tuple[object, float, float, float], ...],
    ) -> None:
        rs = self._rs
        for stream_type, x, y, z in samples:
            message = Imu()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.frame_depth
            if stream_type == rs.stream.gyro:
                message.angular_velocity.x = x
                message.angular_velocity.y = y
                message.angular_velocity.z = z
            elif stream_type == rs.stream.accel:
                message.linear_acceleration.x = x
                message.linear_acceleration.y = y
                message.linear_acceleration.z = z
            else:
                continue
            self._pub_imu.publish(message)

    @staticmethod
    def _image_message(array: np.ndarray, encoding: str, frame_id: str, stamp) -> Image:
        data = np.ascontiguousarray(array)
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.height = int(data.shape[0])
        message.width = int(data.shape[1])
        message.encoding = encoding
        message.is_bigendian = False
        message.step = int(data.strides[0])
        message.data = byte_array("B", data.tobytes())
        return message

    @staticmethod
    def _camera_info(
        image: Image,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> CameraInfo:
        info = CameraInfo()
        info.header = image.header
        info.width = image.width
        info.height = image.height
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _log_projection(
        self,
        name: str,
        crop: tuple[int, int, int, int],
        calibration: tuple[float, float, float, float],
        width: int,
        height: int,
    ) -> None:
        now = time.monotonic()
        key = f"projection:{name}"
        if now - self._last_log_times.get(key, float("-inf")) < 5.0:
            return
        self._last_log_times[key] = now
        fx, fy, _cx, _cy = calibration
        horizontal_fov = math.degrees(2.0 * math.atan(float(width) / (2.0 * fx)))
        vertical_fov = math.degrees(2.0 * math.atan(float(height) / (2.0 * fy)))
        x, y, crop_width, crop_height = crop
        self.get_logger().info(
            f"FOV({name}): {width}x{height}, ROI={crop_width}x{crop_height}"
            f"@({x},{y}), HFOV={horizontal_fov:.4f}, VFOV={vertical_fov:.4f}"
        )

    def _log_throttled(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self._last_log_times.get(key, float("-inf")) < 5.0:
            return
        self._last_log_times[key] = now
        self.get_logger().error(message)

    def _stop_pipeline(self) -> None:
        if not self._pipeline_started:
            return
        self._pipeline_started = False
        try:
            self._pipeline.stop()
        except Exception as exc:
            self.get_logger().warning(f"RealSense stop failed: {exc}")

    def destroy_node(self):
        self._stop_pipeline()
        return super().destroy_node()


def create_node(context: NodeBuildContext) -> RealSenseDepthPublisher:
    return RealSenseDepthPublisher(context.node_name, context.params)


__all__ = ["RealSenseDepthPublisher", "create_node"]

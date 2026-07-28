from __future__ import annotations

from array import array as byte_array
from collections import deque
from collections.abc import Mapping
import math
from pathlib import Path
from threading import Lock
import time

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu

from .realsense_depth_node import _choose_fov_crop, _clamp_depth, _resize_nearest


ORBBEC_VENDOR_ID = 0x2BC5
GEMINI_335_PRODUCT_ID = 0x0800

ORBBEC_DEFAULTS: dict[str, object] = {
    "orbbec_serial": "",
    "orbbec_usb_port": "",
    "orbbec_enable_sdk_filters": True,
    "orbbec_fallback_hfov": 90.0,
    "orbbec_fallback_vfov": 65.0,
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
    "min_dist": 0.2,
    "max_dist": 2.5,
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


def _depth_mm_from_sdk_data(
    data,
    width: int,
    height: int,
    scale_mm: float,
) -> np.ndarray:
    if width <= 0 or height <= 0:
        raise ValueError(f"invalid Orbbec depth dimensions: {width}x{height}")
    if not math.isfinite(scale_mm) or scale_mm <= 0.0:
        raise ValueError(f"invalid Orbbec depth scale: {scale_mm}")
    required = width * height * 2
    if memoryview(data).nbytes < required:
        raise ValueError(
            f"malformed Orbbec depth frame: expected {required} bytes, "
            f"received {memoryview(data).nbytes}"
        )
    raw = np.frombuffer(data, dtype=np.uint16, count=width * height).reshape(
        height, width
    )
    millimeters = np.rint(raw.astype(np.float64) * scale_mm)
    return np.ascontiguousarray(np.clip(millimeters, 0.0, 65535.0), dtype=np.uint16)


class OrbbecDepthPublisher(Node):
    def __init__(
        self,
        node_name: str,
        _mod_root: Path,
        params: Mapping[str, object],
    ) -> None:
        super().__init__(node_name)
        try:
            import pyorbbecsdk as ob
        except ImportError as exc:
            super().destroy_node()
            raise RuntimeError(
                "pyorbbecsdk is required by the Gemini 335 backend; bundle "
                "pyorbbecsdk2 under vendor/python/<platform>-<python-abi>"
            ) from exc

        self._ob = ob
        self._sdk_context = None
        self._device = None
        self._pipeline = None
        self._pipeline_started = False
        self._imu_pipeline = None
        self._imu_pipeline_started = False
        self._frame_lock = Lock()
        self._latest_frameset = None
        self._motion_samples: deque[tuple[str, float, float, float]] = deque(
            maxlen=2048
        )
        self._depth_filters: list[object] = []
        self._enabled_filter_names: list[str] = []
        self._last_log_times: dict[str, float] = {}
        self._depth_intrinsics: tuple[
            float, float, float, float, int, int
        ] | None = None
        self._depth_distortion: list[float] = [0.0] * 5

        try:
            self._read_parameters(params)
            self._create_publishers()
            self._start_sdk()
            self._timer = self.create_timer(
                1.0 / float(self.publish_rate_hz),
                self._publish_latest,
            )
        except BaseException:
            self._stop_sdk()
            super().destroy_node()
            raise

    def _read_parameters(self, params: Mapping[str, object]) -> None:
        unknown = set(params) - set(ORBBEC_DEFAULTS)
        if unknown:
            raise ValueError(f"unknown Orbbec node params: {sorted(unknown)}")
        for name, default in ORBBEC_DEFAULTS.items():
            self.declare_parameter(name, params.get(name, default))
            value = self.get_parameter(name).value
            if isinstance(default, bool):
                if not isinstance(value, bool):
                    raise ValueError(f"Orbbec param '{name}' must be a boolean")
            elif isinstance(default, int):
                if isinstance(value, bool) or not isinstance(value, int):
                    raise ValueError(f"Orbbec param '{name}' must be an integer")
            elif isinstance(default, float):
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise ValueError(f"Orbbec param '{name}' must be a number")
                value = float(value)
            elif not isinstance(value, str):
                raise ValueError(f"Orbbec param '{name}' must be a string")
            setattr(self, name, value)

        if self.serial and not self.orbbec_serial:
            self.orbbec_serial = self.serial
        for name in (
            "publish_rate_hz",
            "depth_w",
            "depth_h",
            "depth_fps",
            "color_w",
            "color_h",
            "color_fps",
            "out_w",
            "out_h",
            "origin_out_w",
            "origin_out_h",
        ):
            if getattr(self, name) <= 0:
                raise ValueError(f"Orbbec param '{name}' must be greater than zero")
        for minimum_name, maximum_name in (
            ("min_dist", "max_dist"),
            ("origin_min_dist", "origin_max_dist"),
        ):
            minimum = getattr(self, minimum_name)
            maximum = getattr(self, maximum_name)
            if minimum < 0.0 or maximum <= 0.0 or minimum > maximum:
                raise ValueError(
                    f"Orbbec params '{minimum_name}/{maximum_name}' are invalid"
                )
        for name in (
            "hfov",
            "vfov",
            "origin_hfov",
            "origin_vfov",
            "orbbec_fallback_hfov",
            "orbbec_fallback_vfov",
        ):
            if not 0.0 < getattr(self, name) < 180.0:
                raise ValueError(f"Orbbec param '{name}' must be between 0 and 180")
        self._min_mm = max(0, min(65535, round(self.min_dist * 1000.0)))
        self._max_mm = max(1, min(65535, round(self.max_dist * 1000.0)))
        self._origin_min_mm = max(0, min(65535, round(self.origin_min_dist * 1000.0)))
        self._origin_max_mm = max(1, min(65535, round(self.origin_max_dist * 1000.0)))

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

    def _start_sdk(self) -> None:
        ob = self._ob
        ob.Context.set_logger_to_console(ob.OBLogLevel.WARNING)
        self._sdk_context = ob.Context()
        self._device, uid = self._select_device(self._sdk_context.query_devices())
        info = self._device.get_device_info()

        self._pipeline = ob.Pipeline(self._device)
        config = ob.Config()
        try:
            depth_profile = self._pipeline.get_stream_profile_list(
                ob.OBSensorType.DEPTH_SENSOR
            ).get_video_stream_profile(
                self.depth_w,
                self.depth_h,
                ob.OBFormat.Y16,
                self.depth_fps,
            )
        except Exception as exc:
            raise RuntimeError(
                "Orbbec depth profile is unavailable: "
                f"{self.depth_w}x{self.depth_h}@{self.depth_fps} Y16: {exc}"
            ) from exc
        config.enable_stream(depth_profile)
        self._initialize_calibration(depth_profile)
        self._configure_video_streams(config, self._pipeline)
        self._initialize_filters()

        try:
            self._pipeline.start(config, self._on_video_frames)
        except Exception as exc:
            raise RuntimeError(
                f"Orbbec SDK depth pipeline start failed: {exc}"
            ) from exc
        self._pipeline_started = True

        if self.enable_imu:
            self._start_imu_pipeline()

        self._log_intrinsics()
        self.get_logger().info(
            "Orbbec SDK depth filters: "
            + (
                ", ".join(self._enabled_filter_names)
                if self._enabled_filter_names
                else "none"
            )
        )
        self.get_logger().info(
            "started Orbbec SDK directly: "
            f"sdk={ob.get_version()}, camera={info.get_name()}, "
            f"serial={info.get_serial_number()}, firmware={info.get_firmware_version()}, "
            f"connection={info.get_connection_type()}, uid={uid}, "
            f"depth={depth_profile.get_width()}x{depth_profile.get_height()}"
            f"@{depth_profile.get_fps()} {depth_profile.get_format()}"
        )

    def _select_device(self, devices):
        requested_serial = self.orbbec_serial.strip()
        requested_port = self.orbbec_usb_port.strip()
        visible: list[str] = []
        for index in range(devices.get_count()):
            serial = devices.get_device_serial_number_by_index(index)
            uid = devices.get_device_uid_by_index(index)
            vid = int(devices.get_device_vid_by_index(index))
            pid = int(devices.get_device_pid_by_index(index))
            visible.append(f"{vid:04x}:{pid:04x} serial={serial} uid={uid}")
            if vid != ORBBEC_VENDOR_ID or pid != GEMINI_335_PRODUCT_ID:
                continue
            if requested_serial and serial != requested_serial:
                continue
            if requested_port and not (
                uid == requested_port or uid.startswith(requested_port + "-")
            ):
                continue
            return devices.get_device_by_index(index), uid
        selection = []
        if requested_serial:
            selection.append(f"serial={requested_serial}")
        if requested_port:
            selection.append(f"usb_port={requested_port}")
        suffix = f" matching {', '.join(selection)}" if selection else ""
        detected = "; ".join(visible) or "none"
        raise RuntimeError(
            f"Orbbec Gemini 335 2bc5:0800{suffix} not available; detected: {detected}"
        )

    def _configure_video_streams(self, config, pipeline) -> None:
        ob = self._ob
        if self.enable_color:
            profiles = pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
            color_profile = None
            for frame_format in (
                ob.OBFormat.RGB,
                ob.OBFormat.BGR,
                ob.OBFormat.MJPG,
            ):
                try:
                    color_profile = profiles.get_video_stream_profile(
                        self.color_w,
                        self.color_h,
                        frame_format,
                        self.color_fps,
                    )
                    break
                except Exception:
                    continue
            if color_profile is None:
                color_profile = profiles.get_default_video_stream_profile()
                self.get_logger().warning(
                    "requested Orbbec color profile is unavailable; using "
                    f"{color_profile.get_width()}x{color_profile.get_height()}"
                    f"@{color_profile.get_fps()} {color_profile.get_format()}"
                )
            config.enable_stream(color_profile)

        if self.enable_ir:
            enabled = 0
            for sensor_type in (
                ob.OBSensorType.LEFT_IR_SENSOR,
                ob.OBSensorType.RIGHT_IR_SENSOR,
            ):
                try:
                    profile = pipeline.get_stream_profile_list(
                        sensor_type
                    ).get_default_video_stream_profile()
                    config.enable_stream(profile)
                    enabled += 1
                except Exception:
                    continue
            if enabled == 0:
                try:
                    profile = pipeline.get_stream_profile_list(
                        ob.OBSensorType.IR_SENSOR
                    ).get_default_video_stream_profile()
                    config.enable_stream(profile)
                except Exception as exc:
                    raise RuntimeError(
                        f"Orbbec IR stream is unavailable: {exc}"
                    ) from exc

    def _initialize_calibration(self, depth_profile) -> None:
        try:
            intrinsic = depth_profile.get_intrinsic()
            self._depth_intrinsics = (
                float(intrinsic.fx),
                float(intrinsic.fy),
                float(intrinsic.cx),
                float(intrinsic.cy),
                int(intrinsic.width),
                int(intrinsic.height),
            )
            distortion = depth_profile.get_distortion()
            self._depth_distortion = [
                float(distortion.k1),
                float(distortion.k2),
                float(distortion.p1),
                float(distortion.p2),
                float(distortion.k3),
            ]
        except Exception as exc:
            self._depth_intrinsics = None
            self._depth_distortion = [0.0] * 5
            self.get_logger().warning(
                f"cannot read Orbbec calibration; using fallback FOV: {exc}"
            )

    def _log_intrinsics(self) -> None:
        if self._depth_intrinsics is None:
            return
        fx, fy, cx, cy, width, height = self._depth_intrinsics
        horizontal_fov = math.degrees(2.0 * math.atan(float(width) / (2.0 * fx)))
        vertical_fov = math.degrees(2.0 * math.atan(float(height) / (2.0 * fy)))
        self.get_logger().info(
            "Orbbec depth intrinsics: "
            f"size={width}x{height}, fx={fx:.3f}, fy={fy:.3f}, "
            f"cx={cx:.3f}, cy={cy:.3f}, model=plumb_bob, "
            f"HFOV={horizontal_fov:.2f}, VFOV={vertical_fov:.2f}"
        )

    def _initialize_filters(self) -> None:
        ob = self._ob
        sensor = self._device.get_sensor(ob.OBSensorType.DEPTH_SENSOR)
        mandatory_filters: list[object] = []
        optional_filters: list[object] = []
        for depth_filter in sensor.get_recommended_filters():
            mandatory = depth_filter.is_disparity_transform_filter()
            optional = self.orbbec_enable_sdk_filters and (
                depth_filter.is_noise_removal_filter()
                or depth_filter.is_spatial_advanced_filter()
                or depth_filter.is_temporal_filter()
                or depth_filter.is_hole_filling_filter()
            )
            should_enable = mandatory or optional
            depth_filter.enable(should_enable)
            if should_enable:
                target = mandatory_filters if mandatory else optional_filters
                target.append(depth_filter)
        self._depth_filters = mandatory_filters + optional_filters
        self._enabled_filter_names = [item.get_name() for item in self._depth_filters]

    def _start_imu_pipeline(self) -> None:
        ob = self._ob
        self._imu_pipeline = ob.Pipeline(self._device)
        config = ob.Config()
        config.enable_accel_stream()
        config.enable_gyro_stream()
        try:
            self._imu_pipeline.start(config, self._on_imu_frames)
        except Exception as exc:
            raise RuntimeError(f"Orbbec SDK IMU pipeline start failed: {exc}") from exc
        self._imu_pipeline_started = True

    def _on_video_frames(self, frameset) -> None:
        if frameset is None:
            return
        with self._frame_lock:
            self._latest_frameset = frameset

    def _on_imu_frames(self, frameset) -> None:
        if frameset is None:
            return
        try:
            samples: list[tuple[str, float, float, float]] = []
            accel = frameset.get_accel_frame()
            if accel:
                samples.append(
                    (
                        "accel",
                        float(accel.get_x()),
                        float(accel.get_y()),
                        float(accel.get_z()),
                    )
                )
            gyro = frameset.get_gyro_frame()
            if gyro:
                samples.append(
                    (
                        "gyro",
                        float(gyro.get_x()),
                        float(gyro.get_y()),
                        float(gyro.get_z()),
                    )
                )
            if samples:
                with self._frame_lock:
                    self._motion_samples.extend(samples)
        except Exception as exc:
            self._log_throttled(
                "imu-callback", f"Orbbec SDK IMU callback failed: {exc}"
            )

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
                self._log_throttled("frameset", f"Orbbec SDK frame failed: {exc}")
        if motion_samples and self._pub_imu is not None:
            self._publish_motion(motion_samples)

    def _publish_frameset(self, frameset) -> None:
        stamp = self.get_clock().now().to_msg()
        depth_frame = frameset.get_depth_frame()
        if depth_frame:
            self._publish_depth_frame(depth_frame, stamp)
        if self._pub_color is not None:
            color_frame = frameset.get_color_frame()
            if color_frame:
                color, encoding = self._color_array(color_frame)
                self._pub_color.publish(
                    self._image_message(color, encoding, self.frame_color, stamp)
                )
        if self._pub_ir1 is not None and self._pub_ir2 is not None:
            left = frameset.get_left_ir_frame()
            right = frameset.get_right_ir_frame()
            if not left and not right:
                left = frameset.get_ir_frame()
            if left:
                image, encoding = self._ir_array(left)
                self._pub_ir1.publish(
                    self._image_message(image, encoding, self.frame_depth, stamp)
                )
            if right:
                image, encoding = self._ir_array(right)
                self._pub_ir2.publish(
                    self._image_message(image, encoding, self.frame_depth, stamp)
                )

    def _publish_depth_frame(self, depth_frame, stamp) -> None:
        processed = depth_frame
        for depth_filter in self._depth_filters:
            processed = depth_filter.process(processed)
            if processed is None:
                raise RuntimeError(
                    f"Orbbec filter {depth_filter.get_name()} returned no frame"
                )
        if hasattr(processed, "as_depth_frame"):
            processed = processed.as_depth_frame()
        width = int(processed.get_width())
        height = int(processed.get_height())
        depth = _depth_mm_from_sdk_data(
            processed.get_data(),
            width,
            height,
            float(processed.get_depth_scale()),
        )
        fx, fy, cx, cy = self._intrinsics(width, height)

        if self.publish_full:
            full = _clamp_depth(depth, self._min_mm, self._max_mm)
            image = self._image_message(full, "16UC1", self.frame_depth, stamp)
            self._pub_depth.publish(image)
            self._pub_depth_info.publish(self._camera_info(image, fx, fy, cx, cy))

        small_crop = _choose_fov_crop(
            width,
            height,
            fx,
            fy,
            self.out_w,
            self.out_h,
            self.hfov,
            self.vfov,
        )
        small, calibration = self._project(
            depth,
            small_crop,
            self.out_w,
            self.out_h,
            fx,
            fy,
            cx,
            cy,
            self._min_mm,
            self._max_mm,
        )
        small_image = self._image_message(small, "16UC1", self.frame_depth, stamp)
        self._pub_small.publish(small_image)
        self._pub_small_info.publish(self._camera_info(small_image, *calibration))
        self._log_projection("small", small_crop, calibration, self.out_w, self.out_h)

        if self._pub_origin is not None and self._pub_origin_info is not None:
            origin_crop = _choose_fov_crop(
                width,
                height,
                fx,
                fy,
                self.origin_out_w,
                self.origin_out_h,
                self.origin_hfov,
                self.origin_vfov,
            )
            origin, origin_calibration = self._project(
                depth,
                origin_crop,
                self.origin_out_w,
                self.origin_out_h,
                fx,
                fy,
                cx,
                cy,
                self._origin_min_mm,
                self._origin_max_mm,
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

    def _intrinsics(self, width: int, height: int) -> tuple[float, float, float, float]:
        if self._depth_intrinsics is not None:
            (
                fx,
                fy,
                cx,
                cy,
                calibration_width,
                calibration_height,
            ) = self._depth_intrinsics
            scale_x = float(width) / float(calibration_width)
            scale_y = float(height) / float(calibration_height)
            return fx * scale_x, fy * scale_y, cx * scale_x, cy * scale_y
        hfov = math.radians(self.orbbec_fallback_hfov)
        vfov = math.radians(self.orbbec_fallback_vfov)
        return (
            float(width) / (2.0 * math.tan(hfov / 2.0)),
            float(height) / (2.0 * math.tan(vfov / 2.0)),
            float(width - 1) / 2.0,
            float(height - 1) / 2.0,
        )

    @staticmethod
    def _project(
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

    def _color_array(self, frame) -> tuple[np.ndarray, str]:
        ob = self._ob
        width = int(frame.get_width())
        height = int(frame.get_height())
        frame_format = frame.get_format()
        data = frame.get_data()
        if frame_format == ob.OBFormat.RGB:
            return np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3), "rgb8"
        if frame_format == ob.OBFormat.BGR:
            return np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3), "bgr8"
        if frame_format == ob.OBFormat.RGBA:
            return (
                np.frombuffer(data, dtype=np.uint8).reshape(height, width, 4),
                "rgba8",
            )
        if frame_format == ob.OBFormat.BGRA:
            return (
                np.frombuffer(data, dtype=np.uint8).reshape(height, width, 4),
                "bgra8",
            )
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError(
                f"OpenCV is required for Orbbec color format {frame_format}"
            ) from exc
        encoded = np.frombuffer(data, dtype=np.uint8)
        if frame_format == ob.OBFormat.MJPG:
            decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        elif frame_format == ob.OBFormat.YUYV:
            decoded = cv2.cvtColor(
                encoded.reshape(height, width, 2), cv2.COLOR_YUV2BGR_YUY2
            )
        elif frame_format == ob.OBFormat.UYVY:
            decoded = cv2.cvtColor(
                encoded.reshape(height, width, 2), cv2.COLOR_YUV2BGR_UYVY
            )
        else:
            raise ValueError(f"unsupported Orbbec color format: {frame_format}")
        if decoded is None:
            raise ValueError(f"cannot decode Orbbec color format: {frame_format}")
        return decoded, "bgr8"

    def _ir_array(self, frame) -> tuple[np.ndarray, str]:
        width = int(frame.get_width())
        height = int(frame.get_height())
        if frame.get_format() == self._ob.OBFormat.Y8:
            return (
                np.frombuffer(frame.get_data(), dtype=np.uint8).reshape(height, width),
                "mono8",
            )
        return (
            np.frombuffer(frame.get_data(), dtype=np.uint16).reshape(height, width),
            "mono16",
        )

    def _publish_motion(
        self,
        samples: tuple[tuple[str, float, float, float], ...],
    ) -> None:
        assert self._pub_imu is not None
        for kind, x, y, z in samples:
            message = Imu()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.frame_depth
            if kind == "gyro":
                message.angular_velocity.x = x
                message.angular_velocity.y = y
                message.angular_velocity.z = z
            else:
                message.linear_acceleration.x = x
                message.linear_acceleration.y = y
                message.linear_acceleration.z = z
            self._pub_imu.publish(message)

    @staticmethod
    def _image_message(
        array: np.ndarray,
        encoding: str,
        frame_id: str,
        stamp,
    ) -> Image:
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

    def _camera_info(
        self,
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
        info.distortion_model = "plumb_bob"
        info.d = list(self._depth_distortion)
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
        if now - self._last_log_times.get(key, float("-inf")) < 30.0:
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

    def _stop_sdk(self) -> None:
        if self._imu_pipeline_started and self._imu_pipeline is not None:
            self._imu_pipeline_started = False
            try:
                self._imu_pipeline.stop()
            except Exception as exc:
                self.get_logger().warning(f"Orbbec SDK IMU pipeline stop failed: {exc}")
        if self._pipeline_started and self._pipeline is not None:
            self._pipeline_started = False
            try:
                self._pipeline.stop()
            except Exception as exc:
                self.get_logger().warning(
                    f"Orbbec SDK depth pipeline stop failed: {exc}"
                )
        self._latest_frameset = None
        self._depth_filters.clear()
        self._enabled_filter_names.clear()
        self._imu_pipeline = None
        self._pipeline = None
        self._device = None
        self._sdk_context = None

    def destroy_node(self):
        self._stop_sdk()
        return super().destroy_node()


__all__ = [
    "GEMINI_335_PRODUCT_ID",
    "ORBBEC_DEFAULTS",
    "ORBBEC_VENDOR_ID",
    "OrbbecDepthPublisher",
    "_depth_mm_from_sdk_data",
]

import math
import time

import communication.msg as bxiMsg
import rclpy
import std_msgs.msg
from rclpy.node import Node


class LineFollowController(Node):
    def __init__(self):
        super().__init__("line_follow_controller")

        self.declare_parameter("line_offset_topic", "/simulation/front_line_camera/line_offset")
        self.declare_parameter("line_state_topic", "/simulation/front_line_camera/line_state")
        self.declare_parameter("motion_commands_topic", "motion_commands")
        self.declare_parameter("forward_vel", 0.5)
        self.declare_parameter("forward_accel_limit", 0.8)
        self.declare_parameter("max_amp_input_vx", 0.18)
        self.declare_parameter("max_amp_input_yawdot", 0.45)
        self.declare_parameter("control_mode", "pid")
        self.declare_parameter("stanley_heading_gain", 0.70)
        self.declare_parameter("stanley_crosstrack_gain", 0.85)
        self.declare_parameter("stanley_yaw_gain", 1.0)
        self.declare_parameter("stanley_min_speed", 0.25)
        self.declare_parameter("confidence_hold_threshold", 0.20)
        self.declare_parameter("confidence_full_speed_threshold", 0.55)
        self.declare_parameter("yaw_gain", 0.2)
        self.declare_parameter("pid_error_source", "offset")
        self.declare_parameter("pid_heading_feedforward_gain", 0.45)
        self.declare_parameter("pid_kp", -1.0)
        self.declare_parameter("pid_ki", 0.0)
        self.declare_parameter("pid_kd", 0.0)
        self.declare_parameter("integral_limit", 0.6)
        self.declare_parameter("max_yawdot", 0.45)
        self.declare_parameter("max_yawdot_rate", 1.8)
        self.declare_parameter("steering_pulse_enabled", True)
        self.declare_parameter("steering_pulse_on_time", 0.18)
        self.declare_parameter("steering_pulse_period", 0.55)
        self.declare_parameter("offset_filter_alpha", 0.35)
        self.declare_parameter("offset_deadband", 0.025)
        self.declare_parameter("heading_deadband", 0.035)
        self.declare_parameter("control_deadband", 0.030)
        self.declare_parameter("pid_derivative_filter_alpha", 0.18)
        self.declare_parameter("slowdown_offset", 0.55)
        self.declare_parameter("min_forward_vel_ratio", 0.45)
        self.declare_parameter("minimum_forward_command", 0.16)
        self.declare_parameter("speed_pid_kp", 1.10)
        self.declare_parameter("speed_pid_ki", 0.04)
        self.declare_parameter("speed_pid_kd", 0.18)
        self.declare_parameter("speed_pid_integral_limit", 0.8)
        self.declare_parameter("enable_visual_speed_control", False)
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("offset_timeout", 0.5)
        self.declare_parameter("boundary_warning_margin", 0.35)
        self.declare_parameter("boundary_recovery_margin", 0.18)
        self.declare_parameter("boundary_stop_margin", 0.02)
        self.declare_parameter("recovery_yaw_scale", 0.65)
        self.declare_parameter("lost_recovery_yaw_scale", 0.20)
        self.declare_parameter("lost_recovery_forward_ratio", 0.30)
        self.declare_parameter("recovery_forward_ratio", 0.36)
        self.declare_parameter("lost_forward_ratio", 0.45)
        self.declare_parameter("enable_lateral_correction", False)
        self.declare_parameter("lateral_velocity_gain", 0.65)
        self.declare_parameter("max_lateral_velocity", 0.35)
        self.declare_parameter("lateral_velocity_sign", 1.0)
        self.declare_parameter("lateral_deadband", 0.035)

        line_offset_topic = self.get_parameter("line_offset_topic").value
        line_state_topic = self.get_parameter("line_state_topic").value
        motion_commands_topic = self.get_parameter("motion_commands_topic").value
        self.forward_vel = float(self.get_parameter("forward_vel").value)
        self.forward_accel_limit = abs(float(self.get_parameter("forward_accel_limit").value))
        self.max_amp_input_vx = abs(float(self.get_parameter("max_amp_input_vx").value))
        self.max_amp_input_yawdot = abs(float(self.get_parameter("max_amp_input_yawdot").value))
        self.control_mode = str(self.get_parameter("control_mode").value).lower()
        self.stanley_heading_gain = float(self.get_parameter("stanley_heading_gain").value)
        self.stanley_crosstrack_gain = float(self.get_parameter("stanley_crosstrack_gain").value)
        self.stanley_yaw_gain = float(self.get_parameter("stanley_yaw_gain").value)
        self.stanley_min_speed = max(0.01, abs(float(self.get_parameter("stanley_min_speed").value)))
        self.confidence_hold_threshold = float(self.get_parameter("confidence_hold_threshold").value)
        self.confidence_full_speed_threshold = float(self.get_parameter("confidence_full_speed_threshold").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.pid_error_source = str(self.get_parameter("pid_error_source").value).lower()
        self.pid_heading_feedforward_gain = float(
            self.get_parameter("pid_heading_feedforward_gain").value
        )
        self.pid_kp = float(self.get_parameter("pid_kp").value)
        if self.pid_kp < 0.0:
            self.pid_kp = self.yaw_gain
        self.pid_ki = float(self.get_parameter("pid_ki").value)
        self.pid_kd = float(self.get_parameter("pid_kd").value)
        self.integral_limit = abs(float(self.get_parameter("integral_limit").value))
        self.max_yawdot = abs(float(self.get_parameter("max_yawdot").value))
        self.max_yawdot_rate = abs(float(self.get_parameter("max_yawdot_rate").value))
        self.steering_pulse_enabled = bool(
            self.get_parameter("steering_pulse_enabled").value
        ) and self.control_mode != "pid"
        self.steering_pulse_on_time = max(
            0.05, float(self.get_parameter("steering_pulse_on_time").value)
        )
        self.steering_pulse_period = max(
            self.steering_pulse_on_time + 0.05,
            float(self.get_parameter("steering_pulse_period").value),
        )
        self.offset_filter_alpha = float(self.get_parameter("offset_filter_alpha").value)
        self.offset_filter_alpha = max(0.0, min(1.0, self.offset_filter_alpha))
        self.offset_deadband = abs(float(self.get_parameter("offset_deadband").value))
        self.heading_deadband = abs(float(self.get_parameter("heading_deadband").value))
        self.control_deadband = abs(float(self.get_parameter("control_deadband").value))
        self.pid_derivative_filter_alpha = float(self.get_parameter("pid_derivative_filter_alpha").value)
        self.pid_derivative_filter_alpha = max(0.0, min(1.0, self.pid_derivative_filter_alpha))
        self.slowdown_offset = max(0.01, abs(float(self.get_parameter("slowdown_offset").value)))
        self.min_forward_vel_ratio = float(self.get_parameter("min_forward_vel_ratio").value)
        self.min_forward_vel_ratio = max(0.0, min(1.0, self.min_forward_vel_ratio))
        self.minimum_forward_command = max(
            0.0, abs(float(self.get_parameter("minimum_forward_command").value))
        )
        self.speed_pid_kp = max(0.0, float(self.get_parameter("speed_pid_kp").value))
        self.speed_pid_ki = max(0.0, float(self.get_parameter("speed_pid_ki").value))
        self.speed_pid_kd = max(0.0, float(self.get_parameter("speed_pid_kd").value))
        self.speed_pid_integral_limit = max(
            0.0, abs(float(self.get_parameter("speed_pid_integral_limit").value))
        )
        self.enable_visual_speed_control = bool(
            self.get_parameter("enable_visual_speed_control").value
        )
        publish_hz = float(self.get_parameter("publish_hz").value)
        self.offset_timeout = float(self.get_parameter("offset_timeout").value)
        self.boundary_warning_margin = float(self.get_parameter("boundary_warning_margin").value)
        self.boundary_recovery_margin = float(self.get_parameter("boundary_recovery_margin").value)
        self.boundary_stop_margin = float(self.get_parameter("boundary_stop_margin").value)
        self.recovery_yaw_scale = max(
            0.1, min(1.0, float(self.get_parameter("recovery_yaw_scale").value))
        )
        self.lost_recovery_yaw_scale = max(
            0.1,
            min(1.0, float(self.get_parameter("lost_recovery_yaw_scale").value)),
        )
        self.lost_recovery_forward_ratio = max(
            0.0,
            min(1.0, float(self.get_parameter("lost_recovery_forward_ratio").value)),
        )
        self.recovery_forward_ratio = float(self.get_parameter("recovery_forward_ratio").value)
        self.lost_forward_ratio = float(self.get_parameter("lost_forward_ratio").value)
        self.enable_lateral_correction = bool(self.get_parameter("enable_lateral_correction").value)
        self.lateral_velocity_gain = abs(float(self.get_parameter("lateral_velocity_gain").value))
        self.max_lateral_velocity = abs(float(self.get_parameter("max_lateral_velocity").value))
        self.lateral_velocity_sign = float(self.get_parameter("lateral_velocity_sign").value)
        self.lateral_deadband = abs(float(self.get_parameter("lateral_deadband").value))

        self.latest_offset = 0.0
        self.latest_heading_error = 0.0
        self.latest_confidence = 0.0
        self.latest_control_error = 0.0
        self.latest_boundary_margin = 1.0
        self.latest_lane_width = 0.0
        self.latest_mode = 3.0
        self.latest_offset_time = 0.0
        self.latest_state_time = 0.0
        self.filtered_offset = 0.0
        self.last_valid_offset = 0.0
        self.filtered_heading_error = 0.0
        self.filtered_pid_error = 0.0
        self.integral_error = 0.0
        self.last_error = 0.0
        self.filtered_derivative_error = 0.0
        self.speed_pid_integral = 0.0
        self.last_speed_pid_error = 0.0
        self.last_yawdot = 0.0
        self.steering_pulse_clock = 0.0
        self.current_forward_vel = 0.0
        self.last_publish_time = time.monotonic()

        self.offset_sub = self.create_subscription(
            std_msgs.msg.Float32,
            line_offset_topic,
            self.offset_callback,
            10,
        )
        self.state_sub = self.create_subscription(
            std_msgs.msg.Float32MultiArray,
            line_state_topic,
            self.line_state_callback,
            10,
        )
        self.pub = self.create_publisher(
            bxiMsg.MotionCommands,
            motion_commands_topic,
            10,
        )
        self.timer = self.create_timer(1.0 / max(publish_hz, 1.0), self.timer_callback)

        self.get_logger().info(
            f"line follow: {line_state_topic} -> {motion_commands_topic}, "
            f"forward_vel={self.forward_vel:.2f}, "
            f"mode={self.control_mode}, "
            f"pid=({self.pid_kp:.3f}, {self.pid_ki:.3f}, {self.pid_kd:.3f}), "
            f"pid_error_source={self.pid_error_source}"
        )

    def offset_callback(self, msg):
        now = time.monotonic()
        if (now - self.latest_state_time) <= self.offset_timeout:
            return
        self.latest_offset = float(msg.data)
        self.latest_control_error = self.latest_offset
        self.latest_heading_error = 0.0
        self.latest_confidence = 0.5
        self.latest_offset_time = now

    def line_state_callback(self, msg):
        if len(msg.data) < 4:
            return
        self.latest_offset = float(msg.data[0])
        self.latest_heading_error = float(msg.data[1])
        self.latest_confidence = float(msg.data[2])
        self.latest_control_error = float(msg.data[3])
        self.latest_boundary_margin = float(msg.data[4]) if len(msg.data) > 4 else 1.0
        self.latest_lane_width = float(msg.data[5]) if len(msg.data) > 5 else 0.0
        self.latest_mode = float(msg.data[6]) if len(msg.data) > 6 else 0.0
        now = time.monotonic()
        self.latest_state_time = now
        self.latest_offset_time = now

    def timer_callback(self):
        now = time.monotonic()
        dt = max(now - self.last_publish_time, 1.0e-3)
        state_is_fresh = (now - self.latest_state_time) <= self.offset_timeout
        offset_is_fresh = (now - self.latest_offset_time) <= self.offset_timeout
        raw_offset = self.latest_offset if offset_is_fresh else 0.0
        raw_heading_error = self.latest_heading_error if state_is_fresh else 0.0
        confidence = self.latest_confidence if state_is_fresh else 0.0
        boundary_margin = self.latest_boundary_margin if state_is_fresh else -1.0
        mode = self.latest_mode if state_is_fresh else 3.0

        if confidence < self.confidence_hold_threshold or mode >= 3.0:
            raw_offset = self.last_valid_offset * 0.80
            raw_heading_error = self.filtered_heading_error * 0.85
        elif confidence >= self.confidence_hold_threshold and mode < 3.0:
            self.last_valid_offset = self.filtered_offset

        if abs(raw_offset) < self.offset_deadband:
            offset = 0.0
        else:
            offset = raw_offset - self.offset_deadband * (1.0 if raw_offset > 0.0 else -1.0)

        self.filtered_offset = (
            (1.0 - self.offset_filter_alpha) * self.filtered_offset
            + self.offset_filter_alpha * offset
        )
        self.filtered_heading_error = (
            (1.0 - self.offset_filter_alpha) * self.filtered_heading_error
            + self.offset_filter_alpha * raw_heading_error
        )
        raw_pid_error = self.select_pid_error()
        self.filtered_pid_error = (
            (1.0 - self.offset_filter_alpha) * self.filtered_pid_error
            + self.offset_filter_alpha * raw_pid_error
        )

        if self.control_mode == "stanley":
            target_yawdot = self.compute_stanley_yawdot()
        else:
            target_yawdot = self.compute_pid_yawdot(offset_is_fresh, dt)

        if self.control_mode != "pid":
            if boundary_margin < self.boundary_recovery_margin:
                target_yawdot *= 1.15
            boundary_violation = boundary_margin <= self.boundary_stop_margin
            if boundary_violation and confidence >= self.confidence_hold_threshold:
                recovery_error = self.filtered_offset
                if abs(recovery_error) > self.offset_deadband:
                    target_yawdot = -math.copysign(
                        self.max_yawdot * self.recovery_yaw_scale,
                        recovery_error,
                    )
                else:
                    target_yawdot = 0.0
            elif mode >= 3.0:
                recovery_offset = self.last_valid_offset
                if abs(recovery_offset) > self.offset_deadband:
                    target_yawdot = -math.copysign(
                        self.max_yawdot * self.lost_recovery_yaw_scale,
                        recovery_offset,
                    )
                else:
                    target_yawdot = 0.0

        if self.steering_pulse_enabled and abs(target_yawdot) > 0.01:
            self.steering_pulse_clock = (
                self.steering_pulse_clock + dt
            ) % self.steering_pulse_period
            if self.steering_pulse_clock > self.steering_pulse_on_time:
                target_yawdot = 0.0
        else:
            self.steering_pulse_clock = 0.0

        target_yawdot = max(-self.max_yawdot, min(self.max_yawdot, target_yawdot))

        max_step = self.max_yawdot_rate * dt
        yawdot_delta = max(-max_step, min(max_step, target_yawdot - self.last_yawdot))
        yawdot = self.last_yawdot + yawdot_delta
        self.last_yawdot = yawdot
        self.last_publish_time = now

        forward_scale = 1.0

        lateral_velocity = 0.0
        if self.enable_lateral_correction and state_is_fresh and mode < 3.0:
            lateral_error = self.apply_deadband(
                self.filtered_offset,
                self.lateral_deadband,
            )
            lateral_velocity = self.lateral_velocity_sign * self.lateral_velocity_gain * lateral_error
            if boundary_margin < self.boundary_warning_margin:
                lateral_velocity *= 1.0 + 0.6 * max(
                    0.0,
                    1.0 - boundary_margin / max(self.boundary_warning_margin, 1.0e-3),
                )
            lateral_velocity = max(
                -self.max_lateral_velocity,
                min(self.max_lateral_velocity, lateral_velocity),
            )

        msg = bxiMsg.MotionCommands()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"line_follow_{self.control_mode}"
        target_forward_vel = max(
            -self.max_amp_input_vx,
            min(self.max_amp_input_vx, self.forward_vel),
        )
        if self.max_amp_input_vx > 0.0:
            target_forward_vel = max(
                min(self.minimum_forward_command, self.max_amp_input_vx),
                target_forward_vel,
            )
        if self.forward_accel_limit > 0.0:
            max_forward_step = self.forward_accel_limit * dt
            forward_delta = max(
                -max_forward_step,
                min(max_forward_step, target_forward_vel - self.current_forward_vel),
            )
            self.current_forward_vel += forward_delta
        else:
            self.current_forward_vel = target_forward_vel
        msg.vel_des.x = float(self.current_forward_vel)
        msg.vel_des.y = float(lateral_velocity)
        msg.vel_des.z = 0.0
        msg.yawdot_des = float(
            max(-self.max_amp_input_yawdot, min(self.max_amp_input_yawdot, yawdot))
        )
        self.pub.publish(msg)

    def compute_stanley_yawdot(self):
        heading_rad = self.filtered_heading_error * (math.pi * 0.25)
        speed = max(abs(self.forward_vel), self.stanley_min_speed)
        crosstrack_term = math.atan2(
            self.stanley_crosstrack_gain * self.filtered_offset,
            speed,
        )
        steering = (
            self.stanley_heading_gain * heading_rad
            + crosstrack_term
        )
        return -self.stanley_yaw_gain * steering

    def compute_pid_yawdot(self, offset_is_fresh, dt):
        if offset_is_fresh:
            self.integral_error += self.filtered_pid_error * dt
        else:
            self.integral_error *= 0.90
        self.integral_error = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral_error),
        )

        raw_derivative_error = (self.filtered_pid_error - self.last_error) / dt
        self.last_error = self.filtered_pid_error
        self.filtered_derivative_error = (
            (1.0 - self.pid_derivative_filter_alpha) * self.filtered_derivative_error
            + self.pid_derivative_filter_alpha * raw_derivative_error
        )

        pid_output = (
            self.pid_kp * self.filtered_pid_error
            + self.pid_ki * self.integral_error
            + self.pid_kd * self.filtered_derivative_error
        )
        return -pid_output

    def select_pid_error(self):
        if self.pid_error_source == "offset":
            return self.filtered_offset
        if self.pid_error_source == "preview":
            return self.filtered_offset + self.pid_heading_feedforward_gain * self.filtered_heading_error
        if self.pid_error_source == "control":
            return self.apply_deadband(self.latest_control_error, self.control_deadband)
        return self.apply_deadband(self.filtered_heading_error, self.heading_deadband)

    def apply_deadband(self, value, deadband):
        if abs(value) <= deadband:
            return 0.0
        return value - deadband * (1.0 if value > 0.0 else -1.0)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

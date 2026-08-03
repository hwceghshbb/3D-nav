import select
import sys
import termios
import threading
import tty

import rclpy
from communication.msg import MotionCommands
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


HELP_TEXT = """\
Keyboard mapping teleop:
  w/s : set forward/backward velocity
  a/d : set left/right velocity
  q/e : set yaw left/right
  x   : clear linear velocity
  c   : clear yaw velocity
  space : full stop
  1   : normal/stand pulse
  2   : recover pulse
  ESC : exit teleop
"""


class KeyboardMotionTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_motion_teleop")
        self.declare_parameter("motion_commands_topic", "motion_commands")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("max_vx", 0.18)
        self.declare_parameter("max_vy", 0.08)
        self.declare_parameter("max_yaw", 0.25)
        self.declare_parameter("height_des", 1.0)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            MotionCommands,
            self.get_parameter("motion_commands_topic").value,
            qos,
        )

        self.lock = threading.Lock()
        self.vx = 0.0
        self.vy = 0.0
        self.yaw = 0.0
        self.btn_1_pulses = 0
        self.btn_6_pulses = 0
        self.stop_requested = False
        self.last_status = None

        period = 1.0 / max(float(self.get_parameter("publish_hz").value), 1.0)
        self.timer = self.create_timer(period, self.publish_command)
        self.thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.thread.start()
        self.get_logger().info("\n" + HELP_TEXT)

    def keyboard_loop(self):
        input_file = None
        close_input = False
        old_settings = None
        try:
            if sys.stdin.isatty():
                input_file = sys.stdin
            else:
                input_file = open("/dev/tty", "r")
                close_input = True
            old_settings = termios.tcgetattr(input_file)
            tty.setraw(input_file.fileno())
            while rclpy.ok() and not self.stop_requested:
                readable, _, _ = select.select([input_file], [], [], 0.1)
                if not readable:
                    continue
                key = input_file.read(1)
                if key == "\x1b":
                    with self.lock:
                        self.stop_requested = True
                    break
                self.handle_key(key)
        except Exception as exc:
            self.get_logger().error(f"keyboard teleop failed: {exc}")
        finally:
            if old_settings is not None and input_file is not None:
                termios.tcsetattr(input_file, termios.TCSADRAIN, old_settings)
            if close_input and input_file is not None:
                input_file.close()

    def handle_key(self, key):
        max_vx = float(self.get_parameter("max_vx").value)
        max_vy = float(self.get_parameter("max_vy").value)
        max_yaw = float(self.get_parameter("max_yaw").value)
        with self.lock:
            if key == "w":
                self.vx = max_vx
            elif key == "s":
                self.vx = -max_vx
            elif key == "a":
                self.vy = max_vy
            elif key == "d":
                self.vy = -max_vy
            elif key == "q":
                self.yaw = max_yaw
            elif key == "e":
                self.yaw = -max_yaw
            elif key == "x":
                self.vx = 0.0
                self.vy = 0.0
            elif key == "c":
                self.yaw = 0.0
            elif key == " ":
                self.vx = 0.0
                self.vy = 0.0
                self.yaw = 0.0
            elif key == "1":
                self.btn_1_pulses += 1
            elif key == "2":
                self.btn_6_pulses += 1
            else:
                return
            self.log_status_locked()

    def log_status_locked(self):
        status = (round(self.vx, 3), round(self.vy, 3), round(self.yaw, 3))
        if status != self.last_status:
            self.last_status = status
            self.get_logger().info(
                f"teleop command vx={status[0]:.3f} vy={status[1]:.3f} yaw={status[2]:.3f}"
            )

    def publish_command(self):
        with self.lock:
            msg = MotionCommands()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "keyboard_mapping_teleop"
            msg.vel_des.x = float(self.vx)
            msg.vel_des.y = float(self.vy)
            msg.yawdot_des = float(self.yaw)
            msg.height_des = float(self.get_parameter("height_des").value)
            if self.btn_1_pulses > 0:
                msg.btn_1 = 1
                self.btn_1_pulses -= 1
            if self.btn_6_pulses > 0:
                msg.btn_6 = 1
                self.btn_6_pulses -= 1
            should_stop = self.stop_requested
        self.publisher.publish(msg)
        if should_stop:
            rclpy.shutdown()

    def destroy_node(self):
        self.stop_requested = True
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardMotionTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

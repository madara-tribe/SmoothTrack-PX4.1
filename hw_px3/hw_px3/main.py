#!/usr/bin/env python3
import time
from custom_msgs.msg import AbsResult
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy)
from std_msgs.msg import Bool
from telemetrix import telemetrix

SERVO_PIN = 9

class AngleForwarder(Node):
    def __init__(self):
        super().__init__("px3")

        # --- Parameters (updated defaults) ---
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud", 9600)          # <- 115200bps
        self.declare_parameter("invert_angle", True)  # <- send 180-angle

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = int(self.get_parameter("baud").get_parameter_value().integer_value)
        self.invert_angle = bool(self.get_parameter("invert_angle").get_parameter_value().bool_value)

        # --- Serial open & center ---
        try:
            self.board = telemetrix.Telemetrix(com_port=port)
            self.board.set_pin_mode_servo(SERVO_PIN, 100, 3000)
            time.sleep(2.0)  # Arduino auto-reset
            self.get_logger().info(f'px3: opened serial {port} @ {baud}; centered to 90.0°')
        except (RuntimeError, OSError) as e:
            self.get_logger().error(f"px3: failed to open serial {port}: {e}")

        # --- Publish px3_ready (latched) ---
        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.ready_pub = self.create_publisher(Bool, "px3_ready", ready_qos)
        self.ready_pub.publish(Bool(data=True))
        self.get_logger().info("px3: published px3_ready=True")
        # --- Subscribe to setpoints (AbsResult) ---
        self.subscription = self.create_subscription(AbsResult, "inference", self._on_angle, 10)
        # --- Throttle/coalesce state ---
        self.last_send_t = time.monotonic()
        self.last_angle = None  # float degrees

    def _on_angle(self, msg: AbsResult):
        # get angle as float
        angle = int(msg.x_angle)
        if self.invert_angle:
            angle = 180 - angle

        now = time.monotonic()
        try:
            if self.board:
                self.board.servo_write(SERVO_PIN, angle)
        except (RuntimeError, OSError, AttributeError) as e:
            self.get_logger().error(f"px3: serial write failed: {e}")
            return

        self.last_send_t = now
        self.last_angle = angle

    def destroy_node(self):
        try:
            if hasattr(self, "board") and self.board:
                self.board.shutdown()
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = AngleForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


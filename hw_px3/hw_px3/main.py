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
        self.declare_parameter("center_deg", 90)
        self.declare_parameter("max_slew_deg_per_s", 180.0)   # limit step rate
        self.declare_parameter("deadband_deg", 0.6)            # ignore tiny errors
        self.declare_parameter("min_deg", 0)
        self.declare_parameter("max_deg", 180)

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = int(self.get_parameter("baud").get_parameter_value().integer_value)
        self.invert_angle = bool(self.get_parameter("invert_angle").get_parameter_value().bool_value)
        self.center_deg = float(self.get_parameter("center_deg").get_parameter_value().double_value)
        self.max_slew = float(self.get_parameter("max_slew_deg_per_s").get_parameter_value().double_value)
        self.deadband = float(self.get_parameter("deadband_deg").get_parameter_value().double_value)
        self.min_deg = float(self.get_parameter("min_deg").get_parameter_value().double_value)
        self.max_deg = float(self.get_parameter("max_deg").get_parameter_value().double_value)

        self.board = telemetrix.Telemetrix(com_port=port)
        self.board.set_pin_mode_servo(SERVO_PIN, 100, 3000)
        time.sleep(2.0)  # Arduino auto-reset
        # --- Serial open & center ---
        try:
            self.board.servo_write(SERVO_PIN, int(self.center_deg))
        except (RuntimeError, OSError) as e:
            pass
        self.get_logger().info(f'px3: opened serial {port} @ {baud}; centered to {self.center_deg:.1f}°')

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
        self.last_angle = None  # last commanded angle (deg)

    def _on_angle(self, msg: AbsResult):
        # get angle as float
        angle = int(msg.x_angle)
        if self.invert_angle:
            angle = 180 - angle

        # Clamp
        angle = max(self.min_deg, min(self.max_deg, angle))
        now = time.monotonic()
        dt = max(1e-3, now - self.last_send_t)

        # Slew limit + deadband
        if self.last_angle is None:
            cmd = angle
        else:
            delta = angle - self.last_angle
            if abs(delta) < self.deadband:
                return  # ignore tiny noise
            step = max(-self.max_slew * dt, min(self.max_slew * dt, delta))
            cmd = self.last_angle + step

        # rotate servo 
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


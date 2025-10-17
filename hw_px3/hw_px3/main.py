#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from custom_msgs.msg import AbsResult
import serial

def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

class AngleForwarder(Node):
    def __init__(self):
        super().__init__('px3')

        # --- Parameters (updated defaults) ---
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)          # <- 115200bps
        self.declare_parameter('min_interval_s', 0.02)  # <- faster follow
        self.declare_parameter('invert_angle', True)    # <- send 180-angle
        self.declare_parameter('deadband_deg', 2.0)     # <- ±2°

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = int(self.get_parameter('baud').get_parameter_value().integer_value)
        self.min_interval = float(self.get_parameter('min_interval_s').get_parameter_value().double_value)
        self.invert_angle = bool(self.get_parameter('invert_angle').get_parameter_value().bool_value)
        self.deadband_deg = float(self.get_parameter('deadband_deg').get_parameter_value().double_value)

        # --- Serial open & center ---
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
            time.sleep(2.0)  # Arduino auto-reset
            # center to 90.0 as float
            self.ser.write(b"90.0\n")
            self.get_logger().info(f'px3: opened serial {port} @ {baud}; centered to 90.0°')
        except Exception as e:
            self.get_logger().error(f'px3: failed to open serial {port}: {e}')
            self.ser = None

        # --- Publish px3_ready (latched) ---
        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.ready_pub = self.create_publisher(Bool, 'px3_ready', ready_qos)
        self.ready_pub.publish(Bool(data=True))
        self.get_logger().info('px3: published px3_ready=True')
        # (添付版と同じ役割) :contentReference[oaicite:3]{index=3}

        # --- Subscribe to setpoints (AbsResult) ---
        self.subscription = self.create_subscription(AbsResult, 'inference', self._on_angle, 10)

        # --- Throttle/coalesce state ---
        self.last_send_t = time.monotonic()
        self.last_angle  = None  # float degrees

    def _on_angle(self, msg: AbsResult):
        # get angle as float
        angle = float(msg.x_angle)
        if self.invert_angle:
            angle = 180.0 - angle
        angle = clamp(angle, 0.0, 180.0)

        # debounce by deadband
        if self.last_angle is not None:
            if abs(angle - self.last_angle) < self.deadband_deg:
                return

        now = time.monotonic()
        if (now - self.last_send_t) < self.min_interval:
            return

        # format as float string (1 decimal)
        payload = f"{angle:.1f}\n".encode('ascii')
        try:
            if self.ser:
                self.ser.write(payload)
                self.ser.flush()
        except Exception as e:
            self.get_logger().error(f'px3: serial write failed: {e}')
            return

        self.last_send_t = now
        self.last_angle  = angle

    def destroy_node(self):
        try:
            if hasattr(self, 'ser') and self.ser and self.ser.is_open:
                self.ser.close()
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


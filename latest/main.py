#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from custom_msgs.msg import AbsResult
import serial

class AngleForwarder(Node):
    def __init__(self):
        super().__init__('px3')

        # --- Parameters ---
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('min_interval_s', 0.10)  # min gap between serial writes
        self.declare_parameter('flip', True)            # True: send 180-angle; False: send angle as-is

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.min_interval = float(self.get_parameter('min_interval_s').get_parameter_value().double_value)
        self.flip = bool(self.get_parameter('flip').get_parameter_value().bool_value)

        # --- Serial open & center ---
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
        time.sleep(2.0)        # Arduino auto-reset
        self.ser.write(b"90\n")
        self.get_logger().info(f'px3: opened serial {port} @ {baud}; centered to 90°')

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
        # (Your current px3 already centers & publishes latched ready exactly like this.)  [oai_citation:2‡main.py](file-service://file-4mNUG95AUmDwceTbe4yx44)

        # --- Subscribe to absolute setpoints ---
        self.subscription = self.create_subscription(AbsResult, 'inference', self._on_angle, 10)

        # --- Throttle/coalesce state ---
        self.last_send_t = time.monotonic()
        self.last_servo  = None
        self.sent_count  = 0
        self.start_t     = self.last_send_t

    def _on_angle(self, msg: AbsResult):
        base = int(round(msg.x_angle))
        servo = 180 - base if self.flip else base
        if servo < 0: servo = 0
        if servo > 180: servo = 180

        now = time.monotonic()
        if (now - self.last_send_t) < self.min_interval:
            return
        if self.last_servo is not None and servo == self.last_servo:
            # unchanged setpoint: skip to reduce serial traffic
            return

        try:
            self.ser.write(f"{servo}\n".encode('ascii'))
        except Exception as e:
            self.get_logger().error(f'px3: serial write failed: {e}')
            return

        self.last_send_t = now
        self.last_servo  = servo
        self.sent_count += 1

        # quieter progress log
        if self.sent_count % 100 == 0:
            dt = max(1e-6, now - self.start_t)
            self.get_logger().info(f'px3: sent {self.sent_count} setpoints (~{self.sent_count/dt:.1f} Hz)')

    def destroy_node(self):
        try:
            if hasattr(self, 'ser') and self.ser.is_open:
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
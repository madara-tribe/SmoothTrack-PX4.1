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

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('min_interval_s', 0.10)  # wait 0.1 s after each send

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.min_interval = float(self.get_parameter('min_interval_s').get_parameter_value().double_value)

        # Open serial and center servo
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
        time.sleep(2.0)  # allow Arduino reset
        self.ser.write(b"90\n")
        self.get_logger().info(f'px3: opened serial {port} @ {baud}; centered to 90°')

        # Publish "ready" (latched)
        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.ready_pub = self.create_publisher(Bool, 'px3_ready', ready_qos)
        self.ready_pub.publish(Bool(data=True))
        self.get_logger().info('px3: published px3_ready=True')

        # Subscribe to absolute servo setpoints
        self.subscription = self.create_subscription(AbsResult, 'inference', self.callback, 10)

        self.msg_count = 0
        self.t0 = time.time()
        self.last_send = 0.0

    def callback(self, msg: AbsResult):
        servo = int(round(msg.x_angle))     # absolute target [0..180]
        if servo < 0: servo = 0
        if servo > 180: servo = 180

        now = time.time()
        if now - self.last_send < self.min_interval:
            return  # honor 0.1s gap

        try:
            self.ser.write(f"{servo}\n".encode('ascii'))
            self.last_send = now
        except Exception as e:
            self.get_logger().error(f'px3: serial write failed: {e}')
            return

        self.msg_count += 1
        if self.msg_count % 20 == 0:
            dt = time.time() - self.t0
            rate = self.msg_count / dt if dt > 0 else 0.0
            self.get_logger().info(f'px3: sent {self.msg_count} setpoints (~{rate:.1f} Hz)')

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

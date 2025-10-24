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
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from telemetrix import telemetrix

SERVO_PIN = 9
ARK_TIME = 0.2

class AngleForwarder(Node):
    def __init__(self):
        super().__init__("px3")

        # --- Parameters (updated defaults) ---
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud", 9600)          # <- 115200bps
        self.declare_parameter("center_deg", 90)

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = int(self.get_parameter("baud").get_parameter_value().integer_value)
        self.center_deg = float(self.get_parameter("center_deg").get_parameter_value().double_value)

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
        
        # --- ACK publisher (non-latched) ---
        self.ack_pub = self.create_publisher(Bool, "px3_ack", 10)

        # --- Subscribe to setpoints (AbsResult) ---
        self.subscription = self.create_subscription(AbsResult, "inference", self._on_angle, 10)
   
    def _on_angle(self, msg: AbsResult):
        #self.board.servo_write(SERVO_PIN, (180 - int(msg.x_angle)))
        self.board.servo_write(SERVO_PIN, int(msg.x_angle))
        time.sleep(ARK_TIME)
        # immediate ACK (or add a short sleep if you prefer to wait for motion)
        self.ack_pub.publish(Bool(data=True))

    def destroy_node(self):
        try:
            if hasattr(self, "board") and self.board:
                self.board.shutdown()
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = AngleForwarder()
    # Use 4 threads (Jetson Nano has 4 logical CPUs)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
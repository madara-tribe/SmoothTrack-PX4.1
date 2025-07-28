import rclpy
from rclpy.node import Node
from custom_msgs.msg import AbsResult
import serial
import time
from rclpy.parameter import Parameter

# Updated for Arduino Uno
BAUD_RATE = 115200

class ServoSubscriber(Node):
    def __init__(self):
        super().__init__('hw_px3_node')
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        port = self.get_parameter('arduino_port').get_parameter_value().string_value
        self.arduino_port = port
        try:
            self.ser = serial.Serial(self.arduino_port, BAUD_RATE, timeout=1)
            self.get_logger().info(f"Connected to Arduino Uno on {self.arduino_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None
        
        self.subscription = self.create_subscription(
            AbsResult,
            'inference',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        angle = int(90 + msg.x_angle)
        angle = max(0, min(180, angle))

        # For Arduino Uno, just send number and newline (no "x:" prefix)
        command = f"{angle}\n"

        self.get_logger().info(f'Send to Arduino: {command.strip()}')

        if self.ser:
            try:
                self.ser.write(command.encode())
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"Failed to send to Arduino: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ServoSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


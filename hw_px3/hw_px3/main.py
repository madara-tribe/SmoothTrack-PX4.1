import rclpy
from rclpy.node import Node
from custom_msgs.msg import AbsResult
import serial
import time
ARDUINO_PORT = '/dev/cu.usbserial-DK0FJVDT'
BAUD_RATE = 115200
class ServoSubscriber(Node):
    def __init__(self):
        super().__init__('hw_px3_node')

        # Connect to ESP8266 serial
        try:
            self.ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info("Connected to ESP8266 on /dev/ttyUSB0")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None

        # Subscribe to 'inference' topic
        self.subscription = self.create_subscription(
            AbsResult,
            'inference',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Clamp x_angle to 0–180 and convert to integer
        angle = int(90 + msg.x_angle)
        angle = max(0, min(180, angle))

        # Format for ESP8266: e.g., "x:120\n"
        command = f"x:{angle}\n"

        self.get_logger().info(f'Send to ESP8266: {command.strip()}')

        if self.ser:
            try:
                self.ser.write(command.encode())
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"Failed to send to ESP8266: {e}")

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

#!/usr/bin/env python3
import time
import threading
from queue import Queue, Full, Empty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from custom_msgs.msg import AbsResult

import serial

class AngleForwarder(Node):
    def __init__(self):
        super().__init__('px3')

        # ---- Parameters ----
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)  # use 115200 if your Arduino sketch matches
        self.declare_parameter('tx_queue_size', 512)

        self.port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud = int(self.get_parameter('baud').get_parameter_value().integer_value)
        self.qsize = int(self.get_parameter('tx_queue_size').get_parameter_value().integer_value)

        # ---- Subscribe ASAP (no blocking before this) ----
        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.subscription = self.create_subscription(
            AbsResult, 'inference', self.callback, qos)

        # ---- Non-blocking serial writer ----
        self.tx_queue = Queue(maxsize=self.qsize)
        self._writer_stop = threading.Event()
        self._writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer_thread.start()

        self.msg_count = 0
        self.t0 = time.time()
        self.get_logger().info('px3: subscriber ready; starting background serial writer...')

    # Background thread: open serial, wait for Arduino reset, then drain queue
    def _writer_loop(self):
        ser = None
        try:
            ser = serial.Serial(self.port, baudrate=self.baud, timeout=0.1)
            self.get_logger().info(f'px3: opened serial {self.port} @ {self.baud}')
            # Many Arduinos reset on open; give them time to boot.
            time.sleep(2.0)
        except Exception as e:
            self.get_logger().error(f'px3: serial open failed ({self.port}): {e}')
            # Keep trying every second until success or shutdown
            while not self._writer_stop.is_set() and ser is None:
                try:
                    time.sleep(1.0)
                    ser = serial.Serial(self.port, baudrate=self.baud, timeout=0.1)
                    self.get_logger().info(f'px3: opened serial {self.port} @ {self.baud}')
                    time.sleep(2.0)
                except Exception:
                    pass

        # Drain queue → serial
        while not self._writer_stop.is_set():
            try:
                line = self.tx_queue.get(timeout=0.2)
            except Empty:
                continue
            if ser is not None:
                try:
                    ser.write(line)
                except Exception as e:
                    self.get_logger().error(f'px3: serial write failed: {e}')
            # tiny pacing to avoid overwhelming very slow sketches
            time.sleep(0.002)  # 2 ms

        # cleanup
        try:
            if ser is not None and ser.is_open:
                ser.close()
        except Exception:
            pass

    def callback(self, msg: AbsResult):
        angle = int(round(msg.x_angle))
        if angle < 0: angle = 0
        if angle > 180: angle = 180
        line = f"{angle}\n".encode('ascii')

        try:
            self.tx_queue.put_nowait(line)
        except Full:
            # Drop the oldest and enqueue the latest (keep the stream current)
            try:
                _ = self.tx_queue.get_nowait()
                self.tx_queue.put_nowait(line)
                self.get_logger().warn('px3: TX queue full → dropping oldest message')
            except Exception:
                pass

        self.msg_count += 1
        if self.msg_count % 30 == 0:
            dt = time.time() - self.t0
            rate = self.msg_count / dt if dt > 0 else 0.0
            self.get_logger().info(f'px3: forwarded {self.msg_count} msgs (~{rate:.1f} Hz)')

    def destroy_node(self):
        self._writer_stop.set()
        if hasattr(self, '_writer_thread'):
            self._writer_thread.join(timeout=1.0)
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


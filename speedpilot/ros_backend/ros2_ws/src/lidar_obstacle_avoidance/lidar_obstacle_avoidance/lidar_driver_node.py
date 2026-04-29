"""LiDAR Driver Node.

Reads RPLidar scans from /dev/ttyUSB0 and publishes sensor_msgs/LaserScan on /scan.
The obstacle_avoidance_node subscribes to /scan and handles avoidance logic.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    from rplidar import RPLidar, RPLidarException
    RPLIDAR_AVAILABLE = True
except ImportError:
    RPLIDAR_AVAILABLE = False

LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUDRATE = 115200
NUM_BINS = 360


class LidarDriverNode(Node):

    def __init__(self):
        super().__init__('lidar_driver')
        self._pub = self.create_publisher(LaserScan, 'scan', 10)
        self._lidar = None
        self._running = True

        if not RPLIDAR_AVAILABLE:
            self.get_logger().error('rplidar library not installed — cannot start LiDAR driver')
            return

        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f'LiDAR driver started on {LIDAR_PORT}')

    def _scan_loop(self):
        while self._running and rclpy.ok():
            try:
                self._lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
                # Stop motor, flush serial RX buffer, then restart — prevents descriptor
                # parse errors (Incorrect starting bytes / length mismatch / wrong body size)
                # that occur when leftover scan bytes remain in the buffer from a prior session.
                self._lidar.stop()
                self._lidar.stop_motor()
                time.sleep(1.0)
                serial_port = getattr(self._lidar, '_serial', None)
                if serial_port is not None:
                    serial_port.reset_input_buffer()
                self._lidar.start_motor()
                time.sleep(1.0)
                self.get_logger().info('LiDAR connected')
                for scan in self._lidar.iter_scans():
                    if not self._running or not rclpy.ok():
                        break
                    self._publish_scan(scan)
            except RPLidarException as e:
                self.get_logger().warn(f'LiDAR error: {e} — reconnecting in 3s')
            except Exception as e:
                self.get_logger().error(f'LiDAR unexpected error: {e}')
            finally:
                self._disconnect()
            time.sleep(3)

    def _publish_scan(self, scan):
        # scan: list of (quality, angle_deg, distance_mm)
        ranges = [float('inf')] * NUM_BINS

        for quality, angle, distance_mm in scan:
            if quality == 0 or distance_mm == 0:
                continue
            dist_m = distance_mm / 1000.0
            if 0.15 < dist_m < 12.0:
                idx = int(angle) % NUM_BINS
                ranges[idx] = min(ranges[idx], dist_m)

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = 2 * math.pi / NUM_BINS
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / 5.5
        msg.range_min = 0.15
        msg.range_max = 12.0
        msg.ranges = ranges
        msg.intensities = []
        self._pub.publish(msg)

    def _disconnect(self):
        if self._lidar:
            try:
                self._lidar.stop()
                self._lidar.stop_motor()
                self._lidar.disconnect()
            except Exception:
                pass
            self._lidar = None

    def destroy_node(self):
        self._running = False
        self._disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

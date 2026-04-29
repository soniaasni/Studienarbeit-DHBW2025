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

# Serial port attribute names to probe (varies by rplidar library version)
_SERIAL_ATTRS = ('_serial', '_serial_port', 'serial_port', 'serial')


def _get_serial(lidar):
    for attr in _SERIAL_ATTRS:
        port = getattr(lidar, attr, None)
        if port is not None:
            return attr, port
    return None, None


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
        attempt = 0
        while self._running and rclpy.ok():
            attempt += 1
            self.get_logger().debug(f'Connection attempt #{attempt}')
            try:
                self._lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)

                attr_name, serial_port = _get_serial(self._lidar)
                if serial_port is not None:
                    self.get_logger().debug(f'Serial attr: {attr_name} — flushing immediately after open')
                    serial_port.reset_input_buffer()
                else:
                    self.get_logger().warn('No serial port attribute found — cannot flush RX buffer')

                # Stop any ongoing scan/motor so the device goes quiet
                self._lidar.stop()
                self._lidar.stop_motor()
                self.get_logger().debug('Sent stop commands, waiting 3 s for motor spindown')
                time.sleep(3.0)

                if serial_port is not None:
                    n = serial_port.in_waiting
                    self.get_logger().debug(f'Bytes in buffer after spindown: {n} — flushing')
                    serial_port.reset_input_buffer()

                # Basic communication check: get_info() sends one command and reads a
                # deterministic response, syncing the parser to a clean packet boundary.
                try:
                    info = self._lidar.get_info()
                    self.get_logger().info(
                        f'LiDAR device: model={info.get("model")}, '
                        f'firmware={info.get("firmware")}, hardware={info.get("hardware")}'
                    )
                except RPLidarException as e:
                    self.get_logger().warn(f'get_info() failed: {e} — baud rate mismatch?')
                    raise

                # Start motor and wait for operating speed
                self._lidar.start_motor()
                self.get_logger().debug('Motor started, waiting 2 s for spinup')
                time.sleep(2.0)

                self.get_logger().info('LiDAR connected — starting scan')
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

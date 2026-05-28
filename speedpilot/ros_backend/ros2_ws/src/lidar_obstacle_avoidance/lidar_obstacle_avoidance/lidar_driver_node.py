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
LIDAR_TIMEOUT = 3      # seconds — default 1 s causes false timeouts
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

    def _flush(self, serial_port, label=''):
        if serial_port is None:
            return
        n = serial_port.in_waiting
        serial_port.reset_input_buffer()
        self.get_logger().debug(f'Flushed {n} bytes [{label}]')

    def _scan_loop(self):
        attempt = 0
        while self._running and rclpy.ok():
            attempt += 1
            self.get_logger().debug(f'Connection attempt #{attempt}')
            try:
                # timeout=3 gives the device more time to respond to commands
                self._lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE, timeout=LIDAR_TIMEOUT)

                attr_name, serial_port = _get_serial(self._lidar)
                self.get_logger().debug(
                    f'Serial attr: {attr_name}' if serial_port else 'No serial attr found'
                )

                # Flush immediately: opening the port toggles DTR which may start the motor
                self._flush(serial_port, 'post-open')

                # Stop scan + motor, wait for full spindown, flush residue
                self._lidar.stop()
                self._lidar.stop_motor()
                self.get_logger().debug('Motor stopped — waiting 3 s for spindown')
                time.sleep(3.0)
                self._flush(serial_port, 'post-spindown')

                # Communication check: get_info() uses a simple request/response pair
                info = self._lidar.get_info()
                self.get_logger().info(
                    f'Device: model={info.get("model")}, '
                    f'firmware={info.get("firmware")}, hardware={info.get("hardware")}'
                )
                self._flush(serial_port, 'post-get_info')

                # Health check: iter_scans() calls this internally, but calling it here
                # first syncs the protocol and lets us log the result
                status, err_code = self._lidar.get_health()
                self.get_logger().info(f'Health: {status}, err_code={err_code}')
                self._flush(serial_port, 'post-get_health')

                # Start motor and wait for operating speed
                self._lidar.start_motor()
                self.get_logger().debug('Motor started — waiting 2 s for spinup')
                time.sleep(2.0)

                # Final flush: motor spinup may generate spurious bytes
                self._flush(serial_port, 'pre-scan')

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

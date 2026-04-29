"""Safety Stop Node.

Sits between the WebSocket bridge and the car controller.
Subscribes to:
  - vehicle_command_raw  (raw commands from the bridge / user input)
  - scan                 (LaserScan from the LiDAR driver)

Publishes to:
  - vehicle_command      (forwarded to the car controller)

Forward commands are passed through unchanged unless an obstacle is
detected within SAFETY_DISTANCE in the front sector. In that case the
speed is clamped to 0 so the car stops before hitting the obstacle.
Steering and backward motion are always passed through unmodified.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from custom_msgs.msg import VehicleCommand

SAFETY_DISTANCE = 0.5      # metres — stop if closer than this
FRONT_HALF_ANGLE_DEG = 30  # ±30 ° around the front of the car


class SafetyStopNode(Node):

    def __init__(self):
        super().__init__('safety_stop')

        self._front_clear = True

        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self.create_subscription(VehicleCommand, 'vehicle_command_raw', self._cmd_cb, 10)
        self._pub = self.create_publisher(VehicleCommand, 'vehicle_command', 10)

        self.get_logger().info(
            f'Safety stop node started '
            f'(distance={SAFETY_DISTANCE} m, front sector=±{FRONT_HALF_ANGLE_DEG}°)'
        )

    def _scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        # Front sector: ±FRONT_HALF_ANGLE_DEG around index 0 (0° = front)
        half_bins = round(FRONT_HALF_ANGLE_DEG * n / 360)
        front_indices = list(range(0, half_bins + 1)) + list(range(n - half_bins, n))

        front_distances = [
            msg.ranges[i]
            for i in front_indices
            if 0 <= i < n and math.isfinite(msg.ranges[i]) and msg.ranges[i] > 0.0
        ]

        if not front_distances:
            self._front_clear = True
            return

        min_dist = min(front_distances)
        was_clear = self._front_clear
        self._front_clear = min_dist > SAFETY_DISTANCE

        if not self._front_clear and was_clear:
            self.get_logger().warn(
                f'Obstacle at {min_dist:.2f} m — forward motion blocked'
            )
        elif self._front_clear and not was_clear:
            self.get_logger().info('Path clear — forward motion allowed')

    def _cmd_cb(self, msg: VehicleCommand):
        out = VehicleCommand()
        out.command = msg.command
        out.angle = msg.angle

        if msg.speed > 0.0 and not self._front_clear:
            out.speed = 0.0
            self.get_logger().debug(
                f'Forward command blocked (speed={msg.speed:.2f}): obstacle in front'
            )
        else:
            out.speed = msg.speed

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

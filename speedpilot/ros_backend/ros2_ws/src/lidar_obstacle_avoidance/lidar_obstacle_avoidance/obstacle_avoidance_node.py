import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import VehicleCommand

from lidar_obstacle_avoidance.gaussian_avoidance import GaussianAvoidanceController


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.controller = GaussianAvoidanceController()

        self.current_y = 0.0
        self.speed = 0.5

        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10
        )

        self.command_publisher = self.create_publisher(
            VehicleCommand,
            'vehicle_command',
            10
        )

        self.get_logger().info('Obstacle Avoidance Node mit Gaussian Controller gestartet.')

    def lidar_callback(self, msg: LaserScan):
        visible_obstacles = self.scan_to_obstacles(msg)

        plan = self.controller.update(
            current_y=self.current_y,
            visible_obstacles=visible_obstacles,
            speed=self.speed,
        )

        self.current_y = plan.next_y

        if plan.is_avoiding:
            speed = self.speed
            angle = math.radians(plan.steering_angle)
        else:
            speed = 0.0
            angle = 0.0

        self.publish_command(speed=speed, angle=angle)

        self.get_logger().info(
            f"obstacles={len(visible_obstacles)} | "
            f"avoiding={plan.is_avoiding} | "
            f"steering={plan.steering_angle:.1f} deg | "
            f"speed={speed:.2f} | "
            f"angle={angle:.2f} rad"
        )

    def scan_to_obstacles(self, msg: LaserScan):
        obstacles = []

        max_detection_distance = 2.0
        obstacle_width_deg = 5.0

        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue

            if distance < msg.range_min or distance > msg.range_max:
                continue

            if distance > max_detection_distance:
                continue

            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)

            if -90.0 <= angle_deg <= 90.0:
                obstacles.append(
                    (
                        angle_deg,
                        float(distance),
                        obstacle_width_deg,
                    )
                )

        return obstacles

    def publish_command(self, speed, angle):
        msg = VehicleCommand()
        msg.command = 'avoid'
        msg.speed = float(speed)
        msg.angle = float(angle)

        self.command_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleAvoidanceNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
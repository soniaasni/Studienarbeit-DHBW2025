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
        self.is_vehicle_moving = False
        self.last_move_speed = 0.0

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

        self.command_subscriber = self.create_subscription(
            VehicleCommand,
            'vehicle_command',
            self.vehicle_command_callback,
            10
        )

    def vehicle_command_callback(self, msg: VehicleCommand):
        self.get_logger().info(
            f"vehicle_command received: command={msg.command}, speed={msg.speed}"
        )

        # Eigene Avoid-Befehle ignorieren
        if msg.command == "avoid":
            return

        if msg.command == "move":
            self.last_move_speed = msg.speed
            self.is_vehicle_moving = msg.speed > 0.05
        else:
            self.last_move_speed = 0.0
            self.is_vehicle_moving = False

    def lidar_callback(self, msg: LaserScan):
        self.get_logger().info(
            f"is_vehicle_moving={self.is_vehicle_moving}"
        )

        if not self.is_vehicle_moving:
            self.controller.reset(current_y=self.current_y)
            return

        visible_obstacles = self.scan_to_obstacles(msg)

        if not visible_obstacles:
            self.controller.reset(current_y=self.current_y)
            self.get_logger().info(
                "No obstacle visible - avoidance reset"
            )
            return

        plan = self.controller.update(
            current_y=self.current_y,
            visible_obstacles=visible_obstacles,
            speed=max(self.last_move_speed, 0.1),
        )

        self.current_y = plan.next_y

        steering_rad = math.radians(plan.steering_angle)

        avoid_speed = min(self.last_move_speed, 0.30)
        avoid_speed = max(avoid_speed, 0.12)

        self.publish_command(
            speed=avoid_speed,
            angle=math.radians(plan.steering_angle)
)

        self.get_logger().info(
            f"Avoiding | obstacles={len(visible_obstacles)} | "
            f"steering={plan.steering_angle:.1f} deg | "
            f"speed={avoid_speed:.2f} | "
            f"angle={steering_rad:.2f} rad"
        )

    def scan_to_obstacles(self, msg: LaserScan):
        max_detection_distance = 1.2
        min_angle = -70.0
        max_angle = 70.0

        max_gap_deg = 4.0
        max_distance_jump = 0.25
        min_cluster_points = 2

        points = []

        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue

            if distance < msg.range_min or distance > msg.range_max:
                continue

            if distance > max_detection_distance:
                continue

            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)

            if min_angle <= angle_deg <= max_angle:
                points.append((angle_deg, float(distance)))

        if not points:
            return []

        clusters = []
        current = [points[0]]

        for angle, distance in points[1:]:
            last_angle, last_distance = current[-1]

            if (
                abs(angle - last_angle) <= max_gap_deg
                and abs(distance - last_distance) <= max_distance_jump
            ):
                current.append((angle, distance))
            else:
                clusters.append(current)
                current = [(angle, distance)]

        clusters.append(current)

        obstacles = []

        for cluster in clusters:
            if len(cluster) < min_cluster_points:
                continue

            angles = [p[0] for p in cluster]
            distances = [p[1] for p in cluster]

            center_angle = sum(angles) / len(angles)
            min_distance = min(distances)
            width_deg = max(max(angles) - min(angles), 8.0)

            obstacles.append(
                (
                    center_angle,
                    min_distance,
                    width_deg,
                )
            )

        obstacles.sort(key=lambda o: o[1])
        return obstacles[:3]

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
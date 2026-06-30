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
        if msg.command == 'move' and abs(msg.speed) > 0.05:
            self.is_vehicle_moving = True
        elif msg.command in ['stop', 'idle'] or abs(msg.speed) <= 0.05:
            self.is_vehicle_moving = False

    def lidar_callback(self, msg: LaserScan):
        if not self.is_vehicle_moving:
            self.controller.reset(current_y=self.current_y)
            return
        visible_obstacles = self.scan_to_obstacles(msg)
        self.get_logger().info(str(visible_obstacles))

        plan = self.controller.update(
            current_y=self.current_y,
            visible_obstacles=visible_obstacles,
            speed=self.speed,
        )

        self.current_y = plan.next_y

        if plan.is_avoiding:
            self.publish_command(
                speed=self.speed,
                angle=math.radians(plan.steering_angle)
            )

            self.get_logger().info(
                f"Avoiding | obstacles={len(visible_obstacles)} | "
                f"steering={plan.steering_angle:.1f} deg"
            )
        else:
            # Kein Hindernis: nichts senden.
            # Dadurch bleibt normale Steuerung aktiv.
            self.get_logger().info("No obstacle detected - normal control remains active.")

        self.get_logger().info(
            f"obstacles={len(visible_obstacles)} | "
            f"avoiding={plan.is_avoiding} | "
            f"steering={plan.steering_angle:.1f} deg | "
            f"speed={self.speed:.2f} | "
            f"angle={math.radians(plan.steering_angle):.2f} rad"
        )

    def scan_to_obstacles(self, msg: LaserScan):
        max_detection_distance = 0.8
        obstacle_width_deg = 20.0

        closest_distance = float('inf')
        closest_angle_deg = None

        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue

            if distance < msg.range_min or distance > msg.range_max:
                continue

            if distance > max_detection_distance:
                continue

            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)

            # Nur vorne betrachten
            if -45.0 <= angle_deg <= 45.0:
                if distance < closest_distance:
                    closest_distance = float(distance)
                    closest_angle_deg = angle_deg

        if closest_angle_deg is None:
            return []

        return [
            (
                closest_angle_deg,
                closest_distance,
                obstacle_width_deg,
            )
        ]

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
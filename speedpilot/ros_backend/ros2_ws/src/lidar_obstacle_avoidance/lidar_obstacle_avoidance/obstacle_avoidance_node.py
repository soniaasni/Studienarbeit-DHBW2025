import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import VehicleCommand

from lidar_obstacle_avoidance.gaussian_avoidance import GaussianAvoidanceController


class ObstacleAvoidanceNode(Node):
    # ROS2-Node, der LiDAR-Daten in Ausweichbefehle für das Fahrzeug umwandelt
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.controller = GaussianAvoidanceController()

        # Vereinfachte seitliche Position und letzter Fahrzustand des Fahrzeugs
        self.current_y = 0.0
        self.speed = 0.5
        self.is_vehicle_moving = False
        self.last_move_speed = 0.0

        # LiDAR-Scans liefern für Hinderniserkennung
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10
        )

        # Ausweichbefehle als VehicleCommand veröffentlichen
        self.command_publisher = self.create_publisher(
            VehicleCommand,
            'vehicle_command',
            10
        )

        self.get_logger().info('Obstacle Avoidance Node mit Gaussian Controller gestartet.')

        # Eingehende Fahrbefehle, um Bewegung und Geschwindigkeit zu verwenden
        self.command_subscriber = self.create_subscription(
            VehicleCommand,
            'vehicle_command',
            self.vehicle_command_callback,
            10
        )

    def vehicle_command_callback(self, msg: VehicleCommand):
        # Speichert normale Fahrbefehle
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
        # Verarbeitet LiDAR-Scan nur, wenn das Fahrzeug fährt
        self.get_logger().info(
            f"is_vehicle_moving={self.is_vehicle_moving}"
        )

        if not self.is_vehicle_moving:
            self.controller.reset(current_y=self.current_y)
            return

        visible_obstacles = self.scan_to_obstacles(msg)

        # Ohne sichtbares Hindernis wird kein Ausweichbefehl gesendet
        if not visible_obstacles:
            self.controller.reset(current_y=self.current_y)
            self.get_logger().info(
                "No obstacle visible - avoidance reset"
            )
            return

        # Controller berechnet den nächsten Lenkwinkel und die geschätzte Position.
        plan = self.controller.update(
            current_y=self.current_y,
            visible_obstacles=visible_obstacles,
            speed=max(self.last_move_speed, 0.1),
        )

        self.current_y = plan.next_y

        steering_rad = math.radians(plan.steering_angle)

        # Beim Ausweichen Geschwindigkeit begrenzen
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
        # Sucht das nächste gültige Hindernis im vorderen Sichtbereich
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
        # Erstellt und veröffentlicht Ausweichbefehl für den Fahrcontroller
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
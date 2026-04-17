import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import VehicleCommand  # Deine benutzerdefinierte Nachricht

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        # Abonniere LiDAR-Daten
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            'scan',  # Topic, auf dem LiDAR-Daten veröffentlicht werden
            self.lidar_callback,
            10
        )
        # Publisher für VehicleCommand (um das Auto zu steuern)
        self.command_publisher = self.create_publisher(
            VehicleCommand,
            'vehicle_command',  # Gleicher Topic wie der Controller verwendet
            10
        )
        self.get_logger().info('Obstacle Avoidance Node gestartet.')

    def lidar_callback(self, msg: LaserScan):
        # Verarbeite LiDAR-Daten
        ranges = msg.ranges  # Liste der Distanzen (in Metern)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Beispiel: Erkenne Hindernisse in einem bestimmten Bereich (z.B. vorne, links, rechts)
        front_distances = self.get_sector_distances(ranges, angle_min, angle_increment, -0.5, 0.5)  # -30° bis +30°
        left_distances = self.get_sector_distances(ranges, angle_min, angle_increment, 0.5, 1.5)   # +30° bis +90°
        right_distances = self.get_sector_distances(ranges, angle_min, angle_increment, -1.5, -0.5) # -90° bis -30°

        # Schwellenwert für Hindernisse (z.B. < 1m)
        obstacle_threshold = 1.0

        # Entscheidungslogik für Ausweichen
        if min(front_distances) < obstacle_threshold:
            if min(left_distances) > min(right_distances):
                # Ausweichen nach links
                self.publish_command(speed=0.5, angle=-0.5)  # Langsam fahren, nach links lenken
                self.get_logger().info('Hindernis vorne: Ausweichen nach links.')
            else:
                # Ausweichen nach rechts
                self.publish_command(speed=0.5, angle=0.5)
                self.get_logger().info('Hindernis vorne: Ausweichen nach rechts.')
        elif min(left_distances) < obstacle_threshold:
            # Hindernis links: Nach rechts lenken
            self.publish_command(speed=0.5, angle=0.5)
            self.get_logger().info('Hindernis links: Nach rechts lenken.')
        elif min(right_distances) < obstacle_threshold:
            # Hindernis rechts: Nach links lenken
            self.publish_command(speed=0.5, angle=-0.5)
            self.get_logger().info('Hindernis rechts: Nach links lenken.')
        else:
            # Kein Hindernis: Normal fahren (oder stoppen)
            self.publish_command(speed=0.0, angle=0.0)
            self.get_logger().info('Kein Hindernis: Stoppen.')

    def get_sector_distances(self, ranges, angle_min, angle_increment, min_angle, max_angle):
        """Extrahiere Distanzen für einen Winkelbereich."""
        distances = []
        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            if min_angle <= angle <= max_angle and distance > 0:  # Ignoriere ungültige Messungen
                distances.append(distance)
        return distances if distances else [float('inf')]

    def publish_command(self, speed, angle):
        """Veröffentliche VehicleCommand."""
        msg = VehicleCommand()
        msg.command = 'avoid'  # Oder ein passender String
        msg.speed = speed
        msg.angle = angle
        self.command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
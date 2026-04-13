import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.declare_parameter('robot_namespace', 'robot_03')
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 12.0)
        ns = self.get_parameter('robot_namespace').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value

        # QoS to match robot's publisher
        qos = QoSProfile(
            reliability=ReliabilityPolicy. RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.scan_pub = self.create_publisher(LaserScan, '/atom/scan', 10)

        self.get_logger().info(
            f'LiDAR Processor started | namespace: {ns} | '
            f'range: [{self.min_range}, {self.max_range}]m'
        )

    def scan_callback(self, msg):
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = self.min_range
        filtered.range_max = self.max_range

        filtered.ranges = [
            r if self.min_range <= r <= self.max_range else float('inf')
            for r in msg.ranges
        ]

        self.scan_pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
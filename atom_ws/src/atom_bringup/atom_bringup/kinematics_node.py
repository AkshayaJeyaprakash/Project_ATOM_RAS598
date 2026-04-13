import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math


class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        self.declare_parameter('wheel_radius', 0.0352)
        self.declare_parameter('wheelbase', 0.233)
        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheelbase').value
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.create_subscription(Odometry, '/odom', self.odom_callback, qos)

        self.odom_pub = self.create_publisher(Odometry, '/atom/odom', 10)
        self.icc_pub = self.create_publisher(Point, '/atom/icc', 10)

        self.get_logger().info(
            f'Kinematics Node started | r={self.r}m | L={self.L}m'
        )

    def odom_callback(self, msg):
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        v_r = v + (omega * self.L / 2.0)
        v_l = v - (omega * self.L / 2.0)

        v_computed = (v_r + v_l) / 2.0
        omega_computed = (v_r - v_l) / self.L

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0.0:
            return

        if abs(omega_computed) > 1e-6:
            R = v_computed / omega_computed
            icc_x = self.x - R * math.sin(self.theta)
            icc_y = self.y + R * math.cos(self.theta)

            old_x = self.x
            old_y = self.y
            omega_dt = omega_computed * dt

            self.x = (math.cos(omega_dt) * (old_x - icc_x)
                      - math.sin(omega_dt) * (old_y - icc_y)
                      + icc_x)
            self.y = (math.sin(omega_dt) * (old_x - icc_x)
                      + math.cos(omega_dt) * (old_y - icc_y)
                      + icc_y)
            self.theta = self.theta + omega_dt

        else:
            self.x += v_computed * math.cos(self.theta) * dt
            self.y += v_computed * math.sin(self.theta) * dt

        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.publish_odom(current_time)

        if abs(omega_computed) > 1e-6:
            self.publish_icc(icc_x, icc_y)

    def publish_odom(self, current_time):
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        self.odom_pub.publish(odom)
        self.get_logger().debug(
            f'Pose: x={self.x:.3f} y={self.y:.3f} theta={math.degrees(self.theta):.1f}°'
        )

    def publish_icc(self, icc_x, icc_y):
        point = Point()
        point.x = icc_x
        point.y = icc_y
        point.z = 0.0
        self.icc_pub.publish(point)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import json


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        self.declare_parameter('robot_namespace', 'robot_03')
        ns = self.get_parameter('robot_namespace').value

        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.create_subscription(
            String,
            '/exploration_goal',
            self.goal_callback,
            10
        )

        self.status_pub = self.create_publisher(
            String, '/atom/nav_status', 10
        )

        self.get_logger().info('Goal Publisher started — waiting for goals')

    def goal_callback(self, msg):
        try:
            goal_data = json.loads(msg.data)
            x = goal_data['x']
            y = goal_data['y']
            self.get_logger().info(f'Goal received: ({x:.2f}, {y:.2f})')
            self.send_nav2_goal(x, y)
        except Exception as e:
            self.get_logger().error(f'Goal parsing failed: {e}')

    def send_nav2_goal(self, x, y):
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('Nav2 not available — goal skipped')
            self.publish_status('NAV2_UNAVAILABLE')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending Nav2 goal: ({x:.2f}, {y:.2f})')
        self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self.publish_status('GOAL_REJECTED')
            return
        self.get_logger().info('Goal accepted by Nav2')
        self.publish_status('GOAL_ACCEPTED')
        goal_handle.get_result_async().add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):
        self.get_logger().info('Navigation complete')
        self.publish_status('GOAL_REACHED')

    def feedback_callback(self, feedback):
        remaining = feedback.feedback.distance_remaining
        self.get_logger().debug(f'Distance remaining: {remaining:.2f}m')

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
import json
import math


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self._current_goal_handle = None
        self._saved_goal = None
        self._is_paused = False
        self._is_approaching = False
        self._home_pose = None

        self._depth_client = self.create_client(Trigger, '/atom/get_depth')

        from nav2_msgs.srv import ClearEntireCostmap
        self._clear_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )
        self._clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )
        self._stop_distance_m = 0.6
        self._pending_spot = None
        self._depth_future = None
        self._pending_nav_goal = None

        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.create_subscription(String, '/exploration_goal', self.goal_callback, 10)
        self.create_subscription(String, '/atom/object_spotted', self.object_spotted_callback, 10)
        self.create_subscription(String, '/atom/resume_navigation', self.resume_callback, 10)
        self.create_subscription(String, '/atom/approach_goal', self.approach_goal_callback, 10)
        self.create_subscription(String, '/return_home', self.return_home_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, qos_transient)

        self.status_pub = self.create_publisher(String, '/atom/nav_status', 10)
        self.user_msg_pub = self.create_publisher(String, '/atom/user_message', 10)
        self.depth_bbox_pub = self.create_publisher(String, '/atom/depth_bbox', 10)
        self.distance_pub = self.create_publisher(Float32, '/atom/object_distance', 10)

        self.get_logger().info('GoalPublisher started | action: /navigate_to_pose')

        self.create_timer(0.1, self._depth_check_timer)

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance[0]
        if cov > 0.1:
            self._home_pose = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            )
            self.get_logger().info(
                f'Home pose stored: ({self._home_pose[0]:.2f}, {self._home_pose[1]:.2f})'
            )

    def goal_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            x = float(data['x'])
            y = float(data['y'])
            is_final = bool(data.get('final', False))
            self._is_approaching = False

            try:
                from nav2_msgs.srv import ClearEntireCostmap
                if self._clear_costmap_client.wait_for_service(timeout_sec=1.0):
                    self._clear_costmap_client.call_async(ClearEntireCostmap.Request())
                    self.get_logger().info('Local costmap cleared')
                if self._clear_global_costmap_client.wait_for_service(timeout_sec=1.0):
                    self._clear_global_costmap_client.call_async(ClearEntireCostmap.Request())
                    self.get_logger().info('Global costmap cleared')
            except Exception as ce:
                self.get_logger().warn(f'Costmap clear failed: {ce}')

            self.get_logger().info(f'Goal received: ({x:.2f}, {y:.2f}) final={is_final}')

            self._pending_nav_goal = (x, y, is_final)
            self.create_timer(1.5, self._send_pending_nav_goal)
        except Exception as e:
            self.get_logger().error(f'goal_callback failed: {e}')

    def _send_pending_nav_goal(self):
        if self._pending_nav_goal is None:
            return
        x, y, is_final = self._pending_nav_goal
        self._pending_nav_goal = None
        self.get_logger().info(f'Sending goal after costmap settle: ({x:.2f}, {y:.2f})')
        self._send_nav2_goal(x, y, is_final)

    def object_spotted_callback(self, msg: String):
        if self._is_paused:
            return
        try:
            data = json.loads(msg.data)
            obj_class = data.get('class', 'unknown')
            confidence = data.get('confidence', 0)
            bbox = data.get('bbox', None)

            if self._is_approaching:
                self.get_logger().info(
                    f'Object spotted during approach: {obj_class} ({confidence:.2f})'
                )
                if bbox is not None:
                    bbox_msg = String()
                    bbox_msg.data = json.dumps({'bbox': bbox, 'class': obj_class})
                    self.depth_bbox_pub.publish(bbox_msg)

                    self._pending_spot = {
                        'class': obj_class,
                        'confidence': confidence,
                        'bbox': bbox
                    }
                    if self._depth_client.wait_for_service(timeout_sec=0.5):
                        self._depth_future = self._depth_client.call_async(
                            Trigger.Request()
                        )
            else:
                self.get_logger().info(
                    f'Object spotted: {obj_class} ({confidence:.2f})'
                )
                self._is_paused = True
                self._cancel_current_goal()
                self._publish_status('PAUSED_FOR_VALIDATION')

                if bbox is not None:
                    bbox_msg = String()
                    bbox_msg.data = json.dumps({'bbox': bbox, 'class': obj_class})
                    self.depth_bbox_pub.publish(bbox_msg)

                    self._pending_spot = {
                        'class': obj_class,
                        'confidence': confidence,
                        'bbox': bbox
                    }
                    if self._depth_client.wait_for_service(timeout_sec=0.5):
                        self._depth_future = self._depth_client.call_async(
                            Trigger.Request()
                        )
                    else:
                        self.get_logger().warn('Depth service not available')
                        self._pending_spot = None

        except Exception as e:
            self.get_logger().error(f'object_spotted_callback failed: {e}')

    def _depth_check_timer(self):
        if self._depth_future is None or self._pending_spot is None:
            return
        if not self._depth_future.done():
            return

        try:
            response = self._depth_future.result()
            spot = self._pending_spot
            self._depth_future = None
            self._pending_spot = None

            obj_class = spot['class']

            if not response.success:
                self.get_logger().warn(
                    f'Depth service failed: {response.message}'
                )
                return

            distance_m = float(response.message)

            dist_msg = Float32()
            dist_msg.data = distance_m
            self.distance_pub.publish(dist_msg)

            self.get_logger().info(
                f'Distance to {obj_class}: {distance_m:.2f}m'
            )

            self.get_logger().info(
                f'Distance published: {distance_m:.3f}m'
            )

        except Exception as e:
            self.get_logger().error(f'_depth_check_timer failed: {e}')
            self._depth_future = None
            self._pending_spot = None

    def resume_callback(self, msg: String):
        if self._is_approaching:
            self.get_logger().info('Resume called during approach')
            return
        if self._saved_goal and self._is_paused:
            x, y = self._saved_goal
            self.get_logger().info(f'Resuming navigation to ({x:.2f}, {y:.2f})')
            self._is_paused = False
            self._send_nav2_goal(x, y)
        else:
            self.get_logger().info('Resume called')
            self._is_paused = False

    def approach_goal_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            x = float(data['x'])
            y = float(data['y'])
            yaw = float(data.get('yaw', 0.0))
            self.get_logger().info(
                f'Approach goal received: ({x:.2f}, {y:.2f})'
            )
            self._is_approaching = True
            self._is_paused = False

            try:
                from nav2_msgs.srv import ClearEntireCostmap
                if self._clear_costmap_client.wait_for_service(timeout_sec=0.5):
                    self._clear_costmap_client.call_async(ClearEntireCostmap.Request())
                    self.get_logger().info('Local costmap cleared')
            except Exception as ce:
                self.get_logger().warn(f'Costmap clear failed: {ce}')

            self._send_nav2_goal(x, y, is_final=False, yaw=yaw)
        except Exception as e:
            self.get_logger().error(f'approach_goal_callback failed: {e}')

    def return_home_callback(self, msg: String):
        if self._home_pose is None:
            self.get_logger().warn('No home pose stored yet')
            return
        x, y = self._home_pose
        self._is_approaching = False
        self.get_logger().info(f'Returning home: ({x:.2f}, {y:.2f})')
        self._send_nav2_goal(x, y)

    def _send_nav2_goal(self, x: float, y: float, is_final: bool = False, yaw: float = 0.0):
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('Nav2 action server not available')
            self._publish_status('NAV2_UNAVAILABLE')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._saved_goal = (x, y)
        self._is_paused = False

        self.get_logger().info(f'Sending Nav2 goal: ({x:.2f}, {y:.2f})')
        future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        future.add_done_callback(
            lambda f, final=is_final: self._goal_response_callback(f, final)
        )

    def _cancel_current_goal(self):
        if self._current_goal_handle is not None:
            self.get_logger().info('Cancelling current Nav2 goal')
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

    def _goal_response_callback(self, future, is_final: bool = False):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self._publish_status('GOAL_REJECTED')
            return

        self._current_goal_handle = goal_handle
        self.get_logger().info('Goal accepted by Nav2')
        self._publish_status('GOAL_ACCEPTED')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, final=is_final: self._result_callback(f, final)
        )

    def _result_callback(self, future, is_final: bool = False):
        self._current_goal_handle = None

        if self._is_paused:
            self.get_logger().info(
                'Navigation cancelled for validation'
            )
            return

        result = future.result()
        error_code = result.result.error_code

        if error_code == 0:
            self.get_logger().info('Navigation succeeded')
            self._publish_status('GOAL_REACHED')
            if is_final:
                self._publish_user_message(
                    'GOAL COMPLETED — Object found and reached.'
                )
        else:
            self.get_logger().warn(f'Navigation failed — error_code: {error_code}')
            self._publish_status('GOAL_REJECTED')

    def _feedback_callback(self, feedback):
        remaining = feedback.feedback.distance_remaining
        self.get_logger().debug(f'Distance remaining: {remaining:.2f}m')

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Nav status: {status}')

    def _publish_user_message(self, message: str):
        msg = String()
        msg.data = message
        self.user_msg_pub.publish(msg)
        self.get_logger().info(f'USER: {message}')


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
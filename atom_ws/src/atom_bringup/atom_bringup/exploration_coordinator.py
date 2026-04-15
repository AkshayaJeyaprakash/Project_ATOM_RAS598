import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import json
import math
import time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from .color_mapper import get_color_range
from .scan_point_generator import generate_scan_points, get_closest_scan_point


class ExplorationCoordinator(Node):
    def __init__(self):
        super().__init__('exploration_coordinator')

        self.state = 'IDLE'
        self.current_task = None
        self.color_range = None
        self.scan_points = []
        self.visited_scan_points = []
        self.current_scan_point = None
        self.rotation_angle = 0.0
        self.rotation_step = 15.0
        self.rotation_pause = 2.5
        self.last_rotation_time = 0.0
        self.is_paused = False
        self.scanning_active = False
        self.approach_direction = None
        self.approach_steps = 0
        self.max_approach_steps = 3
        self.approach_origin = None
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.declare_parameter('server_url', 'http://192.168.1.154:5000')
        self.server_url = self.get_parameter('server_url').value

        self.create_subscription(String, '/task_command', self.task_callback, 10)
        self.create_subscription(String, '/atom/semantic_map', self.map_callback, 10)
        self.create_subscription(String, '/atom/nav_status', self.nav_status_callback, 10)
        self.create_subscription(String, '/atom/color_detection', self.color_detection_callback, 10)
        self.create_subscription(Odometry, '/atom/odom', self.odom_callback, 10)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(OccupancyGrid, '/map', self.occupancy_map_callback, map_qos)

        self.goal_pub = self.create_publisher(String, '/exploration_goal', 10)
        self.status_pub = self.create_publisher(String, '/atom/task_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.scan_trigger_pub = self.create_publisher(String, '/atom/scan_trigger', 10)

        self.create_timer(0.1, self.rotation_timer)
        self.create_timer(60.0, self.exploration_fallback_timer)
        self.create_timer(5.0, self.try_generate_scan_points)

        self.get_logger().info('Exploration Coordinator started — state: IDLE')

    def task_callback(self, msg):
        if self.state != 'IDLE':
            self.get_logger().warn(f"Already on task '{self.current_task}' — ignoring")
            return

        self.current_task = msg.data
        self.color_range = get_color_range(self.current_task)
        self.visited_scan_points = []
        self.rotation_angle = 0.0
        self.scanning_active = False

        self.get_logger().info(
            f"Task: '{self.current_task}' | "
            f"Color filter: {'enabled' if self.color_range else 'disabled'}"
        )
        self.set_state('MOVING_TO_SCAN')
        self.go_to_next_scan_point()

    def occupancy_map_callback(self, msg):
        if not self.scan_points:
            points = generate_scan_points(msg, num_points=4)
            if points:
                self.scan_points = points
                self.get_logger().info(f"Scan points generated: {self.scan_points}")

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def map_callback(self, msg):
        if self.state not in ['SCANNING', 'APPROACHING', 'MOVING_TO_SCAN']:
            return
        if not self.current_task:
            return

        object_map = json.loads(msg.data)
        result = self.check_map_for_task(object_map, self.current_task)
        if result and result['x'] != 0.0 and result['y'] != 0.0:
            self.get_logger().info(
                f"Target found in map: {result['id']} at ({result['x']:.2f}, {result['y']:.2f})"
            )
            self.stop_rotation()
            self.scanning_active = False
            self.publish_goal(result['x'], result['y'])
            self.set_state('VERIFYING')

    def color_detection_callback(self, msg):
        if self.state != 'SCANNING' or not self.is_paused:
            return
        try:
            data = json.loads(msg.data)
            if data.get('detected'):
                self.get_logger().info(
                    f"Color blob detected! direction={data.get('direction')} "
                    f"ratio={data.get('ratio', 0):.3f}"
                )
                self.stop_rotation()
                self.scanning_active = False
                self.approach_direction = data.get('direction', 'center')
                self.approach_steps = 0
                self.approach_origin = (self.robot_x, self.robot_y)
                self.set_state('APPROACHING')
                self.do_approach_step()
        except Exception as e:
            self.get_logger().warn(f'Color detection parse error: {e}')

    def nav_status_callback(self, msg):
        status = msg.data
        self.get_logger().info(f"Nav status: {status}")

        if status == 'GOAL_REACHED':
            if self.state == 'MOVING_TO_SCAN':
                self.get_logger().info("At scan point — starting 360° scan")
                self.set_state('SCANNING')
                self.rotation_angle = 0.0
                self.is_paused = True
                self.last_rotation_time = time.time()
                self.scanning_active = True

            elif self.state == 'APPROACHING':
                trigger_msg = String()
                trigger_msg.data = json.dumps({
                    'task': self.current_task,
                    'mode': 'llava_verify',
                    'color_range': self.color_range
                })
                self.scan_trigger_pub.publish(trigger_msg)

            elif self.state == 'VERIFYING':
                self.get_logger().info("Arrived at target — DONE!")
                self.set_state('DONE')
                self.publish_status('DONE')
                self.current_task = None
                self.scan_points = []
                self.visited_scan_points = []
                self.set_state('IDLE')

        elif status in ['GOAL_REJECTED', 'NAV2_UNAVAILABLE']:
            if self.state == 'MOVING_TO_SCAN':
                self.go_to_next_scan_point()
            elif self.state == 'APPROACHING':
                self.return_to_scan_point()

    def rotation_timer(self):
        if self.state != 'SCANNING' or not self.scanning_active:
            return

        now = time.time()

        if self.is_paused:
            if now - self.last_rotation_time >= self.rotation_pause:
                if self.rotation_angle >= 360.0:
                    self.get_logger().info("360° scan complete — no object found here")
                    self.scanning_active = False
                    self.go_to_next_scan_point()
                    return
                self.is_paused = False
                self._do_rotate()
        else:
            twist = Twist()
            twist.angular.z = 0.3
            self.cmd_vel_pub.publish(twist)

    def _do_rotate(self):
        rotate_duration = math.radians(self.rotation_step) / 0.3
        end_time = time.time() + rotate_duration

        while time.time() < end_time:
            twist = Twist()
            twist.angular.z = 0.3
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self.stop_rotation()
        self.rotation_angle += self.rotation_step
        self.is_paused = True
        self.last_rotation_time = time.time()

        trigger_msg = String()
        trigger_msg.data = json.dumps({
            'task': self.current_task,
            'angle': self.rotation_angle,
            'color_range': self.color_range,
            'mode': 'scan'
        })
        self.scan_trigger_pub.publish(trigger_msg)
        self.get_logger().info(f"Scanning at {self.rotation_angle:.0f}°")

    def stop_rotation(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def do_approach_step(self):
        if self.approach_steps >= self.max_approach_steps:
            self.get_logger().warn("Max approach steps — false positive, returning")
            self.return_to_scan_point()
            return

        step_distance = 0.5
        offset = 0.0
        if self.approach_direction == 'left':
            offset = math.pi / 4
        elif self.approach_direction == 'right':
            offset = -math.pi / 4

        goal_x = self.robot_x + step_distance * math.cos(offset)
        goal_y = self.robot_y + step_distance * math.sin(offset)
        self.approach_steps += 1

        self.get_logger().info(
            f"Approach step {self.approach_steps}/{self.max_approach_steps} → ({goal_x:.2f}, {goal_y:.2f})"
        )
        self.publish_goal(goal_x, goal_y)

    def return_to_scan_point(self):
        if self.approach_origin:
            x, y = self.approach_origin
            self.get_logger().info(f"Returning to scan origin ({x:.2f}, {y:.2f})")
            self.publish_goal(x, y)
            self.set_state('MOVING_TO_SCAN')

    def try_generate_scan_points(self):
        if not self.scan_points:
            self.get_logger().info("Waiting for map to generate scan points...")

    def go_to_next_scan_point(self):
        unvisited = [p for p in self.scan_points if p not in self.visited_scan_points]

        if not unvisited:
            if not self.scan_points:
                import random
                x = random.uniform(-2.0, 2.0)
                y = random.uniform(-2.0, 2.0)
                self.get_logger().warn(f"No scan points yet — using random waypoint ({x:.2f}, {y:.2f})")
                self.publish_goal(x, y)
                return

            self.get_logger().warn("All scan points visited — object not found")
            self.publish_status('NOT_FOUND')
            self.current_task = None
            self.set_state('IDLE')
            return

        next_point = get_closest_scan_point(unvisited, self.robot_x, self.robot_y)
        self.visited_scan_points.append(next_point)
        self.current_scan_point = next_point
        self.get_logger().info(
            f"Moving to scan point {next_point} "
            f"({len(self.visited_scan_points)}/{len(self.scan_points)})"
        )
        self.publish_goal(next_point[0], next_point[1])

    def check_map_for_task(self, object_map, task):
        task_lower = task.lower()
        for obj in object_map:
            if obj['class'].lower() in task_lower and obj['status'] == 'present':
                return obj
        return None

    def exploration_fallback_timer(self):
        if self.state == 'SCANNING' and self.rotation_angle >= 360.0:
            self.get_logger().warn('Fallback timer triggered — moving to next scan point')
            self.scanning_active = False
            self.go_to_next_scan_point()

    def publish_goal(self, x, y):
        msg = String()
        msg.data = json.dumps({'x': x, 'y': y})
        self.goal_pub.publish(msg)
        self.get_logger().info(f"Goal → ({x:.2f}, {y:.2f})")

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def set_state(self, new_state):
        self.get_logger().info(f"State: {self.state} → {new_state}")
        self.state = new_state
        self.publish_status(new_state)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
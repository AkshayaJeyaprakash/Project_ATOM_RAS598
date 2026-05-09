import json
import time

import rclpy
from atom_bringup.config import BATTERY_CHECK_INTERVAL, LOW_BATTERY_THRESHOLD
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__("safety_monitor")
        self._battery_pct = 1.0
        self._low_battery = False
        self._task_active = False
        self._emergency_stopped = False
        self._dock_pending = False
        self._last_battery_warn = 0.0
        self._docking_in_progress = False
        self._dock_nav_pending = False
        self._initial_pose = None
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(BatteryState, "/battery_state", self._battery_callback, qos_sensor)
        self.create_subscription(String, "/atom/nav_status", self._dock_nav_status_callback, 10)
        self.create_subscription(String, "/atom/task_status", self._task_status_callback, 10)
        self.create_subscription(String, "/atom/emergency_stop", self._emergency_stop_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._initialpose_callback, 10)
        self.create_subscription(String, "/atom/dock_command", self._dock_command_callback, 10)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self._cmd_vel_final = self.create_publisher(Twist, "/cmd_vel", 10)
        self._task_status_pub = self.create_publisher(String, "/atom/task_status", 10)
        self._estop_pub = self.create_publisher(String, "/atom/emergency_stop", 10)
        self._nav_goal_pub = self.create_publisher(String, "/exploration_goal", 10)
        self.create_timer(BATTERY_CHECK_INTERVAL, self._battery_check_timer)
        self.create_timer(0.1, self._emergency_vel_timer)
        self.get_logger().info(
            f"SafetyMonitor started | battery threshold: {LOW_BATTERY_THRESHOLD*100:.0f}% | check interval: {BATTERY_CHECK_INTERVAL}s"
        )

    def _dock_nav_status_callback(self, msg: String):
        if not self._dock_nav_pending:
            return
        if msg.data == "GOAL_REACHED":
            self._dock_nav_pending = False
            self.get_logger().info("Reached initial pose — now calling dock action")
            self._call_dock_action()
        elif msg.data == "GOAL_REJECTED":
            self._dock_nav_pending = False
            self.get_logger().warn("Pre-dock navigation failed — retrying in 3 seconds")
            self.create_timer(3.0, self._retry_dock_nav)

    def _retry_dock_nav(self):
        if not self._docking_in_progress:
            return
        if self._initial_pose is None:
            self.get_logger().warn("No initial pose — docking from current position")
            self._call_dock_action()
            return
        x, y = self._initial_pose
        self.get_logger().info(f"Retrying navigation to ({x:.2f}, {y:.2f})...")
        nav_msg = String()
        nav_msg.data = json.dumps({"x": x, "y": y, "final": False})
        self._dock_nav_pending = True
        self._nav_goal_pub.publish(nav_msg)

    def _emergency_vel_timer(self):
        if not self._emergency_stopped:
            return
        zero = Twist()
        self._cmd_vel_pub.publish(zero)
        self._cmd_vel_final.publish(zero)

    def _battery_callback(self, msg: BatteryState):
        self._battery_pct = msg.percentage

    def _battery_check_timer(self):
        if self._emergency_stopped or self._docking_in_progress:
            return
        pct = self._battery_pct * 100.0
        if self._battery_pct < LOW_BATTERY_THRESHOLD:
            if not self._low_battery:
                self._low_battery = True
                self.get_logger().warn(f"LOW BATTERY: {pct:.1f}% < {LOW_BATTERY_THRESHOLD*100:.0f}% threshold")
            if self._task_active:
                now = time.time()
                if now - self._last_battery_warn > 30.0:
                    self._last_battery_warn = now
                    self.get_logger().warn(f"LOW BATTERY ({pct:.1f}%) — waiting for task to complete before docking")
                self._dock_pending = True
            else:
                self.get_logger().warn(f"LOW BATTERY ({pct:.1f}%) — docking now")
                self._initiate_dock()
        else:
            if self._low_battery:
                self.get_logger().info(f"Battery restored: {pct:.1f}% — clearing low battery flag")
            self._low_battery = False
            self._dock_pending = False

    def _initiate_dock(self):
        if self._docking_in_progress:
            return
        self._docking_in_progress = True
        if self._initial_pose is not None:
            x, y = self._initial_pose
            self.get_logger().info(f"Navigating to initial pose ({x:.2f}, {y:.2f}) before docking...")
            nav_msg = String()
            nav_msg.data = json.dumps({"x": x, "y": y, "final": False})
            self._dock_nav_pending = True
            self._nav_goal_pub.publish(nav_msg)
            self.get_logger().info("Waiting for robot to reach initial pose...")
            return
        self.get_logger().warn(
            "No initial pose set — docking from current position. Set 2D Pose Estimate in RViz first for accurate docking."
        )
        self._call_dock_action()

    def _call_dock_action(self):
        self.get_logger().info("Calling /dock action...")
        try:
            from irobot_create_msgs.action import Dock

            dock_client = ActionClient(self, Dock, "/dock")
            if not dock_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Dock action server not available — cannot autodock")
                self._docking_in_progress = False
                return
            goal = Dock.Goal()
            future = dock_client.send_goal_async(goal)
            future.add_done_callback(self._dock_goal_response)
            self.get_logger().info("Dock goal sent")
        except ImportError:
            self.get_logger().error("irobot_create_msgs not found — cannot autodock. Install: ros-jazzy-irobot-create-msgs")
            self._docking_in_progress = False

    def _dock_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Dock goal rejected")
            self._docking_in_progress = False
            return
        self.get_logger().info("Docking in progress...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._dock_result)

    def _dock_result(self, future):
        self._docking_in_progress = False
        try:
            _ = future.result()
            self.get_logger().info("Docking complete ✅")
        except Exception as e:
            self.get_logger().error(f"Docking failed: {e}")

    def _initiate_undock(self):
        self.get_logger().info("Initiating undock...")
        try:
            from irobot_create_msgs.action import Undock

            undock_client = ActionClient(self, Undock, "/undock")
            if not undock_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Undock action server not available")
                return
            goal = Undock.Goal()
            future = undock_client.send_goal_async(goal)
            future.add_done_callback(self._undock_goal_response)
            self.get_logger().info("Undock goal sent")
        except ImportError:
            self.get_logger().error("irobot_create_msgs not found — cannot undock")

    def _undock_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Undock goal rejected")
            return
        self.get_logger().info("Undocking in progress...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._undock_result)

    def _undock_result(self, future):
        try:
            _ = future.result()
            self.get_logger().info("Undocking complete ✅")
        except Exception as e:
            self.get_logger().error(f"Undocking failed: {e}")

    def _task_status_callback(self, msg: String):
        status = msg.data
        if any(s in status for s in ["SCANNING", "MOVING_TO_SCAN", "MEMORY_NAV", "CENTERING", "DEPTH_CHECK", "DRIVING_1M", "APPROACHING"]):
            self._task_active = True
        elif any(s in status for s in ["GOAL COMPLETED", "OBJECT_NOT_FOUND", "IDLE", "EMERGENCY_STOP"]):
            self._task_active = False
            if self._dock_pending and not self._emergency_stopped:
                self.get_logger().warn("Task complete — initiating pending autodock (low battery)")
                self._dock_pending = False
                self._initiate_dock()

    def _initialpose_callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._initial_pose = (x, y)
        self.get_logger().info(f"Initial pose updated: ({x:.2f}, {y:.2f}) — used as pre-dock target")

    def _dock_command_callback(self, msg: String):
        command = msg.data.strip().strip("'").strip('"').upper()
        if command == "DOCK":
            self.get_logger().info("Manual DOCK command received")
            self._initiate_dock()
        elif command == "UNDOCK":
            self.get_logger().info("Manual UNDOCK command received")
            self._initiate_undock()
        else:
            self.get_logger().warn(f'Unknown dock command: "{command}" — use DOCK or UNDOCK')

    def _emergency_stop_callback(self, msg: String):
        command = msg.data.strip().strip("'").strip('"').upper()
        if command == "STOP" and not self._emergency_stopped:
            self._emergency_stop()
        elif command == "RESUME" and self._emergency_stopped:
            self._emergency_resume()

    def _emergency_stop(self):
        self._emergency_stopped = True
        self.get_logger().error(
            "\n" + "!" * 60 + "\nEMERGENCY STOP ACTIVATED\nPublish \"RESUME\" to /atom/emergency_stop to resume\n" + "!" * 60
        )
        zero = Twist()
        for _ in range(10):
            self._cmd_vel_pub.publish(zero)
            self._cmd_vel_final.publish(zero)
        status_msg = String()
        status_msg.data = "EMERGENCY_STOP"
        self._estop_pub.publish(status_msg)
        self._task_status_pub.publish(status_msg)

    def _emergency_resume(self):
        self._emergency_stopped = False
        self.get_logger().info(
            "\n" + "=" * 60 + "\nEMERGENCY STOP CLEARED — Robot going to IDLE\nSend new command to /atom/resolved_target to start\n" + "=" * 60
        )
        status_msg = String()
        status_msg.data = "EMERGENCY_RESUME"
        self._estop_pub.publish(status_msg)
        self._task_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
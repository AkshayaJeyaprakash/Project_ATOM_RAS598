import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

IDLE = "IDLE"
APPROACHING = "APPROACHING"
EMERGENCY_STOP = "EMERGENCY_STOP"

DRIVE_SPEED = 0.12
LOST_TARGET_TIMEOUT_S = 0.5


class ExplorationCoordinator(Node):
    def __init__(self):
        super().__init__("exploration_coordinator")

        self._state = IDLE
        self._resolved_target = None
        self._last_spotted_data = None
        self._last_detection_time = 0.0
        self._latest_frame = None

        self.create_subscription(String, "/atom/resolved_target", self.resolved_target_callback, 10)
        self.create_subscription(Image, "/atom/camera/rgb", self.camera_callback, 10)
        self.create_subscription(String, "/atom/emergency_stop", self.emergency_stop_callback, 10)
        self.create_subscription(String, "/atom/object_spotted", self.object_spotted_callback, 10)

        self.status_pub = self.create_publisher(String, "/atom/task_status", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)

        self.create_timer(0.1, self._state_machine_tick)

        self.get_logger().info("ExplorationCoordinator started | state: IDLE")

    def resolved_target_callback(self, msg: String):
        self._resolved_target = msg.data.strip().lower()
        self._last_spotted_data = None
        self._last_detection_time = 0.0
        self._stop_robot()
        self._state = IDLE
        self._publish_status(f"TARGET {self._resolved_target}")
        self.get_logger().info(f'Resolved target set to "{self._resolved_target}"')

    def camera_callback(self, msg: Image):
        # Stored for later commits; not used yet.
        self._latest_frame = msg

    def emergency_stop_callback(self, msg: String):
        command = msg.data.strip().strip("'").strip('"').upper()

        if command == "EMERGENCY_STOP":
            self.get_logger().error("EMERGENCY STOP received — halting motion")
            self._stop_robot()
            self._state = EMERGENCY_STOP
            self._publish_status("EMERGENCY_STOP")

        elif command == "EMERGENCY_RESUME":
            self.get_logger().info("EMERGENCY RESUME received — returning to IDLE")
            self._full_reset()
            self._publish_status("IDLE")

    def object_spotted_callback(self, msg: String):
        if self._state == EMERGENCY_STOP:
            return

        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Could not parse object_spotted message: {e}")
            return

        detected_class = data.get("class", "").lower()
        if not detected_class or self._resolved_target is None:
            return

        if detected_class != self._resolved_target:
            return

        self._last_spotted_data = data
        self._last_detection_time = time.time()

        if self._state == IDLE:
            self.get_logger().info(f'Target "{detected_class}" spotted — moving forward')
            self._state = APPROACHING
            self._publish_status("APPROACHING")

    def _state_machine_tick(self):
        if self._state in [IDLE, EMERGENCY_STOP]:
            return

        if self._state == APPROACHING:
            now = time.time()

            if self._last_spotted_data is None:
                self._stop_robot()
                self._state = IDLE
                self._publish_status("IDLE")
                return

            if now - self._last_detection_time > LOST_TARGET_TIMEOUT_S:
                self.get_logger().info("Target lost — stopping")
                self._stop_robot()
                self._last_spotted_data = None
                self._state = IDLE
                self._publish_status("IDLE")
                return

            twist = Twist()
            twist.linear.x = DRIVE_SPEED
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

    def _stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def _full_reset(self):
        self._stop_robot()
        self._state = IDLE
        self._resolved_target = None
        self._last_spotted_data = None
        self._last_detection_time = 0.0
        self._latest_frame = None

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
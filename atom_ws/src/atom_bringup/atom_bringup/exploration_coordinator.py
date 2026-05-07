import json
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

IDLE = "IDLE"
CENTERING = "CENTERING"
APPROACHING = "APPROACHING"
EMERGENCY_STOP = "EMERGENCY_STOP"

FRAME_CENTER_X = 320.0
CENTER_THRESHOLD = 25.0
CONFIRM_COUNT = 5

KP = 0.003
MIN_ROT_SPEED = 0.12
MAX_ROT_SPEED = 0.8

DRIVE_SPEED = 0.12
LOST_TARGET_TIMEOUT_S = 0.5


class ExplorationCoordinator(Node):
    def __init__(self):
        super().__init__("exploration_coordinator")

        self._state = IDLE
        self._resolved_target = None
        self._last_spotted_data = None
        self._last_detection_time = 0.0
        self._center_confirm_count = 0
        self._latest_frame_bgr = None
        self._bridge = CvBridge()

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
        self._center_confirm_count = 0
        self._stop_robot()
        self._state = IDLE
        self._publish_status(f"TARGET {self._resolved_target}")
        self.get_logger().info(f'Resolved target set to "{self._resolved_target}"')

    def camera_callback(self, msg: Image):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            self._latest_frame_bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        except Exception:
            pass

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
            self.get_logger().info(f'Target "{detected_class}" spotted — centering')
            self._state = CENTERING
            self._center_confirm_count = 0
            self._publish_status("CENTERING")

        elif self._state == CENTERING:
            bbox = data.get("bbox")
            if bbox is not None:
                bbox_cx = (bbox[0] + bbox[2]) / 2.0
                self.get_logger().info(
                    f"Centering update | bbox_cx: {bbox_cx:.0f} | confirms: {self._center_confirm_count}/{CONFIRM_COUNT}"
                )

        elif self._state == APPROACHING:
            bbox = data.get("bbox")
            if bbox is not None:
                bbox_cx = (bbox[0] + bbox[2]) / 2.0
                drift = abs(bbox_cx - FRAME_CENTER_X)
                self.get_logger().info(f"Approaching update | bbox_cx: {bbox_cx:.0f} | drift: {drift:.0f}px")

    def _state_machine_tick(self):
        if self._state in [IDLE, EMERGENCY_STOP]:
            return

        if self._state == CENTERING:
            self._tick_centering()
        elif self._state == APPROACHING:
            self._tick_approaching()

    def _tick_centering(self):
        now = time.time()

        if self._last_spotted_data is None:
            self._stop_robot()
            self._state = IDLE
            self._publish_status("IDLE")
            return

        if now - self._last_detection_time > LOST_TARGET_TIMEOUT_S:
            self.get_logger().info("Target lost while centering — stopping")
            self._stop_robot()
            self._last_spotted_data = None
            self._center_confirm_count = 0
            self._state = IDLE
            self._publish_status("IDLE")
            return

        bbox = self._last_spotted_data.get("bbox")
        if bbox is None:
            return

        x1, y1, x2, y2 = bbox
        bbox_cx = (x1 + x2) / 2.0
        error = bbox_cx - FRAME_CENTER_X

        if abs(error) <= CENTER_THRESHOLD:
            self._stop_robot()
            self._center_confirm_count += 1
            self.get_logger().info(
                f"Centered! {self._center_confirm_count}/{CONFIRM_COUNT} | bbox_cx: {bbox_cx:.0f} | error: {error:.0f}px"
            )

            if self._center_confirm_count >= CONFIRM_COUNT:
                self.get_logger().info("Target centered — moving forward")
                self._state = APPROACHING
                self._publish_status("APPROACHING")
        else:
            self._center_confirm_count = 0
            omega = -KP * error
            omega = max(min(omega, MAX_ROT_SPEED), -MAX_ROT_SPEED)

            if 0 < abs(omega) < MIN_ROT_SPEED:
                omega = math.copysign(MIN_ROT_SPEED, omega)

            twist = Twist()
            twist.angular.z = omega
            self.cmd_vel_pub.publish(twist)

            self.get_logger().info(
                f"Centering | bbox_cx: {bbox_cx:.0f} | error: {error:.0f}px | omega: {math.degrees(omega):.1f}°/s"
            )

    def _tick_approaching(self):
        now = time.time()

        if self._last_spotted_data is None:
            self._stop_robot()
            self._state = IDLE
            self._publish_status("IDLE")
            return

        if now - self._last_detection_time > LOST_TARGET_TIMEOUT_S:
            self.get_logger().info("Target lost while approaching — stopping")
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
        self._center_confirm_count = 0
        self._latest_frame_bgr = None

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
import base64
import json
import math
import time

import cv2
import requests
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

IDLE = "IDLE"
CENTERING = "CENTERING"
DEPTH_CHECK = "DEPTH_CHECK"
DRIVING_1M = "DRIVING_1M"
APPROACHING = "APPROACHING"
DONE = "DONE"
EMERGENCY_STOP = "EMERGENCY_STOP"

FRAME_CENTER_X = 320.0
CENTER_THRESHOLD = 25.0
CONFIRM_COUNT = 5

KP = 0.003
MIN_ROT_SPEED = 0.12
MAX_ROT_SPEED = 0.8

DRIVE_SPEED = 0.12
DRIVE_1M_DIST = 1.0

LOST_TARGET_TIMEOUT_S = 0.5
DEPTH_CHECK_INTERVAL = 0.6
DEPTH_TIMEOUT_S = 5.0

STOP_DISTANCE_M = 0.55
STEREO_RELIABLE_M = 1.50
APPROACH_MAX_DIST_M = 2.50

DRIFT_THRESHOLD = 90.0

SERVER_URL = "http://localhost:8000"
CLIP_THRESHOLD = 0.25
VOTES_REQUIRED = 2


class ExplorationCoordinator(Node):
    def __init__(self):
        super().__init__("exploration_coordinator")

        self._state = IDLE
        self._resolved_target = None

        self._last_spotted_data = None
        self._last_detection_time = 0.0
        self._center_confirm_count = 0
        self._centering_start_time = 0.0
        self._centering_return_state = DEPTH_CHECK

        self._latest_frame_bgr = None
        self._bridge = CvBridge()

        self._depth_client = self.create_client(Trigger, "/atom/get_depth")
        self._depth_future = None
        self._depth_check_start = 0.0
        self._last_depth_check_time = 0.0
        self._latest_distance = None

        self._drive_start_time = 0.0
        self._drive_duration = DRIVE_1M_DIST / DRIVE_SPEED

        self.create_subscription(String, "/atom/resolved_target", self.resolved_target_callback, 10)
        self.create_subscription(Image, "/atom/camera/rgb", self.camera_callback, 10)
        self.create_subscription(String, "/atom/emergency_stop", self.emergency_stop_callback, 10)
        self.create_subscription(String, "/atom/object_spotted", self.object_spotted_callback, 10)

        self.status_pub = self.create_publisher(String, "/atom/task_status", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self.depth_bbox_pub = self.create_publisher(String, "/atom/depth_bbox", 10)

        self.create_timer(0.1, self._state_machine_tick)

        self.get_logger().info("ExplorationCoordinator started | state: IDLE")

    def resolved_target_callback(self, msg: String):
        raw_target = msg.data.strip().lower()
        self._full_reset()
        self._resolved_target = raw_target
        self._publish_status(f"TARGET {raw_target}")
        self.get_logger().info(f'Resolved target set to "{raw_target}"')

    def camera_callback(self, msg: Image):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            self._latest_frame_bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        except Exception:
            pass

    def emergency_stop_callback(self, msg: String):
        command = msg.data.strip().strip("'").strip('"').upper()

        if command == "EMERGENCY_STOP":
            self.get_logger().error("EMERGENCY STOP received — halting all motion")
            self._stop_robot()
            self._state = EMERGENCY_STOP
            self._publish_status("EMERGENCY_STOP")
        elif command == "EMERGENCY_RESUME":
            self.get_logger().info("EMERGENCY RESUME received — returning to IDLE")
            self._full_reset()
            self._publish_status("IDLE")

    def object_spotted_callback(self, msg: String):
        if self._state in [DONE, EMERGENCY_STOP]:
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
            self._start_centering(return_to=DEPTH_CHECK)

        elif self._state == CENTERING:
            bbox = data.get("bbox")
            if bbox is not None:
                bbox_cx = (bbox[0] + bbox[2]) / 2.0
                self.get_logger().info(
                    f"Centering update | bbox_cx: {bbox_cx:.0f} | confirms: {self._center_confirm_count}/{CONFIRM_COUNT}"
                )

        elif self._state in [DRIVING_1M, APPROACHING]:
            bbox = data.get("bbox")
            if bbox is not None:
                bbox_cx = (bbox[0] + bbox[2]) / 2.0
                drift = abs(bbox_cx - FRAME_CENTER_X)
                self.get_logger().info(f"Detection during drive | bbox_cx: {bbox_cx:.0f} | drift: {drift:.0f}px")

    def _state_machine_tick(self):
        if self._state == CENTERING:
            self._tick_centering()
        elif self._state == DEPTH_CHECK:
            self._tick_depth_check()
        elif self._state == DRIVING_1M:
            self._tick_driving_1m()
        elif self._state == APPROACHING:
            self._tick_approaching()

    def _start_centering(self, return_to: str):
        self._state = CENTERING
        self._center_confirm_count = 0
        self._centering_return_state = return_to
        self._centering_start_time = time.time()
        self.get_logger().info(f"CENTERING — will return to {return_to}")
        self._publish_status("CENTERING")

    def _tick_centering(self):
        now = time.time()

        if now - self._centering_start_time > 15.0:
            self.get_logger().warn("Centering timeout — back to IDLE")
            self._stop_robot()
            self._full_reset()
            self._publish_status("IDLE")
            return

        if self._last_spotted_data is not None and now - self._last_detection_time > LOST_TARGET_TIMEOUT_S:
            self.get_logger().warn("Target lost during centering — back to IDLE")
            self._stop_robot()
            self._last_spotted_data = None
            self._center_confirm_count = 0
            self._state = IDLE
            self._publish_status("IDLE")
            return

        if self._last_spotted_data is None:
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
                self.get_logger().info(
                    f'TARGET CENTERED | class: {self._last_spotted_data.get("class")} | bbox_cx: {bbox_cx:.0f}px | error: {error:.0f}px'
                )
                if self._centering_return_state == DEPTH_CHECK:
                    self._start_depth_check()
                elif self._centering_return_state == APPROACHING:
                    self._state = APPROACHING
                    self._last_depth_check_time = 0.0
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

    def _start_depth_check(self):
        self._state = DEPTH_CHECK
        self._depth_future = None
        self._depth_check_start = time.time()
        self.get_logger().info("DEPTH_CHECK — calling depth service")
        self._publish_status("DEPTH_CHECK")

        if self._last_spotted_data is not None:
            bbox_msg = String()
            bbox_msg.data = json.dumps(
                {
                    "bbox": self._last_spotted_data["bbox"],
                    "class": self._last_spotted_data.get("class", "unknown"),
                }
            )
            self.depth_bbox_pub.publish(bbox_msg)

        if not self._depth_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Depth service not available — back to IDLE")
            self._state = IDLE
            self._publish_status("IDLE")
            return

        self._depth_future = self._depth_client.call_async(Trigger.Request())

    def _tick_depth_check(self):
        if time.time() - self._depth_check_start > DEPTH_TIMEOUT_S:
            self.get_logger().warn("Depth check timeout — back to IDLE")
            self._depth_future = None
            self._state = IDLE
            self._publish_status("IDLE")
            return

        if self._depth_future is None or not self._depth_future.done():
            return

        try:
            response = self._depth_future.result()
            self._depth_future = None

            if not response.success:
                self.get_logger().warn(f"Depth failed: {response.message} — back to IDLE")
                self._state = IDLE
                self._publish_status("IDLE")
                return

            distance_m = float(response.message)
            self._latest_distance = distance_m
            self.get_logger().info(
                f"Depth: {distance_m:.3f}m | reliable: {STEREO_RELIABLE_M:.2f}m | stop: {STOP_DISTANCE_M:.2f}m"
            )

            if distance_m <= STOP_DISTANCE_M:
                self._declare_done(distance_m)
                return

            if distance_m <= STEREO_RELIABLE_M:
                self.get_logger().info(f"Near ({distance_m:.2f}m) — running validation before approach")
                if self._validate_with_clip_llava():
                    self.get_logger().info("Validation confirmed — starting approach")
                    self._state = APPROACHING
                    self._last_depth_check_time = 0.0
                else:
                    self.get_logger().warn("Validation failed — back to IDLE")
                    self._state = IDLE
                    self._publish_status("IDLE")
                return

            if distance_m <= APPROACH_MAX_DIST_M:
                self.get_logger().info(f"Safe zone ({distance_m:.2f}m) — driving 1m first")
                if self._validate_with_clip_llava():
                    self._start_driving_1m()
                else:
                    self.get_logger().warn("Validation failed — back to IDLE")
                    self._state = IDLE
                    self._publish_status("IDLE")
                return

            self.get_logger().info(f"Far ({distance_m:.2f}m) — driving 1m")
            self._start_driving_1m()

        except Exception as e:
            self.get_logger().error(f"Depth check failed: {e}")
            self._depth_future = None
            self._state = IDLE
            self._publish_status("IDLE")

    def _validate_with_clip_llava(self) -> bool:
        if self._latest_frame_bgr is None or self._resolved_target is None:
            self.get_logger().warn("Validation: no frame or target — defaulting to confirmed")
            return True

        votes = 1
        description = self._resolved_target

        try:
            frame_small = cv2.resize(self._latest_frame_bgr, (320, 240))
            _, buffer = cv2.imencode(".jpg", frame_small, [cv2.IMWRITE_JPEG_QUALITY, 80])
            img_b64 = base64.b64encode(buffer.tobytes()).decode()

            try:
                r = requests.post(
                    f"{self._server_url}/clip_score",
                    json={"image": img_b64, "text": f"a photo of a {description}"},
                    timeout=5,
                )
                clip_score = r.json().get("score", 0.0)
                clip_ok = clip_score >= CLIP_THRESHOLD
                if clip_ok:
                    votes += 1
                self.get_logger().info(
                    f"CLIP: score={clip_score:.4f} | threshold={CLIP_THRESHOLD} | ok={clip_ok}"
                )
            except Exception as e:
                self.get_logger().warn(f"CLIP failed: {e}")

            try:
                r = requests.post(
                    f"{self._server_url}/llava_reason",
                    json={"image": img_b64, "target": description},
                    timeout=15,
                )
                result = r.json()
                llava_ok = result.get("present", False)
                if llava_ok:
                    votes += 1
                self.get_logger().info(
                    f'LLaVA: present={llava_ok} | confidence={result.get("confidence")} | reason: {result.get("reasoning", "")}'
                )
            except Exception as e:
                self.get_logger().warn(f"LLaVA failed: {e}")

            confirmed = votes >= VOTES_REQUIRED
            self.get_logger().info(f"Validation: {votes}/3 votes | confirmed: {confirmed}")
            return confirmed

        except Exception as e:
            self.get_logger().error(f"_validate_with_clip_llava failed: {e}")
            return True

    def _start_driving_1m(self):
        self._state = DRIVING_1M
        self._drive_start_time = time.time()
        self.get_logger().info(f"DRIVING_1M — {DRIVE_1M_DIST}m at {DRIVE_SPEED}m/s ({self._drive_duration:.1f}s)")
        self._publish_status("DRIVING_1M")

    def _tick_driving_1m(self):
        now = time.time()
        elapsed = now - self._drive_start_time

        if self._last_spotted_data is not None:
            bbox = self._last_spotted_data.get("bbox")
            if bbox is not None:
                bbox_cx = (bbox[0] + bbox[2]) / 2.0
                drift = abs(bbox_cx - FRAME_CENTER_X)
                if drift > DRIFT_THRESHOLD:
                    self.get_logger().warn(f"Drift during 1m drive: {drift:.0f}px — re-centering")
                    self._stop_robot()
                    self._last_spotted_data = None
                    self._last_detection_time = 0.0
                    self._start_centering(return_to=DEPTH_CHECK)
                    return

        if elapsed >= self._drive_duration:
            self._stop_robot()
            self.get_logger().info("1m drive complete — re-centering")
            self._last_spotted_data = None
            self._last_detection_time = 0.0
            self._start_centering(return_to=DEPTH_CHECK)
            return

        twist = Twist()
        twist.linear.x = DRIVE_SPEED
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def _tick_approaching(self):
        now = time.time()

        if self._last_spotted_data is not None:
            bbox = self._last_spotted_data.get("bbox")
            if bbox is not None:
                bbox_cx = (bbox[0] + bbox[2]) / 2.0
                drift = abs(bbox_cx - FRAME_CENTER_X)
                if drift > DRIFT_THRESHOLD:
                    self.get_logger().warn(f"Drift during approach: {drift:.0f}px — re-centering")
                    self._stop_robot()
                    self._start_centering(return_to=APPROACHING)
                    return

        if now - self._last_depth_check_time >= DEPTH_CHECK_INTERVAL:
            self._last_depth_check_time = now

            if self._latest_distance is not None:
                self.get_logger().info(f"Approach depth: {self._latest_distance:.3f}m")
                if self._latest_distance <= STOP_DISTANCE_M:
                    self._stop_robot()
                    self._declare_done(self._latest_distance)
                    return

            if self._last_spotted_data is not None:
                bbox_msg = String()
                bbox_msg.data = json.dumps(
                    {
                        "bbox": self._last_spotted_data["bbox"],
                        "class": self._last_spotted_data.get("class", "unknown"),
                    }
                )
                self.depth_bbox_pub.publish(bbox_msg)

                if self._depth_client.wait_for_service(timeout_sec=0.2):
                    future = self._depth_client.call_async(Trigger.Request())
                    future.add_done_callback(self._approach_depth_callback)

        twist = Twist()
        twist.linear.x = DRIVE_SPEED
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def _approach_depth_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self._latest_distance = float(response.message)
                self.get_logger().info(f"Approach depth update: {self._latest_distance:.3f}m")
        except Exception as e:
            self.get_logger().warn(f"Approach depth callback: {e}")

    def _declare_done(self, distance_m: float):
        self._stop_robot()
        self._state = DONE
        self.get_logger().info(
            f"\n{'=' * 60}\nGOAL COMPLETED\nTarget   : {self._resolved_target}\nDistance : {distance_m:.3f}m\n{'=' * 60}"
        )
        self._publish_status(f"GOAL COMPLETED — {self._resolved_target} at {distance_m:.3f}m")
        self.create_timer(2.0, self._auto_reset)

    def _auto_reset(self):
        if self._state == DONE:
            self.get_logger().info("Auto reset — ready for next command")
            self._full_reset()

    def _stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def _full_reset(self):
        self._stop_robot()
        self._state = IDLE
        self._resolved_target = None
        self._last_spotted_data = None
        self._last_detection_time = 0.0
        self._center_confirm_count = 0
        self._centering_start_time = 0.0
        self._depth_future = None
        self._depth_check_start = 0.0
        self._last_depth_check_time = 0.0
        self._latest_distance = None
        self._drive_start_time = 0.0
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
import base64
import json
import math
import os
import time

import cv2
import numpy as np
import requests
import rclpy
from atom_bringup.config import (
    CLIP_THRESHOLD, CENTER_THRESHOLD, CONFIRM_COUNT, DEPTH_CHECK_INTERVAL, DRIVE_1M_DIST,
    DRIVE_SPEED, FRAME_CENTER_X, KP, MAX_ROT_SPEED, MEMORY_FILE, MIN_ROT_SPEED, SCAN_PAUSE_S,
    SCAN_SPEED, SCAN_SPACING_M, SCAN_STEP_DEG, SCAN_TOTAL_DEG, SERVER_URL, STEREO_RELIABLE_M,
    STOP_DISTANCE_M, VOTES_REQUIRED, WALL_CLEARANCE_M, DRIFT_THRESHOLD,
)
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

IDLE = "IDLE"
SCANNING = "SCANNING"
MOVING_TO_SCAN = "MOVING_TO_SCAN"
MEMORY_NAV = "MEMORY_NAV"
CENTERING = "CENTERING"
DEPTH_CHECK = "DEPTH_CHECK"
DRIVING_1M = "DRIVING_1M"
APPROACHING = "APPROACHING"
DONE = "DONE"
EMERGENCY_STOP = "EMERGENCY_STOP"


class ExplorationCoordinator(Node):
    def __init__(self):
        super().__init__("exploration_coordinator")
        self._state = IDLE
        self._resolved_target = None
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._map = None
        self._scan_points = []
        self._current_scan_idx = 0
        self._rotation_total = 0.0
        self._rotation_active = False
        self._rotation_pausing = False
        self._rotation_pause_end = 0.0
        self._rotating = False
        self._rotate_end_time = 0.0
        self._nav_status = None
        self._last_spotted_data = None
        self._center_confirm_count = 0
        self._last_detection_time = 0.0
        self._centering_start_time = 0.0
        self._centering_return_state = DEPTH_CHECK
        self._depth_future = None
        self._depth_check_start = 0.0
        self._latest_distance = None
        self._memory = {}
        self._memory_poses = []
        self._memory_pose_idx = 0
        self._memory_nav_status = None
        self._latest_frame_bgr = None
        self._bridge = CvBridge()
        self._server_url = SERVER_URL
        self._user_description = None
        self._drive_start_time = 0.0
        self._drive_duration = DRIVE_1M_DIST / DRIVE_SPEED
        self._last_depth_check_time = 0.0
        self._depth_client = self.create_client(Trigger, "/atom/get_depth")

        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(String, "/atom/resolved_target", self.resolved_target_callback, 10)
        self.create_subscription(Image, "/atom/camera/rgb", self.camera_callback, 10)
        self.create_subscription(String, "/atom/emergency_stop", self.emergency_stop_callback, 10)
        self.create_subscription(String, "/atom/object_spotted", self.object_spotted_callback, 10)
        self.create_subscription(String, "/atom/nav_status", self.nav_status_callback, 10)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, qos_transient)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_pose_callback, qos_transient)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.status_pub = self.create_publisher(String, "/atom/task_status", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self.depth_bbox_pub = self.create_publisher(String, "/atom/depth_bbox", 10)
        self.goal_pub = self.create_publisher(String, "/exploration_goal", 10)

        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.resolved_class_pub = self.create_publisher(String, "/atom/resolved_class", qos_latched)

        self.create_timer(0.1, self._state_machine_tick)
        self.get_logger().info("ExplorationCoordinator started | state: IDLE")

    def resolved_target_callback(self, msg: String):
        raw_task = msg.data.strip()
        self.get_logger().info(f"Task received: {raw_task}")
        self._full_reset()
        self._memory = self._load_memory()
        memory_list = ", ".join(sorted(self._memory.keys())) if self._memory else "memory is empty"
        yolo_class = raw_task.lower().strip()
        memory_object = "none"
        description = raw_task.lower().strip()

        try:
            r = requests.post(
                f"{self._server_url}/llava_parse_task",
                json={"task": raw_task, "memory_list": memory_list},
                timeout=30,
            )
            result = r.json()
            if result.get("yolo_class", "none") != "none":
                yolo_class = result["yolo_class"]
                memory_object = result.get("memory_object", "none")
                description = result.get("description", yolo_class)
                self.get_logger().info(
                    f'LLM parsed: yolo_class="{yolo_class}" | memory_object="{memory_object}" | description="{description}" | reason: {result.get("reasoning", "")}'
                )
            else:
                self.get_logger().warn(f'LLM returned none — falling back to raw task: "{raw_task}"')
        except Exception as e:
            self.get_logger().warn(f"LLM parse failed: {e} — using raw task as target")

        self._resolved_target = yolo_class
        self._user_description = description
        self.get_logger().info(f'Resolved: YOLO="{self._resolved_target}" | description="{self._user_description}"')

        clear_msg = String()
        clear_msg.data = ""
        self.resolved_class_pub.publish(clear_msg)

        resolved_msg = String()
        resolved_msg.data = yolo_class
        self.resolved_class_pub.publish(resolved_msg)

        poses_to_use = None
        poses_label = None
        if memory_object != "none" and memory_object in self._memory:
            poses_to_use = list(self._memory[memory_object])
            poses_label = memory_object
        elif yolo_class in self._memory and len(self._memory[yolo_class]) > 0:
            poses_to_use = list(self._memory[yolo_class])
            poses_label = yolo_class

        if poses_to_use:
            self._memory_poses = poses_to_use
            self._memory_pose_idx = 0
            self.get_logger().info(
                f'Memory hit: {len(self._memory_poses)} poses for "{poses_label}" | trying highest confidence first'
            )
            self._start_memory_nav()
        else:
            self.get_logger().info(f'No memory for "{yolo_class}" — starting exploration')
            self._start_scanning()

    def camera_callback(self, msg):
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
            self._stop_rotation()
            self._state = EMERGENCY_STOP
            self._publish_status("EMERGENCY_STOP")
        elif command == "EMERGENCY_RESUME":
            self.get_logger().info("EMERGENCY RESUME received — going to IDLE")
            self._full_reset()
            self._publish_status("IDLE")

    def object_spotted_callback(self, msg: String):
        if self._state in [DONE, EMERGENCY_STOP]:
            return
        try:
            data = json.loads(msg.data)
            if data.get("class", "").lower() != self._resolved_target:
                return
            self._last_spotted_data = data
            self._last_detection_time = time.time()
            if self._state in [SCANNING, MOVING_TO_SCAN, MEMORY_NAV]:
                self.get_logger().info("Target spotted — stopping, centering")
                self._stop_rotation()
                self._start_centering(return_to=DEPTH_CHECK)
            elif self._state == CENTERING:
                bbox_cx = (data["bbox"][0] + data["bbox"][2]) / 2.0
                self.get_logger().info(
                    f"Centering detection | bbox_cx: {bbox_cx:.0f} | confirms: {self._center_confirm_count}/{CONFIRM_COUNT}"
                )
            elif self._state in [DRIVING_1M, APPROACHING]:
                bbox_cx = (data["bbox"][0] + data["bbox"][2]) / 2.0
                drift = abs(bbox_cx - FRAME_CENTER_X)
                self.get_logger().info(f"Detection during drive | bbox_cx: {bbox_cx:.0f} | drift: {drift:.0f}px")
        except Exception as e:
            self.get_logger().error(f"object_spotted_callback failed: {e}")

    def nav_status_callback(self, msg: String):
        if self._state == MOVING_TO_SCAN:
            self._nav_status = msg.data
        elif self._state == MEMORY_NAV:
            self._memory_nav_status = msg.data

    def map_callback(self, msg: OccupancyGrid):
        if self._map is None:
            self._map = msg
            self.get_logger().info(f"Map received: {msg.info.width}x{msg.info.height} @ {msg.info.resolution}m/cell")

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny, cosy)

    def _state_machine_tick(self):
        if self._state == IDLE:
            return
        if self._state == SCANNING:
            self._tick_scanning()
        elif self._state == MOVING_TO_SCAN:
            self._tick_moving_to_scan()
        elif self._state == MEMORY_NAV:
            self._tick_memory_nav()
        elif self._state == CENTERING:
            self._tick_centering()
        elif self._state == DEPTH_CHECK:
            self._tick_depth_check()
        elif self._state == DRIVING_1M:
            self._tick_driving_1m()
        elif self._state == APPROACHING:
            self._tick_approaching()

    def _load_memory(self) -> dict:
        if os.path.exists(MEMORY_FILE):
            try:
                with open(MEMORY_FILE, "r") as f:
                    data = json.load(f)
                self.get_logger().info(f"Memory loaded | objects: {list(data.keys())}")
                return data
            except Exception as e:
                self.get_logger().warn(f"Failed to load memory: {e}")
        else:
            self.get_logger().info(f"No memory file found at {MEMORY_FILE}")
        return {}

    def _start_memory_nav(self):
        if self._memory_pose_idx >= len(self._memory_poses):
            self.get_logger().info(f"All {len(self._memory_poses)} memory poses exhausted — starting exploration")
            self._start_scanning()
            return
        pose = self._memory_poses[self._memory_pose_idx]
        x = pose["x"]
        y = pose["y"]
        conf = pose["confidence"]
        self.get_logger().info(
            f"MEMORY_NAV: pose {self._memory_pose_idx + 1}/{len(self._memory_poses)} | ({x:.2f}, {y:.2f}) | conf: {conf:.2f}"
        )
        self._state = MEMORY_NAV
        self._memory_nav_status = None
        self._publish_status(f"MEMORY_NAV {self._memory_pose_idx + 1}/{len(self._memory_poses)} ({x:.2f}, {y:.2f}) conf:{conf:.2f}")
        msg = String()
        msg.data = json.dumps({"x": x, "y": y, "final": False})
        self.goal_pub.publish(msg)

    def _tick_memory_nav(self):
        if self._memory_nav_status == "GOAL_REACHED":
            self._memory_nav_status = None
            self.get_logger().info(f"Memory pose {self._memory_pose_idx + 1} reached — scanning")
            self._start_scanning()
        elif self._memory_nav_status == "GOAL_REJECTED":
            self._memory_nav_status = None
            self.get_logger().warn(f"Memory pose {self._memory_pose_idx + 1} rejected — trying next")
            self._memory_pose_idx += 1
            self._start_memory_nav()

    def _validate_with_clip_llava(self) -> bool:
        if self._latest_frame_bgr is None or self._resolved_target is None:
            self.get_logger().warn("Validation: no frame or target — defaulting to confirmed")
            return True
        votes = 1
        description = self._user_description if self._user_description else self._resolved_target
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
                    f'CLIP: score={clip_score:.4f} | threshold={CLIP_THRESHOLD} | description="{description}" | ok={clip_ok}'
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
                    f'LLaVA: present={llava_ok} | description="{description}" | confidence={result.get("confidence")} | reason: {result.get("reasoning", "")}'
                )
            except Exception as e:
                self.get_logger().warn(f"LLaVA failed: {e}")
            confirmed = votes >= VOTES_REQUIRED
            self.get_logger().info(f"Validation: {votes}/3 votes | confirmed: {confirmed}")
            return confirmed
        except Exception as e:
            self.get_logger().error(f"_validate_with_clip_llava failed: {e}")
            return True

    def _start_scanning(self):
        self._state = SCANNING
        self._rotation_total = 0.0
        self._rotation_active = True
        self._rotation_pausing = False
        self._rotating = False
        self.get_logger().info(f"SCANNING at point {self._current_scan_idx}/{len(self._scan_points)} — {SCAN_TOTAL_DEG}° rotation")
        self._publish_status("SCANNING")

    def _tick_scanning(self):
        if not self._rotation_active:
            return
        now = time.time()
        if self._rotating:
            if now >= self._rotate_end_time:
                self._rotating = False
                self.cmd_vel_pub.publish(Twist())
                self._rotation_total += SCAN_STEP_DEG
                self._rotation_pausing = True
                self._rotation_pause_end = now + SCAN_PAUSE_S
            return
        if self._rotation_pausing:
            if now >= self._rotation_pause_end:
                self._rotation_pausing = False
                if self._rotation_total >= SCAN_TOTAL_DEG:
                    self.get_logger().info("Scan complete at this point — moving to next")
                    self._rotation_active = False
                    self._go_to_next_scan_point()
            return
        step_rad = math.radians(SCAN_STEP_DEG)
        duration = step_rad / SCAN_SPEED
        twist = Twist()
        twist.angular.z = SCAN_SPEED
        self.cmd_vel_pub.publish(twist)
        self._rotate_end_time = now + duration
        self._rotating = True

    def _stop_rotation(self):
        self._rotation_active = False
        self._rotation_pausing = False
        self._rotating = False
        self.cmd_vel_pub.publish(Twist())

    def _go_to_next_scan_point(self):
        if not self._scan_points:
            self._generate_scan_points()
            if not self._scan_points:
                self.get_logger().warn("No scan points generated — IDLE")
                self._publish_status("OBJECT_NOT_FOUND")
                self._state = IDLE
                return
        if self._current_scan_idx >= len(self._scan_points):
            self.get_logger().info(f"All {len(self._scan_points)} scan points visited — object not found")
            self._publish_status("OBJECT_NOT_FOUND")
            self._state = IDLE
            return
        x, y = self._scan_points[self._current_scan_idx]
        self._current_scan_idx += 1
        self._state = MOVING_TO_SCAN
        self._nav_status = None
        self.get_logger().info(f"Moving to scan point {self._current_scan_idx}/{len(self._scan_points)} → ({x:.2f}, {y:.2f})")
        self._publish_status(f"MOVING_TO_SCAN {self._current_scan_idx}/{len(self._scan_points)} ({x:.2f}, {y:.2f})")
        msg = String()
        msg.data = json.dumps({"x": x, "y": y, "final": False})
        self.goal_pub.publish(msg)

    def _tick_moving_to_scan(self):
        if self._nav_status == "GOAL_REACHED":
            self._nav_status = None
            self.get_logger().info("Scan point reached — starting scan")
            self._start_scanning()
        elif self._nav_status == "GOAL_REJECTED":
            self._nav_status = None
            self.get_logger().warn(f"Scan point {self._current_scan_idx} rejected — skipping to next")
            self._go_to_next_scan_point()

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
            self.get_logger().warn("Centering timeout — back to scanning")
            self._stop_robot()
            self._start_scanning()
            return
        if self._last_spotted_data is not None and now - self._last_detection_time > 4.0:
            self.get_logger().warn("Target lost during centering — back to scanning")
            self._stop_robot()
            self._last_spotted_data = None
            self._start_scanning()
            return
        if self._last_spotted_data is None:
            return
        bbox = self._last_spotted_data.get("bbox", None)
        if bbox is None:
            return
        x1, y1, x2, y2 = bbox
        bbox_cx = (x1 + x2) / 2.0
        error = bbox_cx - FRAME_CENTER_X
        if abs(error) <= CENTER_THRESHOLD:
            self._stop_robot()
            self._center_confirm_count += 1
            self.get_logger().info(f"Centered! {self._center_confirm_count}/{CONFIRM_COUNT} | bbox_cx: {bbox_cx:.0f} | error: {error:.0f}px")
            if self._center_confirm_count >= CONFIRM_COUNT:
                self.get_logger().info(
                    f'\n{"=" * 50}\nTARGET CENTERED | class: {self._last_spotted_data.get("class")} | bbox_cx: {bbox_cx:.0f}px | error: {error:.0f}px\n{"=" * 50}'
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
            self.get_logger().info(f"Centering | bbox_cx: {bbox_cx:.0f} | error: {error:.0f}px | omega: {math.degrees(omega):.1f}°/s")

    def _start_depth_check(self):
        self._state = DEPTH_CHECK
        self.get_logger().info("DEPTH_CHECK — calling depth service")
        self._publish_status("DEPTH_CHECK")
        if self._last_spotted_data is not None:
            bbox_msg = String()
            bbox_msg.data = json.dumps({"bbox": self._last_spotted_data["bbox"], "class": self._last_spotted_data.get("class", "unknown")})
            self.depth_bbox_pub.publish(bbox_msg)
        if not self._depth_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Depth service not available — back to scanning")
            self._start_scanning()
            return
        self._depth_future = self._depth_client.call_async(Trigger.Request())
        self._depth_check_start = time.time()

    def _tick_depth_check(self):
        if time.time() - self._depth_check_start > 5.0:
            self.get_logger().warn("Depth check timeout — back to scanning")
            self._depth_future = None
            self._start_scanning()
            return
        if self._depth_future is None or not self._depth_future.done():
            return
        try:
            response = self._depth_future.result()
            self._depth_future = None
            if not response.success:
                self.get_logger().warn(f"Depth failed: {response.message} — back to scanning")
                self._start_scanning()
                return
            distance_m = float(response.message)
            self._latest_distance = distance_m
            self.get_logger().info(f"Depth: {distance_m:.3f}m | reliable: {STEREO_RELIABLE_M}m | stop: {STOP_DISTANCE_M}m")
            if distance_m <= STOP_DISTANCE_M:
                self._declare_done(distance_m)
            elif distance_m <= STEREO_RELIABLE_M:
                self.get_logger().info(f"Near ({distance_m:.2f}m) — running CLIP+LLaVA validation")
                if self._validate_with_clip_llava():
                    self.get_logger().info("Validation confirmed — starting approach")
                    self._state = APPROACHING
                    self._last_depth_check_time = 0.0
                else:
                    self.get_logger().warn(f"Validation FAILED at {distance_m:.2f}m — back to scanning")
                    self._last_spotted_data = None
                    self._last_detection_time = 0.0
                    self._start_scanning()
            elif distance_m <= 2.5:
                self.get_logger().info(f"Safe zone ({distance_m:.2f}m) — running CLIP+LLaVA validation")
                if self._validate_with_clip_llava():
                    self.get_logger().info("Validation confirmed — driving 1m")
                    self._start_driving_1m()
                else:
                    self.get_logger().warn(f"Validation FAILED at {distance_m:.2f}m — back to scanning")
                    self._last_spotted_data = None
                    self._last_detection_time = 0.0
                    self._start_scanning()
            else:
                self.get_logger().info(f"Far ({distance_m:.2f}m) — driving 1m")
                self._start_driving_1m()
        except Exception as e:
            self.get_logger().error(f"Depth check failed: {e}")
            self._depth_future = None
            self._start_scanning()

    def _start_driving_1m(self):
        self._state = DRIVING_1M
        self._drive_start_time = time.time()
        self.get_logger().info(f"DRIVING_1M — {DRIVE_1M_DIST}m at {DRIVE_SPEED}m/s ({self._drive_duration:.1f}s)")
        self._publish_status("DRIVING_1M")

    def _tick_driving_1m(self):
        now = time.time()
        elapsed = now - self._drive_start_time
        if self._last_spotted_data is not None:
            bbox = self._last_spotted_data.get("bbox", None)
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
            bbox = self._last_spotted_data.get("bbox", None)
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
                bbox_msg.data = json.dumps({"bbox": self._last_spotted_data["bbox"], "class": self._last_spotted_data.get("class", "unknown")})
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
            f'\n{"=" * 60}\nGOAL COMPLETED\nTarget   : {self._resolved_target}\nDistance : {distance_m:.3f}m\n{"=" * 60}'
        )
        self._publish_status(f"GOAL COMPLETED — {self._resolved_target} at {distance_m:.3f}m")
        self.create_timer(2.0, self._auto_reset)

    def _auto_reset(self):
        if self._state == DONE:
            self.get_logger().info("Auto reset — ready for next command")
            self._full_reset()

    def _generate_scan_points(self):
        if self._map is None:
            self.get_logger().warn("No map yet — cannot generate scan points")
            return
        info = self._map.info
        data = np.array(self._map.data, dtype=np.int8).reshape(info.height, info.width)
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        clearance_cells = max(int(math.ceil(WALL_CLEARANCE_M / res)), 1)
        spacing_cells = max(int(round(SCAN_SPACING_M / res)), 1)
        self.get_logger().info(
            f"Generating scan points | map: {info.width}x{info.height} @ {res:.4f}m/cell | clearance: {clearance_cells} cells ({clearance_cells * res:.2f}m) | spacing: {spacing_cells} cells ({spacing_cells * res:.2f}m)"
        )
        free = (data == 0).astype(np.uint8)

        from scipy.ndimage import minimum_filter, label

        safe = minimum_filter(free, size=2 * clearance_cells + 1) == 1
        rows = range(clearance_cells, info.height - clearance_cells, spacing_cells)
        points_by_row = []
        grid_point_set = set()

        for row in rows:
            row_pts = []
            for col in range(clearance_cells, info.width - clearance_cells, spacing_cells):
                if not safe[row, col]:
                    continue
                wx = ox + col * res
                wy = oy + row * res
                row_pts.append((wx, wy))
                grid_point_set.add((row, col))
            points_by_row.append(row_pts)

        scan_points = []
        for i, row_pts in enumerate(points_by_row):
            if not row_pts:
                continue
            scan_points.extend(row_pts if i % 2 == 0 else reversed(row_pts))

        self.get_logger().info(f"Phase 1 (boustrophedon): {len(scan_points)} points")

        labeled, num_features = label(safe)
        self.get_logger().info(f"Phase 2: found {num_features} connected free regions")
        extra_points = []

        for region_id in range(1, num_features + 1):
            region_mask = labeled == region_id
            region_cells = np.argwhere(region_mask)
            if len(region_cells) < 256:
                self.get_logger().info(f"Phase 2: skipping region {region_id} ({len(region_cells)} cells) — too small")
                continue
            covered = any(region_mask[row, col] for row, col in grid_point_set)
            if covered:
                continue
            centroid_row = int(np.mean(region_cells[:, 0]))
            centroid_col = int(np.mean(region_cells[:, 1]))
            if safe[centroid_row, centroid_col]:
                best_row, best_col = centroid_row, centroid_col
            else:
                dists = np.sqrt((region_cells[:, 0] - centroid_row) ** 2 + (region_cells[:, 1] - centroid_col) ** 2)
                idx = np.argmin(dists)
                best_row = region_cells[idx, 0]
                best_col = region_cells[idx, 1]
            wx = ox + best_col * res
            wy = oy + best_row * res
            extra_points.append((wx, wy))
            self.get_logger().info(
                f"Phase 2: adding room centroid ({wx:.2f}, {wy:.2f}) for region {region_id} ({len(region_cells)} cells)"
            )

        self._scan_points = scan_points + extra_points
        self.get_logger().info(
            f"Total scan points: {len(self._scan_points)} ({len(scan_points)} grid + {len(extra_points)} room centroids) | robot at ({self._robot_x:.2f}, {self._robot_y:.2f})"
        )

    def _stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def _full_reset(self):
        self._stop_robot()
        self._stop_rotation()
        self._state = IDLE
        self._last_spotted_data = None
        self._center_confirm_count = 0
        self._last_detection_time = 0.0
        self._depth_future = None
        self._latest_distance = None
        self._rotation_total = 0.0
        self._nav_status = None
        self._scan_points = []
        self._current_scan_idx = 0
        self._memory_poses = []
        self._memory_pose_idx = 0
        self._memory_nav_status = None
        self._user_description = None

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
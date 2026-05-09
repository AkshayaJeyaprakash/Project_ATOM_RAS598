import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import requests
import base64
import json
import cv2
import time
import numpy as np

from atom_bringup.config import (
    CLIP_THRESHOLD, VOTES_REQUIRED,
    CONFIDENCE_THRESHOLD, DETECTION_INTERVAL,
    SPOTTED_COOLDOWN, TARGET_LOST_TIMEOUT,
    SERVER_URL
)


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.declare_parameter('server_url', SERVER_URL)
        self.declare_parameter('confidence_threshold', CONFIDENCE_THRESHOLD)

        self.server_url = self.get_parameter('server_url').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.bridge = CvBridge()

        cv2.namedWindow('ATOM - YOLO Detection', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('ATOM - YOLO Detection', 640, 480)

        self.latest_frame_bgr = None
        self.last_detection_time = 0.0
        self.detection_interval = DETECTION_INTERVAL

        self.current_task = None
        self.resolved_target = None

        self.spotted_cooldown = 0.0
        self.last_seen_time = 0.0
        self.target_lost_timeout = TARGET_LOST_TIMEOUT

        self.nav_active = False
        self._is_approaching = False

        self.create_subscription(Image, '/atom/camera/rgb',
                                 self.image_callback, 10)
        self.create_subscription(String, '/task_command',
                                 self.task_callback, 10)
        self.create_subscription(String, '/atom/nav_status',
                                 self.nav_status_callback, 10)

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.create_subscription(String, '/atom/resolved_class',
                                 self.resolved_target_callback, qos_latched)
        self.create_subscription(String, '/atom/task_status',
                                 self.task_status_callback, 10)

        self.detection_pub = self.create_publisher(String, '/atom/detections', 10)
        self.viz_pub = self.create_publisher(Image, '/atom/detection_viz', 10)
        self.object_spotted_pub = self.create_publisher(String, '/atom/object_spotted', 10)
        self.resume_pub = self.create_publisher(String, '/atom/resume_navigation', 10)

        self.create_timer(0.5, self._check_target_lost)

        self.get_logger().info(
            f'Object Detector started | server: {self.server_url} | '
            f'confidence threshold: {self.conf_threshold}'
        )

    def task_callback(self, msg):
        self.current_task = msg.data
        self.resolved_target = None
        self.spotted_cooldown = 0.0
        self._is_approaching = False
        self.get_logger().info(f'Task set: {self.current_task}')

    def resolved_target_callback(self, msg):
        value = msg.data.lower().strip()
        if not value:
            self.resolved_target = None
            self.spotted_cooldown = 0.0
            self.last_seen_time = 0.0
            return
        self.resolved_target = value
        self.spotted_cooldown = 0.0
        self.last_seen_time = 0.0
        self.get_logger().info(f'Resolved target: {self.resolved_target}')

    def nav_status_callback(self, msg):
        status = msg.data
        if status == 'GOAL_ACCEPTED':
            self.nav_active = True
        elif status in ['GOAL_REACHED', 'GOAL_REJECTED', 'NAV2_UNAVAILABLE',
                        'NOT_FOUND', 'DONE', 'IDLE']:
            self.nav_active = False

    def task_status_callback(self, msg):
        status = msg.data
        if 'SCANNING' in status or 'IDLE' in status or 'OBJECT_NOT_FOUND' in status:
            self._is_approaching = False
        elif 'APPROACHING' in status or 'MOVING_TO_SCAN' in status:
            self._is_approaching = True
        elif 'GOAL COMPLETED' in status or 'DONE' in status:
            self._is_approaching = False
            self.resolved_target = None
            self.spotted_cooldown = 0.0
            self.last_seen_time = 0.0
        elif 'EMERGENCY_RESUME' in status:
            self._is_approaching = False
            self.spotted_cooldown = 0.0
            self.last_seen_time = 0.0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.latest_frame_bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            now = time.time()
            if now - self.last_detection_time >= self.detection_interval:
                self.last_detection_time = now
                self.run_yolo_detection(self.latest_frame_bgr)
        except Exception as e:
            self.get_logger().error(f'Image callback failed: {e}')

    def run_yolo_detection(self, frame_bgr):
        try:
            frame_small = cv2.resize(frame_bgr, (320, 240))
            _, buffer = cv2.imencode('.jpg', frame_small,
                                     [cv2.IMWRITE_JPEG_QUALITY, 60])
            img_b64 = base64.b64encode(buffer.tobytes()).decode()

            r = requests.post(
                f'{self.server_url}/detect',
                json={'image': img_b64},
                timeout=5
            )
            detections = r.json()['detections']
            detections = [d for d in detections if d['confidence'] >= self.conf_threshold]

            display_h, display_w = frame_bgr.shape[:2]
            scale_x = display_w / 320.0
            scale_y = display_h / 240.0

            viz = frame_bgr.copy()
            for d in detections:
                x1, y1, x2, y2 = d['bbox']
                vx1 = int(x1 * scale_x)
                vy1 = int(y1 * scale_y)
                vx2 = int(x2 * scale_x)
                vy2 = int(y2 * scale_y)

                label = f"{d['class']} {d['confidence']:.2f}"
                color = (0, 255, 0)

                if self.resolved_target and d['class'].lower() == self.resolved_target:
                    color = (0, 0, 255)
                    label = f"TARGET: {label}"

                cv2.rectangle(viz, (vx1, vy1), (vx2, vy2), color, 2)
                cv2.putText(viz, label, (vx1, vy1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                self.get_logger().info(
                    f"Detected: {d['class']} ({d['confidence']:.2f})"
                )
                self._check_target_spotted(d)

            self.publish_viz(viz)

            if detections:
                msg = String()
                msg.data = json.dumps(detections)
                self.detection_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'YOLO detection failed: {e}')

    def _check_target_spotted(self, detection):
        if not self.resolved_target:
            return

        now = time.time()
        obj_class = detection['class'].lower()

        if obj_class == self.resolved_target:
            self.last_seen_time = now

            if now - self.spotted_cooldown < SPOTTED_COOLDOWN:
                return

            self.spotted_cooldown = now
            msg = String()
            msg.data = json.dumps({
                'class': detection['class'],
                'confidence': detection['confidence'],
                'bbox': detection['bbox'],
                'task': self.current_task
            })
            self.object_spotted_pub.publish(msg)

    def _check_target_lost(self):
        if self.last_seen_time == 0.0:
            return

        if self._is_approaching:
            return

        if time.time() - self.last_seen_time >= self.target_lost_timeout:
            self.last_seen_time = 0.0
            self.spotted_cooldown = 0.0
            msg = String()
            msg.data = 'resume'
            self.resume_pub.publish(msg)

    def publish_viz(self, frame_bgr):
        try:
            if self.resolved_target:
                cv2.putText(
                    frame_bgr,
                    f"Looking for: {self.resolved_target}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 255), 2
                )
            else:
                cv2.putText(
                    frame_bgr,
                    "Waiting for next command...",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 0), 2
                )

            cv2.imshow('ATOM - YOLO Detection', frame_bgr)
            cv2.waitKey(1)

            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            viz_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
            self.viz_pub.publish(viz_msg)
        except Exception as e:
            self.get_logger().warn(f'Viz publish failed: {e}')


def validate_with_clip_llava(server_url: str, frame_bgr, target: str, logger=None) -> dict:
    votes = 1
    details = {
        'yolo': True, 'clip': False, 'llava': False,
        'clip_score': 0.0, 'llava_present': False
    }

    try:
        frame_small = cv2.resize(frame_bgr, (320, 240))
        _, buffer = cv2.imencode('.jpg', frame_small,
                                 [cv2.IMWRITE_JPEG_QUALITY, 80])
        img_b64 = base64.b64encode(buffer.tobytes()).decode()

        try:
            r = requests.post(
                f'{server_url}/clip_score',
                json={'image': img_b64, 'text': f'a photo of a {target}'},
                timeout=5
            )
            clip_score = r.json().get('score', 0.0)
            details['clip_score'] = round(clip_score, 4)
            if clip_score >= CLIP_THRESHOLD:
                details['clip'] = True
                votes += 1
            if logger:
                logger.info(f'CLIP score: {clip_score:.4f}')
        except Exception as e:
            if logger:
                logger.warn(f'CLIP validation failed: {e}')

        try:
            r = requests.post(
                f'{server_url}/llava_reason',
                json={'image': img_b64, 'target': target},
                timeout=15
            )
            result = r.json()
            llava_present = result.get('present', False)
            details['llava_present'] = llava_present
            details['llava_confidence'] = result.get('confidence', 'low')
            details['llava_reasoning'] = result.get('reasoning', '')
            if llava_present:
                details['llava'] = True
                votes += 1
            if logger:
                logger.info(f'LLaVA result: {llava_present}')
        except Exception as e:
            if logger:
                logger.warn(f'LLaVA validation failed: {e}')

        confirmed = votes >= VOTES_REQUIRED
        if logger:
            logger.info(f'Validation result: {votes}/3')

        return {'confirmed': confirmed, 'votes': votes, 'details': details}

    except Exception as e:
        if logger:
            logger.error(f'validate_with_clip_llava failed: {e}')
        return {'confirmed': True, 'votes': 1, 'details': details}


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
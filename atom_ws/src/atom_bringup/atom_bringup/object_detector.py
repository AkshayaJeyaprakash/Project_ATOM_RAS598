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

from .color_mapper import color_detected


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.declare_parameter('server_url', 'http://192.168.1.154:5000')
        self.declare_parameter('confidence_threshold', 0.5)
        self.server_url = self.get_parameter('server_url').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.bridge = CvBridge()
        self.latest_frame_bgr = None
        self.last_detection_time = 0.0
        self.detection_interval = 1.0
        self.current_task = None
        self.current_color_range = None
        self.scan_mode = None

        cv2.namedWindow('ATOM - YOLO Detection', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('ATOM - YOLO Detection', 640, 480)
        
        self.create_subscription(Image, '/atom/camera/rgb', self.image_callback, 10)
        self.create_subscription(String, '/atom/scan_trigger', self.scan_trigger_callback, 10)
        self.create_subscription(String, '/task_command', self.task_callback, 10)

        self.detection_pub = self.create_publisher(String, '/atom/detections', 10)
        self.viz_pub = self.create_publisher(Image, '/atom/detection_viz', 10)
        self.color_detection_pub = self.create_publisher(String, '/atom/color_detection', 10)

        self.get_logger().info(f'Object Detector started | 'f'server: {self.server_url} | 'f'confidence threshold: {self.conf_threshold}')

    def task_callback(self, msg):
        self.current_task = msg.data

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

    def scan_trigger_callback(self, msg):
        if self.latest_frame_bgr is None:
            self.get_logger().warn('Scan triggered but no frame available')
            return
        try:
            data = json.loads(msg.data)
            self.current_task = data.get('task', self.current_task)
            self.current_color_range = data.get('color_range', None)
            self.scan_mode = data.get('mode', 'scan')
            self.run_color_filter(self.latest_frame_bgr, self.current_color_range)
        except Exception as e:
            self.get_logger().error(f'Scan trigger failed: {e}')

    def run_color_filter(self, frame_bgr, color_range):
        if color_range is None:
            result = {'detected': False, 'ratio': 0.0, 'direction': None}
            msg = String()
            msg.data = json.dumps(result)
            self.color_detection_pub.publish(msg)
            return

        result = color_detected(frame_bgr, color_range)

        self.get_logger().info(
            f"Color filter: detected={result['detected']} "
            f"ratio={result['ratio']} direction={result['direction']}"
        )

        msg = String()
        msg.data = json.dumps(result)
        self.color_detection_pub.publish(msg)
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        lower = np.array(color_range[0])
        upper = np.array(color_range[1])
        mask = cv2.inRange(hsv, lower, upper)
        viz = frame_bgr.copy()
        viz[mask > 0] = [0, 255, 0]
        self.publish_viz(viz)

    def run_yolo_detection(self, frame_bgr, force=False):
        try:
            _, buffer = cv2.imencode('.jpg', frame_bgr)
            img_b64 = base64.b64encode(buffer.tobytes()).decode()
            r = requests.post(
                f'{self.server_url}/detect',
                json={'image': img_b64},
                timeout=5
            )
            detections = r.json()['detections']
            detections = [d for d in detections if d['confidence'] >= self.conf_threshold]
            viz = frame_bgr.copy()
            for d in detections:
                x1, y1, x2, y2 = d['bbox']
                label = f"{d['class']} {d['confidence']:.2f}"
                cv2.rectangle(viz, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(viz, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                self.get_logger().info(f"Detected: {d['class']} ({d['confidence']:.2f})")

            self.publish_viz(viz)

            if detections:
                msg = String()
                msg.data = json.dumps(detections)
                self.detection_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'YOLO detection failed: {e}')

    def publish_viz(self, frame_bgr):
        try:
            cv2.imshow('ATOM - YOLO Detection', frame_bgr)
            cv2.waitKey(1)
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            viz_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
            self.viz_pub.publish(viz_msg)
        except Exception as e:
            self.get_logger().warn(f'Viz publish failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
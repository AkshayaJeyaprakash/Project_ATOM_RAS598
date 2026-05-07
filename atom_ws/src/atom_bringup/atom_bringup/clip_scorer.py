import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import requests
import base64
import json
import cv2


class ClipScorer(Node):
    def __init__(self):
        super().__init__('clip_scorer')

        self.declare_parameter('server_url', 'http://192.168.1.154:5000')
        self.declare_parameter('score_threshold', 0.2)

        self.server_url = self.get_parameter('server_url').value
        self.score_threshold = self.get_parameter('score_threshold').value
        self.bridge = CvBridge()

        self.current_task = None
        self.create_subscription(
            Image,
            '/atom/camera/rgb',
            self.image_callback,
            10
        )

        self.create_subscription(
            String,
            '/task_command',
            self.task_callback,
            10
        )

        self.score_pub = self.create_publisher(
            Float32, '/atom/clip_score', 10
        )

        self.ranking_pub = self.create_publisher(
            String, '/atom/frontier_ranking', 10
        )

        self.get_logger().info(
            f'CLIP Scorer started | '
            f'server: {self.server_url} | '
            f'score threshold: {self.score_threshold}'
        )

    def task_callback(self, msg):
        self.current_task = msg.data
        self.get_logger().info(f'Task updated: {self.current_task}')

    def image_callback(self, msg):
        if not self.current_task:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            _, buffer = cv2.imencode('.jpg', cv_image)
            image_bytes = buffer.tobytes()

            score = self.get_clip_score(image_bytes, self.current_task)

            score_msg = Float32()
            score_msg.data = score
            self.score_pub.publish(score_msg)

            if score >= self.score_threshold:
                ranking = {
                    'task': self.current_task,
                    'score': score,
                    'recommendation': 'explore_this_direction'
                }
                msg_out = String()
                msg_out.data = json.dumps(ranking)
                self.ranking_pub.publish(msg_out)
                self.get_logger().info(
                    f'CLIP score for "{self.current_task}": {score:.4f} '
                    f'— above threshold, recommend exploring'
                )

        except Exception as e:
            self.get_logger().error(f'CLIP scoring failed: {e}')

    def get_clip_score(self, image_bytes, text):
        img_b64 = base64.b64encode(image_bytes).decode()
        r = requests.post(
            f'{self.server_url}/clip_score',
            json={'image': img_b64, 'text': text},
            timeout=5
        )
        return r.json()['score']


def main(args=None):
    rclpy.init(args=args)
    node = ClipScorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
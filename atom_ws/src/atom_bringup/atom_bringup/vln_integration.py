import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
import requests
import json


class VLNIntegration(Node):
    def __init__(self):
        super().__init__('vln_integration')

        self.declare_parameter('server_url', 'http://192.168.1.154:5000')
        self.server_url = self.get_parameter('server_url').value

        self.create_subscription(Float32, '/atom/clip_score', self.clip_score_callback, 10)
        self.create_subscription(String, '/atom/frontier_ranking', self.frontier_callback, 10)
        self.create_subscription(String, '/atom/detections', self.detection_callback, 10)

        self.decision_pub = self.create_publisher(String, '/atom/vln_decision', 10)

        self._test_connection()
        self.get_logger().info('VLN Integration started — coordinating nodes')

    def _test_connection(self):
        try:
            r = requests.get(f'{self.server_url}/health', timeout=3)
            self.get_logger().info(f'Inference server OK: {r.json()}')
        except Exception as e:
            self.get_logger().error(f'Cannot reach inference server: {e}')

    def clip_score_callback(self, msg):
        score = msg.data
        self.get_logger().debug(f'CLIP score received: {score:.4f}')

    def frontier_callback(self, msg):
        ranking = json.loads(msg.data)
        self.get_logger().info(
            f'Frontier ranking received: score={ranking["score"]:.4f}'
        )
        decision = String()
        decision.data = json.dumps({
            'action': 'explore',
            'score': ranking['score'],
            'task': ranking['task']
        })
        self.decision_pub.publish(decision)

    def detection_callback(self, msg):
        detections = json.loads(msg.data)
        self.get_logger().info(f'Detections received: {len(detections)} objects')


def main(args=None):
    rclpy.init(args=args)
    node = VLNIntegration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
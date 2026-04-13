import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import cv2

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.declare_parameter('robot_namespace', 'robot_03')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        ns = self.get_parameter('robot_namespace').value
        self.bridge = CvBridge()

        # QoS to match robot's publisher
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, qos)
        self.create_subscription(Image, '/oakd/stereo/image_raw', self.depth_callback, qos)
        self.rgb_pub = self.create_publisher(Image, '/atom/camera/rgb', 10)
        self.depth_pub = self.create_publisher(Image, '/atom/camera/depth', 10)
        self.get_logger().info(f'Camera Processor started | namespace: {ns}')

    
    def image_callback(self, msg):
        """
        Processes an incoming RGB image and republishes it.

        Input:
            msg: ROS Image message (RGB format)

        Output:
            Publishes the processed image to self.rgb_pub

        Description:
            Converts the ROS image message to an OpenCV image and back,
            then republishes it while keeping the original header.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
            processed_msg.header = msg.header
            self.rgb_pub.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f'RGB processing failed: {e}')
    
    def depth_callback(self, msg):
        """
        Processes an incoming depth image and republishes it.

        Input:
            msg: ROS Image message (depth format)

        Output:
            Publishes the depth image to self.depth_pub

        Description:
            Directly republishes the received depth image message without modification.
        """
        try:
            self.depth_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Depth processing failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
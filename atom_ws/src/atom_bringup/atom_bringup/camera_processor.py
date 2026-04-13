import rclpy
from rclpy.node import Node

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
            processed_msg.header = msg.header
            self.rgb_pub.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f'RGB processing failed: {e}')
    
    def depth_callback(self, msg):
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
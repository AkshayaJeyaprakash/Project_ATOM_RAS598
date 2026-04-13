import rclpy
from rclpy.node import Node

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')


def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
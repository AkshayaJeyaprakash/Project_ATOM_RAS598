import rclpy
from rclpy.node import Node

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
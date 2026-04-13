import rclpy
from rclpy.node import Node

class VLNIntegration(Node):
    def __init__(self):
        super().__init__('vln_integration')


def main(args=None):
    rclpy.init(args=args)
    node = VLNIntegration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
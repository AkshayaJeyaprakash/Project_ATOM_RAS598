import rclpy
from rclpy.node import Node

class SemanticMapBuilder(Node):
    def __init__(self):
        super().__init__('semantic_map_builder')

def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
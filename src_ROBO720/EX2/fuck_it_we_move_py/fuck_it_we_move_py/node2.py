import rclpy
from rclpy.node import Node

class NodeOne(Node):
    def __init__(self):
        super().__init__('node1')
        self.get_logger().info('Node2 is running!')

def main(args=None):
    rclpy.init(args=args)
    node = NodeOne()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class SimplePyNode(Node):
    def __init__(self):
        super().__init__('simple_py_node')
        self.get_logger().info('Python Node started!')
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Hello from Python node')

def main(args=None):
    rclpy.init(args=args)
    node = SimplePyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

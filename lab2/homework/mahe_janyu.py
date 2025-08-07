#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MaheJanyu(Node):
    def __init__(self):
        super().__init__('MAHE_Janyu')
        self.get_logger().info('MAHE-Janyu Initiated!')
        self.create_timer(1.0, self.publish_messages)
        self.toggle = True

    def publish_messages(self):
        if self.toggle:
            self.get_logger().info('Welcome to MAHE-Janyu!')
        else:
            self.get_logger().info('I am [Your Name]!')
        self.toggle = not self.toggle

def main(args=None):
    rclpy.init(args=args)
    node = MaheJanyu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

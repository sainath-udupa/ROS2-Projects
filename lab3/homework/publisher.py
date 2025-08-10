import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random

class UltrasonicPublish(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'ultrasonic_distances', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [random.uniform(0.2, 1.0) for _ in range(3)]
        self.get_logger().info(f'Distance: {msg.data}')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicPublish()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

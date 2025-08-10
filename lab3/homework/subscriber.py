import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class UltrasonicSubscribe(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'ultrasonic_distances',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        left, front, right = msg.data
        self.get_logger().info(f'Distances => Left: {left:.2f}, Front: {front:.2f}, Right: {right:.2f}')

        threshold = 0.5
        if front > threshold:
            direction = 'Forward'
        elif left > threshold:
            direction = 'Left'
        elif right > threshold:
            direction = 'Right'
        else:
            direction = 'Stop (No clear path)'

        self.get_logger().info(f'Movement: {direction}')

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSubscribe()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

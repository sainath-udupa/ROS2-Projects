import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotPublisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(String, 'robot_news', 10)
        timer_period = 1.0 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'I am Robot: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    robot_publisher = RobotPublisher()
    rclpy.spin(robot_publisher)
    robot_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

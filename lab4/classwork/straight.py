#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class MoveStraight(Node):
    def __init__(self):
        super().__init__('move_straight')

        # Subscriber: Get turtle's position and orientation
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Publisher: Send movement commands
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Tracking variables
        self.start_x = None
        self.current_x = 0.0
        self.distance_travelled = 0.0

        # Movement parameter
        self.target_distance = 3.0       # Move forward 3 units

        # State machine: start with forward motion
        self.state = "forward"

        # Timer: Run motion loop every 0.1 seconds
        self.timer = self.create_timer(0.1, self.motion_loop)
        self.get_logger().info("Starting: Move 2 units forward.")

    def pose_callback(self, msg: Pose):
        """Updates current position of the turtle."""
        self.current_x = msg.x

        # Store starting position for forward motion
        if self.state == "forward" and self.start_x is None:
            self.start_x = msg.x

        # Calculate distance travelled
        if self.start_x is not None:
            self.distance_travelled = self.current_x - self.start_x

    def motion_loop(self):
        twist = Twist()

        if self.state == "forward":
            # Move forward until distance goal is reached
            if self.distance_travelled < self.target_distance:
                twist.linear.x = 1.0  # Move forward
            else:
                twist.linear.x = 0.0
                self.get_logger().info("Reached 3 Units. Stopping.")
                self.timer.cancel()  # Stop motion

        # Turning code removed / commented out
        # elif self.state == "turn":
        # pass

        # Publish movement command
        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MoveStraight()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

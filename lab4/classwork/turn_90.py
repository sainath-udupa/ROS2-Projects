#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class StraightAndTurn(Node):
    def __init__(self):
        super().__init__('turn')

        # Subscriber: Get turtle's position and orientation
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Publisher: Send movement commands
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Tracking variables
        self.start_x = None
        self.start_theta = None
        self.current_x = 0.0
        self.current_theta = 0.0
        self.distance_travelled = 0.0

        # Movement parameters
        self.target_distance = 3.0       # Move forward 3 units
        self.target_angle = math.pi / 2  # Turn 90 degrees in radians

        # State machine: start with forward motion
        self.state = "forward"

        # Timer: Run motion loop every 0.1 seconds
        self.timer = self.create_timer(0.1, self.motion_loop)
        self.get_logger().info("Starting: Move 3 Units Forward, then Turn 90 Degrees.")

    def pose_callback(self, msg: Pose):
        """Updates current position and orientation of the turtle."""
        self.current_x = msg.x
        self.current_theta = msg.theta

        # Store starting position for forward motion
        if self.state == "forward" and self.start_x is None:
            self.start_x = msg.x

        # Store starting angle for turning
        if self.state == "turn" and self.start_theta is None:
            self.start_theta = msg.theta

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
                self.state = "turn"
                self.start_theta = None  # Reset for turning
                self.get_logger().info("Reached 3 Units. Turning now.")

        elif self.state == "turn":
            # Record starting orientation when starting turn
            if self.start_theta is None:
                self.start_theta = self.current_theta

            # Calculate how much we have turned (handle wrap-around)
            angle_diff = (self.current_theta - self.start_theta + 2 * math.pi) % (2 * math.pi)

            if angle_diff < self.target_angle:
                twist.angular.z = 1.0  # Turn in place
            else:
                twist.angular.z = 0.0
                self.get_logger().info("Turn Complete. Stopping.")
                self.timer.cancel()  # Stop motion

        # Publish movement command
        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = StraightAndTurn()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

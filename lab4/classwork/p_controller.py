#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        # Subscribe to turtle's pose
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Publisher for movement commands
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Goal coordinates
        self.goal_x = 4.0
        self.goal_y = 4.0

        # Current pose variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Controller gains
        self.linear_k = 1.0
        self.angular_k = 4.0

        # Timer to run control loop every 0.1 seconds
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Go-to-goal controller started. Target: (4, 4)")

    def pose_callback(self, msg: Pose):
        """Updates current pose of the turtle."""
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def control_loop(self):
        """Calculates velocities to move toward the goal."""
        twist = Twist()

        # Calculate distance and angle to goal
        distance = math.sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)
        angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)

        # Calculate angular error
        angle_error = angle_to_goal - self.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normalize to [-pi, pi]

        # If far from goal, keep moving
        if distance > 0.1:
            twist.linear.x = self.linear_k * distance
            twist.angular.z = self.angular_k * angle_error
        else:
            # Goal reached → stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Goal Reached!")
            self.timer.cancel()

        # Publish command
        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

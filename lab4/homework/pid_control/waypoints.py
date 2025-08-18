#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, SetPen, TeleportAbsolute
from turtlesim.msg import Pose

Letter_dict = {
     'A':[(0,0.5,2),(1,0,0),(0,0.5,2),(1,1,0),(0,0.25,1),(1,0.75,1)],
     'B':[(0,0,2),(1,0,0),(0,0,2),(1,0.5,2),(1,0.5,1),(1,0,1),(1,1,1),(1,1,0),(1,0,0)],
     'C':[(0,1,2),(1,0,2),(1,0,0),(1,1,0)],
     'D':[(0,0,2),(1,0,0),(0,0,2),(1,0.5,2),(1,1,1.4),(1,1,0.8),(1,0.5,0),(1,0,0)],
     'E':[(0,0,2),(1,1,2),(0,0,2),(1,0,0),(1,1,0),(0,0,1),(1,0.75,1)],
     'F':[(0,0,2),(1,1,2),(0,0,2),(1,0,0),(0,0,1),(1,0.75,1)],
     'G':[(0,1,2),(1,0,2),(1,0,0),(1,1,0),(1,1,1),(1,0.5,1)],
     'H':[(0,0,2),(1,0,0),(0,0,1),(1,1,1),(1,1,2),(1,1,0)],
     'I':[(0,0.25,2),(1,0.75,2),(0,0.5,2),(1,0.5,0),(1,0.25,0),(1,0.75,0)],
     'J':[(0,0,2),(1,1,2),(0,0.65,2),(1,0.65,0),(1,0,0),(1,0,1)],
     'K':[(0,0,2),(1,0,0),(0,1,2),(1,0,1),(1,1,0)],
     'L':[(0,0,2),(1,0,0),(1,1,0)],
     'M':[(0,0.4,2),(1,0,0),(0,0.4,2),(1,0.8,0),(1,1.2,2),(1,1.6,0)],
     'N':[(0,0,2),(1,0,0),(0,0,2),(1,1,0),(1,1,2)],
     'O':[(0,0,2),(1,0,0),(1,1,0),(1,1,2),(1,0,2)],
     'P':[(0,0,2),(1,0,0),(0,0,2),(1,1,2),(1,1,1),(1,0,1)],
     'Q':[(0,1,2),(1,0,2),(1,0,0.4),(1,1,0.4),(1,1,2),(0,0.5,0.4),(1,1,0)],
     'R':[(0,0,2),(1,0,0),(0,0,2),(1,1,2),(1,1,1),(1,0,1),(1,1,0)],
     'S':[(0,1,2),(1,0,2),(1,0,1),(1,1,1),(1,1,0),(1,0,0)],
     'T':[(0,0,2),(1,1,2),(0,0.5,2),(1,0.5,0)],
     'U':[(0,0,2),(1,0,0),(1,1,0),(1,1,2)],
     'V':[(0,0,2),(1,0.5,0),(1,1,2)],
     'W':[(0,0,2),(1,0.4,0),(1,0.8,2),(1,1.2,0),(1,1.6,2)],
     'X':[(0,0,2),(1,1,0),(0,1,2),(1,0,0)],
     'Y':[(0,0,2),(1,0.5,1.2),(0,1,2),(1,0.5,1.2),(1,0.5,0)],
     'Z':[(0,0,2),(1,1,2),(1,0,0),(1,1,0)]
}

class TurtleWriter(Node):
    def __init__(self):
        super().__init__('turtle_writer')
        self.pose = None
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.spawn_cli = self.create_client(Spawn, '/spawn')
        self.pen_cli = self.create_client(SetPen, '/turtle1/set_pen')
        self.teleport_cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.spawn_cli.wait_for_service()
        self.pen_cli.wait_for_service()
        self.teleport_cli.wait_for_service()
        self.teleport(5.5, 5.5, 0.0)

    def pose_callback(self, msg: Pose):
        self.pose = msg

    def penUp(self):
        req = SetPen.Request()
        req.r = req.g = req.b = 255
        req.width = 2
        req.off = True
        self.pen_cli.call_async(req)

    def penDown(self):
        req = SetPen.Request()
        req.r = req.g = req.b = 0
        req.width = 2
        req.off = False
        self.pen_cli.call_async(req)

    def teleport(self, x, y, theta=0.0):
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        self.teleport_cli.call_async(req)

    def move_to(self, dst):
        goal_x, goal_y = dst
        while rclpy.ok() and self.pose is not None:
            dx = goal_x - self.pose.x
            dy = goal_y - self.pose.y
            distance = math.sqrt(dx**2 + dy**2)
            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - self.pose.theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            if distance < 0.05:
                break
            twist = Twist()
            twist.linear.x = min(1.5, 1.0 * distance)
            twist.angular.z = 2.0 * angle_error
            self.vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        stop = Twist()
        self.vel_pub.publish(stop)

    def writeLetter(self, letter, x, y, w, h):
        if letter not in Letter_dict:
            return
        for i,(p,xx,yy) in enumerate(Letter_dict[letter]):
            if p:
                self.penDown()
            else:
                self.penUp()
            dst = (x + xx*w, y + yy*h/2)
            if i == 0:
                self.teleport(dst[0], dst[1], 0.0)
                rclpy.spin_once(self, timeout_sec=0.2)
                continue
            self.move_to(dst)

    def write(self, string):
        currentX, currentY = 5.5, 5.5
        standardWidth, standardHeight = 2.5, 5.0
        gap = 1.0
        for ch in string:
            if ch == ' ':
                currentX += gap
                continue
            self.writeLetter(ch, currentX, currentY, standardWidth, 2*standardWidth)
            currentX += standardWidth + gap

def main(args=None):
    rclpy.init(args=args)
    node = TurtleWriter()
    text = input("Enter text (A–Z) to draw: ")
    node.write(text.upper())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

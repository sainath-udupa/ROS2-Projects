#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String  #importing the standard String message type from example_interfaces

class RobotSubscriber(Node): #defining a subscriber node class
    def __init__(self):
        #initialize the node with the name "subscriber"
        super().__init__("subscriber")

        #create a subscriber to listen to the "robot_news" topic
        #the callback function will be called whenever a message is received
        self.subscriber_ = self.create_subscription(String,"robot_news",self.callback_robot_news, 10)
        #(#message type, #topic name, #callback function to process received messages, #queue size)

        #log to know that the node has started
        self.get_logger().info("Robot Subscriber Node Started")

    #callback function triggered when a message is received
    def callback_robot_news(self, msg):
        #log to know the received message content
        #msg.data contains the string message sent by the publisher
        self.get_logger().info(f"Received: {msg.data}")

#main function to initialize and spin the node
def main(args=None):
    rclpy.init(args=args)            #initialize the ROS 2 communication
    node = RobotSubscriber()         #create an instance of the subscriber node
    rclpy.spin(node)                 #keep the node running and processing messages
    rclpy.shutdown()                 #shutdown ROS when done

#run the main function when the script is executed
if __name__ == "__main__":
    main()

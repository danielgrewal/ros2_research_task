#!/usr/bin/env python3

# This script sets up a simple ROS2 node using Python
# to subscribe to pose messages from the turtlesim package.
# This script is used to demonstrate the ROS2 subscriber functionality
# in a real time simulation

# rclpy is the standard ROS2 client library for Python
# Node is imported from rclpy.node, which is a base class for creating ROS2 nodes
# Pose from turtlesim.msg is a message type that includes position and orientation data
# (specifically for the turtlesim node, which is used in this demonstration)
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

# PoseSubscriberNode class inherits from Node, enabling it to function as a ROS2 node
class PoseSubscriberNode(Node):
    
    # Constructor initializes the node with the name "pose_subscriber" and
    # creates a subscription to the /turtle1/pose topic, which publishes Pose messages. 
    # Messages on this topic give the position (x, y coordinates) and orientation (theta) 
    # of a turtle in the turtlesim simulator.
    def __init__(self):
        super().__init__("pose_subscriber")
        
        # self.pose_callback is the function that will be called every time
        # a new message is received on this topic.
        # The number 10 specifies the queue size for incoming messages,
        # it determines how many messages can be buffered before they start being dropped.
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)
        
    # This method is triggered whenever a Pose message is received.
    # It logs the current x and y coordinates of the turtle to the ROS2 logger,
    # allowing these positions to be viewed in real time or logged for analysis.
    def pose_callback(self, msg: Pose):
        self.get_logger().info("(" + str(msg.x) + ", " + str(msg.y) + ")")
                                              
def main(args=None):
    # Initializes the ROS2 client library
    rclpy.init(args=args)
    
    # Creates an instance of PoseSubscriberNode
    node = PoseSubscriberNode()
    
    # 'spin' keeps the node alive, allowing it to listen for messages
    # on the /turtle1/pose topic and process them via the callback method.
    rclpy.spin(node)
    
    # clean up the ROS2 client library on exiting the program.
    rclpy.shutdown()
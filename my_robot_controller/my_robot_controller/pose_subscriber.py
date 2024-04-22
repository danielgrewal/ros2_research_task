#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):
    
    # create a subscriber node, Pose datatype for the turtlesim
    def __init__(self):
        super().__init__("pose_subscriber")
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)
        
    # callback will be called when a msg is received on the topic above
    def pose_callback(self, msg: Pose):
        self.get_logger().info("(" + str(msg.x) + ", " + str(msg.y) + ")")
                                                   
def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    # enables callbacks, because node is kept alive
    # callback will be called whenver message is received to turtle1/pose topic
    # Pose message type
    rclpy.spin(node)
    rclpy.shutdown()
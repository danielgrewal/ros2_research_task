#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    
    def __init__(self):
        super().__init__("draw_circle")
        # create a publisher, of type Twist (geometry msgs), using /turtle1/cmd_vel topic
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        # timer will call the send velocity command function every 0.5 seconds
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Draw circle node has been started")
        
    def send_velocity_command(self):
        # create a message (geometry type, Twist)
        msg = Twist()
        # fill the message with some values for the linear and angular fields
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        # then publish the message
        self.cmd_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()
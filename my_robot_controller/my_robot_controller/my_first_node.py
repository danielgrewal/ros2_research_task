#!/usr/bin/env python3

# python library for ROS2
import rclpy
from rclpy.node import Node

# Node class, inherited from the ROS2 Node class in rclpy library
class MyNode(Node):
    
    #  Node constructor
    def __init__(self):
        super().__init__("first_node")
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter_))
        self.counter_ += 1

def main(args=None):
    # init ROS2 communications
    rclpy.init(args=args)
    
    # create a node
    node = MyNode()
    
    # keep the node alive, node will wait for actions
    # kill manually using CTRL+C in terminal
    # spin enables callbacks
    rclpy.spin(node)
    
    # shutdown ROS2 comms
    rclpy.shutdown()   

if __name__ == '__main__':
    main()
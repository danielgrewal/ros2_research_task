#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial

class TurtleControllerNode(Node):
    
    def __init__(self):
        super().__init__("turtle_controller")
        self.previous_x_ = 0
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info("Turtle controller has been started.")

    def pose_callback(self, pose: Pose):
        # When a Pose msg is received, create a Twist (geomtry msg)
        cmd = Twist()
        
        # check if turtle is approaching a "wall" (square edges), if so then turn around
        # else, move forward
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        
        # publish the command to the turtle
        self.cmd_vel_publisher_.publish(cmd)
        
        # change the pen color if the turtle moves to the right/left side of the square
        # this will call the set pen service and change the color accordingly if condition is met
        # only when the turtle crosses the halfway point of the square the service will be called
        # this avoids polling the service constantly for checking the position
        # Services should only be called when needed to avoid unecessary overhead 
        if pose.x > 5.5 and self.previous_x_ <= 5.5:
            self.previous_x_ = pose.x
            self.get_logger().info("Set color to red")
            self.call_set_pen_service(255, 0, 0, 3, 0)
        elif pose.x <= 5.5 and self.previous_x_ > 5.5:
            self.previous_x_ = pose.x
            self.get_logger().info("Set color to green")
            self.call_set_pen_service(0, 255, 0, 3, 0)
        
    # service to change the color of the pen trace of the turtle based on its position
    def call_set_pen_service(self, r, g, b, width, off):
        # similiar to topics, services require a type (SetPen) and a name /turtle1/set_pen
        client = self.create_client(SetPen, "/turtle1/set_pen")
        # after 1 second of waiting for the service, print a message
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for service...")
        # create the request object
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        
        # make the call async so program continues after sending the request
        # this function will then return once this request is sent
        # using "call" is instead will be a synchronous call, where the client will wait indefinetely for the server
        # however, that behaviour is not intended for this demo, so we will use async
        # the response will be a "future" object
        future = client.call_async(request)
        
        # add a call back for the future response
        future.add_done_callback(partial(self.callback_set_pen))
        
    # callback for when the service replies with a response
    def callback_set_pen(self, future):
        # handle exception in case the service fails
        try:
            response = future.result()
            # if we need to do someting with the response, it would happen here
            # but for changing the pen color, we do not need to do anything with response
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))
    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
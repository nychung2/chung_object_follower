#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

import numpy as np
import sys

class RotateObject(Node):
    def __init__(self):
        super().__init__('rotate_object')

        self.cam_width = 320
        self.cam_left_boundary = self.cam_width / 3
        self.cam_right_boundary = 2 * self.cam_width / 3

        self.loc_subscriber = self.create_subscription(
            Point,
            '/obj_location',
            self.callback,
            5)
        
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 5)

        self.loc_subscriber
        self.vel_publisher

    def callback(self, msg):
        x = msg.x

        if x < self.cam_left_boundary: # turn left
            twist_msg = Twist()
            twist_msg.data = [0, 1]
            self.vel_publisher.publish(twist_msg)
        elif x > self.cam_right_boundary: # turn right
            twist_msg = Twist()
            twist_msg.data = [0, -1]
            self.vel_publisher.publish(twist_msg)

        

def main():
    rclpy.init()
    find_object = RotateObject()
    try:
        rclpy.spin(find_object)
    except SystemExit:
        rclpy.get_logger("Rotate Object Node").info("Shutting Down")
    find_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

import numpy as np
import sys

class RotateObject(node):
    def __init__(self):
        super().__init__('rotate_object')

        self.cam_width = 320
        self.cam_left_boundary = self.cam_width / 3
        self.cam_right_boundary = 2 * self.cam_width / 3

        self.loc_subscriber = self.create_subsription(
            Point,
            '/obj_location',
            self.callback,
            5)
        
        self.vel_publisher = self.create_publisher(Twist, ’/cmd_vel’, 5)

        self.loc_subscriber
        self.vel_publisher

    def callback(self, msg):
        x = msg.data[0]

        if x < self.cam_left_boundary: # turn left
            
        elif x > self.cam_right_boundary: # turn right

        

def main():
    rclpy.init()
    find_object = RotateObject()
    try:
        rclpy.spin(find_object)
    except SystemExit:
        rclpy.get_logger("Find Object Node").info("Shutting Down")
    find_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
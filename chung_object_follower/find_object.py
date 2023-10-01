import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point

import numpy as np
import cv2
from cv_bridge import CVBridge
import sys


class FindObject(Node):
    '''
    
    '''
    def __init__(self):
        super().__init__('find_object')
        
        self.kernel = np.ones((5,5), np.uint16)

        self.color = [B, G, R]
        self.hue = 
        self.sat = 
        self.val = 

        #Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(depth=5)
        image_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        image_qos_profile.durability = QoSDurabilityPolicy.VOLATILE 
        image_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 

        # make subcription to compressed image
        self.img_subscriber = self.create_subsription(
            CompressedImage,
            '/image_raw/compressed',
            self.callback,
            image_qos_profile)
        
        self.objloc_publisher = self.create_publisher(Point, '/obj_location', 5)

        self.img_subscriber
        self.objloc_publisher

    def callback(self, CompressedImage):
        imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        hsv_frame = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2HSV)
        low_bound = np.array([color[0]-self.hue, color[1]-self.sat, color[2]-self.val])
        high_bound = np.array([color[0]+self.hue, color[1]+self.sat, color[2]+self.val])
        hsv_threshold = cv2.inRange(self.hsv_frame, low_bound, high_bound)
        close_dialation = cv2.dilate(hsv_threshold, self.kernel, iterations=1)
        close_erosion = cv2.erode(close_dialation, self.kernel, iterations=1)
        open_erosion = cv2.erode(close_erosion, self.kernel, iterations=1)
        open_dialation = cv2.dilate(open_erosion, self.kernel, iterations=1)
        contours, hierarchy = cv2.findContours(open_dialation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            x, y, w, h = self.get_object_location(contours)
            cx, cy = self.get_center(x,y,w,h)
            self.publish_message(cx,cy)


    def publish_message(self, x, y):
        msg = Point()
        msg.data = [x, y, 0]
        self.objloc_publisher.publish(msg)
        self.get_logger().info("Location of Object - Publishing: %s" %msg.data)

def main():
    rclpy.init()
    find_object = FindObject()
    try:
        rclpy.spin(find_object)
    except SystemExit:
        rclpy.get_logger("Find Object Node").info("Shutting Down")
    find_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
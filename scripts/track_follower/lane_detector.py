#!/usr/bin/env python

import numpy as np
import rospy
import sys
from os import path

sys.path.append('/home/racecar/racecar_ws/src/final_challenge_2022/scripts/computer_vision')

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from final_challenge.msg import LaneLocationPixels
from color_segmentation import lane_color_segmentation


class LaneDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # Subscribe to ZED camera RGB frames
        self.left_lane_pub = rospy.Publisher("/relative_left_lane_px", LaneLocationPixels, queue_size=10)
        self.right_lane_pub = rospy.Publisher("/relative_right_lane_px", LaneLocationPixels, queue_size=10)
        self.debug_pub = rospy.Publisher("/lane_debug_img", Image, queue_size=10)
        self.image_sub_r = rospy.Subscriber("/zed/zed_node/right/image_rect_color", Image, self.right_image_callback)
        self.image_sub_l = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.left_image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg, side):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.

        # Will probably redefine the below later
        ret = lane_color_segmentation(image, side)
        pos = None
        if ret is not None:
            pos = LaneLocationPixels()
            pos.v = ret[0]
            #print "Num Columns: " + str(len(pos.v))
            pos.u = ret[1]
            #print "Num Rows: " + str(len(pos.u))


        if side == "LEFT" and pos is not None:
            self.left_lane_pub.publish(pos)
        elif side == "RIGHT" and pos is not None:
            self.right_lane_pub.publish(pos)

        #################################
        # debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        # self.debug_pub.publish(debug_msg)

    def right_image_callback(self, image_msg):
        side = "RIGHT"
        self.image_callback(image_msg, side)

    def left_image_callback(self, image_msg):
        side = "LEFT"
        self.image_callback(image_msg, side)



if __name__ == '__main__':
    try:
        rospy.init_node('LaneDetector', anonymous=True)
        LaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

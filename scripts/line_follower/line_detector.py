#!/usr/bin/env python

import numpy as np
import rospy
import cv2

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from final_challenge.msg import ObjectLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation


class LineDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.cone_pub = rospy.Publisher("/relative_line_px", ObjectLocationPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/line_debug_img", Image, queue_size=10)
        self.image_pub = rospy.Publisher("/binarized_image", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback_depth)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
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
        (x1,y1), (x2,y2) = cd_color_segmentation(image, None, "orange")
        
        pos = ObjectLocationPixel()
        pos.u = (x1+x2)/2.0
        pos.v = y2
        
        self.cone_pub.publish(pos)
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################

        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)

    def image_callback_depth(self, img):
        image = self.bridge.imgmsg_to_cv2(img, "bgr8")

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        light = np.array([3, 140, 140])
        dark = np.array([25,255,255])

        mask = cv2.inRange(hsv, light, dark)
        isolated_color = cv2.bitwise_and(image,image, mask=mask)

        gray = cv2.cvtColor(isolated_color, cv2.COLOR_BGR2GRAY)

        # kernel = np.ones((5,5), np.uint8)
        # gray = cv2.dilate(gray, kernel, iterations=1)
        
        # create a binary thresholded image
        ret,binarized = cv2.threshold(gray,50,255,cv2.THRESH_BINARY)

        output = self.bridge.cv2_to_imgmsg(binarized)
        self.image_pub.publish(output)

        


if __name__ == '__main__':
    try:
        rospy.init_node('line_detector', anonymous=True)
        LineDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

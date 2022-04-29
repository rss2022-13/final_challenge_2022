#!/usr/bin/env python

import numpy as np
import rospy
import sys

sys.path.append('/home/racecar/racecar_ws/src/final_challenge_2022/scripts/computer_vision')

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from final_challenge.msg import ObjectLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
from color_segmentation import cd_color_segmentation


class CarwashDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_carwash_px (ObjectLocationPixel) : the coordinates of the carwash in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs carwash
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.carwash_pub = rospy.Publisher("/relative_carwash_px", ObjectLocationPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/carwash_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
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
        rospy.loginfo("here")
        (x1,y1), (x2,y2) = cd_color_segmentation(image, None, "blue")
        
        pos = ObjectLocationPixel()
        pos.u = (x1+x2)/2.0
        pos.v = y2
        
        self.carwash_pub.publish(pos)
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################

        
        

        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('carwash_detector', anonymous=True)
        CarwashDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
import numpy as np

import cv2
from scipy.signal import convolve2d

from visualization_msgs.msg import Marker
from final_challenge.msg import ObjectLocation, ObjectLocationPixel
from geometry_msgs.msg import Pose, PoseArray, Point
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped

class DepthMapper():
    def __init__(self):
        self.depth_sub = rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, self.depth_callback)
        self.segment_sub = rospy.Subscriber("/binarized_image", Image, self.segment_callback)
        self.mouse = rospy.Subscriber("/zed/zed_node/depth/depth_registered_mouse_left", Point, self.mouse_callback)
        
        self.depth_map = None
        self.depth_set = False

        self.mask = np.ones((3,3))/9

        self.path_pub = rospy.Publisher("/city_path", PoseArray, queue_size=5)

        self.focalx = 338.658
        self.focaly = 338.658
        # TODO: Set this experimentally
        self.pxToMetre = None

        self.mouse_x = 0
        self.mouse_y = 0
        self.testing = True

    def mouse_callback(self,msg):
        self.mouse_x = msg.x
        self.mouse_y = msg.y

    def depth_callback(self,img_msg):
        '''
        Takes depth camera input and stores as a cv2 image.

        Note that there is a possible timing problem between taking the color image and overlaying
        this image since they are activated by two different topics.
        '''

        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        # average out the pixel values so that we avoid outliers
        print np_img.shape
        print self.mask.shape
        np_img = convolve2d(np_img, self.mask, mode="same")
        np_img = np_img.astype(int)
        if self.testing:
            print np_img[-self.mouse_y,self.mouse_x]
        # gray_img = np_img[:,:] #THIS IS PROBABLY INCORRECT
        self.depth_map = cv2.cvtColor(np_img, cv2.COLOR_GRAY2GRAY)
        # self.depth_map = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width)
        self.depth_set = True

    def convert_from_uvd(self, u, v, d):
        # d *= self.pxToMetre
        x_over_z = (self.cx - u) / self.focalx
        y_over_z = (self.cy - v) / self.focaly
        z = d / np.sqrt(1. + x_over_z**2 + y_over_z**2)
        x = x_over_z * z
        y = y_over_z * z
        return x, y

    def segment_callback(self, img):
        '''
        This takes the color segmented image of the road and uses it as a mask to isolate
        the road in the depth map. This gives us the depths(positions) of each point in the line.
        '''
        # np_img = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)
        # bgr_img = np_img[:,:-1] #THIS IS PROBABLY INCORRECT
        # segmented = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        segmented = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)

        path_depths = cv2.bitwise_and(self.depth_map,self.depth_map, mask=segmented)

        poses = []

        for row in range(0,len(path_depths.data),int(len(path_depths.data)/20)):
            for col in range(0,len(path_depths.data[0]),5):
                if path_depths.data[row][col] != 0:
                    pose = Pose()
                    pose.position.x,pose.position.y = self.convert_from_uvd(row,col,path_depths.data[row][col])
                    poses.append(pose)

        if poses:
            self.path_pub.publish(poses)

if __name__ == '__main__':
    try:
        rospy.init_node('depth_mapper', anonymous=True)
        DepthMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



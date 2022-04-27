#!/usr/bin/env python

import rospy
import numpy as np
import message_filters

import cv2
from cv_bridge import CvBridge
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

        # self.segment_sub = message_filters.Subscriber("/binarized_image", Image)
        # self.depth_sub = message_filters.Subscriber("/zed/zed_node/depth/depth_registered_mouse_left", Image)
        # ts = message_filters.ApproximateTimeSynchronizer([self.segment_sub, self.depth_sub],10, 0.01)#, allow_headerless=True)
        # ts.registerCallback(self.overlay_callback)
        
        self.depth_map = None
        self.depth_set = False

        self.mask = np.ones((3,3))/9

        self.path_pub = rospy.Publisher("/city_path", PoseArray, queue_size=5)
        self.debug_pub = rospy.Publisher("/depth_overlay", Image, queue_size=10)

        self.focalx = 338.658
        self.focaly = 338.658
        # TODO: Set this experimentally

        self.mouse_x = 100
        self.mouse_y = 100
        self.testing = False

        self.bridge = CvBridge()

    def mouse_callback(self,msg):
        self.mouse_x = int(msg.x)
        self.mouse_y = int(msg.y)

    def overlay_callback(self,img,depth_msg):
        depths = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        print "running"
        # Convert the depth image to a Numpy array
        # Depths are in METERS
        # depths = np.array(cv_image, dtype = np.dtype('f8'))

        # segmented = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)
        segmented = self.bridge.imgmsg_to_cv2(img, "mono8")

        path_depths = cv2.bitwise_and(depths,depths, mask=segmented)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(path_depths))

        # poses = []

        # for row in range(0,len(path_depths.data),int(len(path_depths.data)/20)):
        #     for col in range(0,len(path_depths.data[0]),5):
        #         if path_depths.data[row][col] != 0:
        #             pose = Pose()
        #             pose.position.x,pose.position.y = self.convert_from_uvd(row,col,path_depths.data[row][col])
        #             poses.append(pose)

        # if poses:
        #     self.path_pub.publish(poses)

    def depth_callback(self,img_msg):
        '''
        Takes depth camera input and stores as a cv2 image.

        Note that there is a possible timing problem between taking the color image and overlaying
        this image since they are activated by two different topics.
        '''

        # np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width,-1)
        # average out the pixel values so that we avoid outliers
        # print np_img.shape
        # print self.mask.shape
        # depths = cv2.cvtColor(np_img, cv2.COLOR_GRAY2GRAY)
        # print type(depths.data)

        self.depth_map = self.bridge.imgmsg_to_cv2(img_msg, "32FC1")
        # Convert the depth image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        # Depths are in METERS
        depths = np.array(self.depth_map, dtype = np.dtype('f8'))

        # np_img = convolve2d(depths, self.mask, mode="same")
        # np_img = np_img.astype(int)
        if self.testing:
            print self.mouse_y,self.mouse_x
            print depths[self.mouse_y,self.mouse_x]
        
        # self.depth_map = cv2.cvtColor(np_img, cv2.COLOR_GRAY2GRAY)
        self.depth_set = True

    def convert_from_uvd(self, u, v, d):
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
        # segmented = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)
        segmented = self.bridge.imgmsg_to_cv2(img)

        dim = (self.depth_map.shape[1], self.depth_map.shape[0])

        segmented = cv2.resize(segmented, dim, interpolation=cv2.INTER_AREA)

        path_depths = cv2.bitwise_and(self.depth_map,self.depth_map, mask=segmented)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(path_depths))

        # poses = []

        # for row in range(0,len(path_depths.data),int(len(path_depths.data)/20)):
        #     for col in range(0,len(path_depths.data[0]),5):
        #         if path_depths.data[row][col] != 0:
        #             pose = Pose()
        #             pose.position.x,pose.position.y = self.convert_from_uvd(row,col,path_depths.data[row][col])
        #             poses.append(pose)

        # if poses:
        #     self.path_pub.publish(poses)

if __name__ == '__main__':
    try:
        rospy.init_node('depth_mapper', anonymous=True)
        DepthMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



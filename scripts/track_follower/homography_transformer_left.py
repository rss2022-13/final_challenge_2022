#!/usr/bin/env python

from matplotlib.pyplot import draw
import rospy
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from final_challenge.msg import LaneLocation, LaneLocationPixels
from geometry_msgs.msg import Point

#The following collection of pixel locations and corresponding relative
#ground plane locations are used to compute our homography matrix

# PTS_IMAGE_PLANE units are in pixels
# see README.md for coordinate frame description

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_IMAGE_PLANE = [[325, 257], #325,257
                   [443,257], #443,257
                   [428,238], #428,238
                   [334,238]] # 334,238
######################################################

# PTS_GROUND_PLANE units are in inches
# car looks along positive x axis with positive y axis to left

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_GROUND_PLANE = [[27.5, 6.125],
                    [27.5, -4.875],
                    [36.125, -4.875],
                    [36.125, 6.125]] # dummy points
######################################################

METERS_PER_INCH = 0.0254


class LeftHomographyTransformer:
    def __init__(self):
        self.right_lane_px_sub = rospy.Subscriber("/relative_right_lane_px", LaneLocationPixels, self.lane_detection_callback)
        self.right_lane_pub = rospy.Publisher("/relative_right_lane", LaneLocation, queue_size=10)

        self.mouse_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color_mouse_left", Point, self.mouse_callback)
        self.marker_pub = rospy.Publisher("/right_lane_markers",Marker, queue_size=10)

        if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
            rospy.logerr("ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length")

        #Initialize data into a homography matrix

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

    def mouse_callback(self,msg):
        x, y = self.transformUvToXy(msg.x,msg.y)
        self.draw_marker(x,y,"/map")

    def lane_detection_callback(self, msg):
        #Extract information from message
        u = msg.u
        v = msg.v

        #Call to main function
        x, y = self.transformUvToXy(u, v)

        #Publish relative xy position of object in real world
        relative_xy_msg = LaneLocation()
        relative_xy_msg.x_pos = x
        relative_xy_msg.y_pos = y

        self.right_lane_pub.publish(relative_xy_msg)
        self.draw_marker(x,y,"/map")


    def transformUvToXy(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.

        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.

        Units are in meters.
        """
        
        is_list = True
        #rospy.loginfo("Length of u: %d", len(u))
        #rospy.loginfo("Is this a list? %d", is_list)
        #rospy.loginfo(is_list)
        if is_list:
            z = [1 for el in range(len(u))]

        homogeneous_point = np.array([u, v, z])
        if not is_list:
            homogeneous_point = homogeneous_point.reshape((3,1))

        xy = np.dot(self.h, homogeneous_point)
        if is_list:
            homogeneous_xy = np.zeros((2, len(u)))
            scaling_factor = np.divide(1.0, xy[2,:])
            homogeneous_xy[0,:] = np.multiply(xy[0,:], scaling_factor)
            homogeneous_xy[1,:] = np.multiply(xy[1,:], scaling_factor)
            x = homogeneous_xy[0,:]
            y = homogeneous_xy[1,:]
            #rospy.loginfo(x)
        else:
            scaling_factor = 1.0 / xy[2, 0]
            homogeneous_xy = xy * scaling_factor
            x = homogeneous_xy[0, 0]
            y = homogeneous_xy[1, 0]
        return x, y

    def draw_marker(self, lane_x, lane_y, message_frame):
        """
        Publish a marker to represent the lane in rviz.
        (Call this function if you want)
        """
        marker = Marker()
        marker.header.frame_id = message_frame
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = .2
        #marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        points = []
        for i in range(len(lane_x)):
            point = Point()
            point.x = lane_x[i]
            point.y = lane_y[i]
            point.z = 0

            points.append(point)

        marker.points = points
        
        self.marker_pub.publish(marker)

if __name__ == "__main__":
    rospy.init_node('left_homography_transformer')
    left_homography_transformer = LeftHomographyTransformer()
    rospy.spin()

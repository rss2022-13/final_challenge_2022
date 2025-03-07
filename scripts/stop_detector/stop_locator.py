#!/usr/bin/env python 

import rospy
import numpy as np

import cv2

from visualization_msgs.msg import Marker
from final_challenge.msg import ObjectLocation, ObjectLocationPixel, Finish
from geometry_msgs.msg import Point

import time

# The following collection of pixel locations and corresponding relative
# ground plane locations are used to compute our homography matrix

# PTS_IMAGE_PLANE units are in pixels
# see README.md for coordinate frame description

######################################################
# DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
# TODO: CHANGE THESE VALUES FOR THE STOP SIGN
PTS_IMAGE_PLANE = [[417, 152],  # 325,257
                   [351, 151],  # 443,257
                   [434, 142],  # 428,238
                   [351, 142]]  # 334,238
######################################################

# PTS_GROUND_PLANE units are in inches
# car looks along positive x axis with positive y axis to left

######################################################
# DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_GROUND_PLANE = [[37, -4.25],
                    [37, 4.25],
                    [27.25, -4.25],
                    [27.25, 4.25]]  # dummy points
######################################################

METERS_PER_INCH = 0.0254


class SignLocator:
    def __init__(self):
        self.line_px_sub = rospy.Subscriber("/relative_sign_px", ObjectLocationPixel, self.sign_detection_callback)
        self.cone_pub = rospy.Publisher("/relative_sign", ObjectLocation, queue_size=10)
        self.finish_sub = rospy.Subscriber("/finished", Finish, self.end_process_callback)

        self.mouse_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color_mouse_left", Point, self.mouse_callback)
        self.marker_pub = rospy.Publisher("/sign_marker", Marker, queue_size=1)

        if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
            rospy.logerr(
                "ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length")

        # Initialize data into a homography matrix

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

        # for changing states with a cooldown
        self.cooldown = 3
        self.can_pub = True
        self.timer = 0

    def mouse_callback(self, msg):
        x, y = self.transformUvToXy(msg.x, msg.y)
        self.draw_marker(x, y, "/map")

    def sign_detection_callback(self, msg):
        # Extract information from message
        u = msg.u
        v = msg.v

        # Call to main function
        x, y = self.transformUvToXy(u, v)

        # Publish relative xy position of object in real world
        relative_xy_msg = ObjectLocation()
        relative_xy_msg.x = x
        relative_xy_msg.y = y

        if self.can_pub:
            print "publishing stop sign location"
            self.cone_pub.publish(relative_xy_msg)
            self.draw_marker(x, y, "/map")

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
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y

    def draw_marker(self, cone_x, cone_y, message_frame):
        """
        Publish a marker to represent the cone in rviz.
        (Call this function if you want)
        """
        marker = Marker()
        marker.header.frame_id = message_frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = cone_x
        marker.pose.position.y = cone_y
        self.marker_pub.publish(marker)

    def end_process_callback(self,finished):
        if finished.process == "stop":
            self.can_pub = False
            self.timer = time.time()

            #just pause all outputs from this node for a little while
            while True:
                if time.time() - self.timer > self.cooldown:
                    break
            self.can_pub = True


if __name__ == "__main__":
    rospy.init_node('stop_locator')
    homography_transformer = SignLocator()
    rospy.spin()

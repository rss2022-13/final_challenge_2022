#!/usr/bin/env python

import rospy
import numpy as np
import math

from final_challenge.msg import ObjectLocation, FollowerError, State
from ackermann_msgs.msg import AckermannDriveStamped

class LineFollower():
    """
    A controller for following a line in the city
    Listens to the line's relative position and publishes driving commands if in
    the correct state.
    """
    def __init__(self):
        rospy.Subscriber("/relative_line", ObjectLocation, self.relative_line_callback)
        rospy.Subscriber("/state", State, self.state_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/follower_error", FollowerError, queue_size=10)
        
        # For determining if we can publish driving commands (in the correct state)
        self.can_publish = False

        self.parking_distance = .5 # meters; try playing with this number!

        self.speed = 0.5
        self.angle = 0

        self.prev_dist_err = 0
        self.prev_ang_err = 0

    def state_callback(self,state):
        if state.state == 0:
            self.can_publish = True
        else:
            self.can_publish = False

    def relative_line_callback(self, msg):
        #relative x and y wrt frame of racecar
        pass

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = FollowerError()      
        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = (self.relative_x**2 + self.relative_y**2)**(0.5) - self.parking_distance
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('line_follower', anonymous=True)
        LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
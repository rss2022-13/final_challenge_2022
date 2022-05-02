#!/usr/bin/env python

from dis import dis
import rospy
import numpy as np
import math
import time

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

        DRIVE_TOPIC = rospy.get_param("~drive_topic", "vesc/ackermann_cmd_mux/input/navigation") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/follower_error", FollowerError, queue_size=10)
        
        # For determining if we can publish driving commands (in the correct state)
        self.can_publish = True
        self.stop = False

        self.parking_distance = .5 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.speed = 0.5
        self.angle = 0
        self.steer_angle = 0
        self.dist_P = 1
        self.dist_D = 1.1
        self.ang_P = rospy.get_param("~ang_P", .45)
        self.ang_D = rospy.get_param("~ang_D", .25)
        self.prev_dist_err = 0
        self.prev_ang_err = 0



    def state_callback(self,state):
        if state.state == 0:
            self.can_publish = True
        elif state.state == 1:
            # this means we should be stopping for about a half second
            self.can_pubish = True
            if not self.stop:
                # Not necessarily sure if this will work, but I suspect it might 
                # since callbacks aren't thread safe (others will run while this is running)
                self.stop = True
                time.sleep(.5)
                self.stop = False
        else:
            self.can_publish = False

    def relative_line_callback(self, msg):
        #relative x and y wrt frame of racecar
        self.relative_x = msg.x
        self.relative_y = msg.y
        drive_cmd = AckermannDriveStamped()

        target_angle = math.atan2(self.relative_y, self.relative_x)
        current_distance = (self.relative_x**2 + self.relative_y**2)**(0.5) + .5

        dist_err = current_distance - self.parking_distance
        ang_err = target_angle

        #TODO: change so that if youre far and angle is large, just go forward
        #or increase the threshold for a large angle
        #case for when we're still searching for the line, just stay at the same angle and move back
        if abs(ang_err) >= math.pi/4 and abs(self.prev_ang_err) >= math.pi/4: 
            self.speed = -0.5
        elif abs(ang_err) >= math.pi/4:
            #case where we just now lost track of the line
            self.speed = -0.5
            self.steer_angle = -self.steer_angle
        else:
            self.speed = 0.5
            self.steer_angle = self.ang_P*ang_err + self.ang_D*(ang_err - self.prev_ang_err)


        #print('speed:', self.speed, 'steer:', self.steer_angle, 'ang err:', ang_err)
        self.prev_ang_err = ang_err
        self.prev_dist_err = dist_err

        drive_cmd.drive.speed = self.speed
        drive_cmd.drive.steering_angle = self.steer_angle
        if self.can_publish:
            # rospy.loginfo("line follower publishing")   
            if self.stop:
                drive_cmd.drive.speed = 0
            self.drive_pub.publish(drive_cmd)
            self.error_publisher()
        else:
            pass
            # print "cannot publish line follower"

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

#!/usr/bin/env python

from dis import dis
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

        DRIVE_TOPIC = rospy.get_param("~drive_topic", "vesc/ackermann_cmd_mux/input/navigation") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/follower_error", FollowerError, queue_size=10)
        
        # For determining if we can publish driving commands (in the correct state)
        self.can_publish = True

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
        # self.speed = self.dist_P*dist_err + self.dist_D*(dist_err - self.prev_dist_err)
        # if abs(self.speed) > 1:
        #     self.speed = self.speed/abs(self.speed)
        #self.steer_angle = self.ang_P*ang_err + self.ang_D*(ang_err - self.prev_ang_err)

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

        '''
        #stop if close enough
        if abs(dist_err) < 0.05 and abs(ang_err) < 0.1:
            self.speed = 0
            self.steer_angle = 0
        '''

        #print('speed:', self.speed, 'steer:', self.steer_angle, 'ang err:', ang_err)
        self.prev_ang_err = ang_err
        self.prev_dist_err = dist_err
        '''
        # CASE FOR SIMULATOR ONLY: Cone behind us:
        if self.relative_x <= 0:
            if self.relative_y <= 0:
                self.angle = -math.pi/2
            else:
                self.angle = math.pi/2
        
        # CASE: Cone in Front
        else:
            self.angle = target_angle

        # CASE: Cone too close
        if current_distance < self.parking_distance:
            #self.angle * -1
            self.speed = -self.speed

        # CASE: Cone to the side
        else:
            if target_angle > math.pi/8:
                self.angle = math.pi/2
            elif target_angle < math.pi/8:
                self.angle = -math.pi/2
            else:
                # CASE: Angle is fine, but we need to get closer
                if current_distance > self.parking_distance + 1: # We set the correctness tolerance as +1 Meter
                    self.speed = 0.5
                else:
                    self.speed = 0
        '''
        drive_cmd.drive.speed = self.speed
        drive_cmd.drive.steering_angle = self.steer_angle
        if self.can_publish:
            # rospy.loginfo("line follower publishing")   
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

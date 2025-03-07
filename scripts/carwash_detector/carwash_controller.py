#!/usr/bin/env python

import rospy
import math


from ackermann_msgs.msg import AckermannDriveStamped
from final_challenge.msg import ObjectLocation, State, Finish
from visualization_msgs.msg import Marker

class CarWashController():
    """
    A controller for detecting and navigating to the carwash.
    Listens for blue carwash indicator and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    # variables for inside carwash wall following
    VELOCITY = rospy.get_param("carwash_velocity", 0.5)
    DESIRED_DISTANCE = rospy.get_param("carwash_desired_distance", 0.5)
    integral = 0
    previous_error = 0
    kp = rospy.get_param("wall_follower/kp",1)
    kd = rospy.get_param("wall_follower/kd",0.2)
    N=1
    
    def __init__(self):
        self.carwash = rospy.Subscriber("/relative_carwash", ObjectLocation, self.relative_carwash_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic", "/vesc/ackermann_cmd_mux/input/navigation") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

        rospy.Subscriber("/state", State, self.state_callback)
        rospy.Subscriber("/relative_line", ObjectLocation, self.line_callback)

        self.can_publish = False
        self.finish_pub = rospy.Publisher("/finished", Finish, queue_size=1)
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.line_pub = rospy.Publisher("/wall", Marker, queue_size=1)
        self.state = 0
        
    def line_callback(self, msg):
        if self.state == 2 and msg.x <= 1: #start line following after exiting carwash
            self.finish_pub.pub("exited carwash")
        
    def state_callback(self,state):
        self.state = state
        if state.state == 2:
            if self.carwash is not None: #driving to carwash
                self.can_publish = True
            else: #drive straight through carwash
                drive_cmd = AckermannDriveStamped()
                drive_cmd.drive.speed = 1
                drive_cmd.drive.steering_angle = 0
                if self.can_publish:
                    self.drive_pub.publish(drive_cmd)
        else:
            self.can_publish = False

    def relative_carwash_callback(self, msg):
        rospy.loginfo("car wash detector publishing")
        #relative x and y wrt frame of racecar
        self.relative_x = msg.x
        self.relative_y = msg.y
        drive_cmd = AckermannDriveStamped()

        target_angle = math.atan2(self.relative_y, self.relative_x)
        current_distance = (self.relative_x**2 + self.relative_y**2)**(0.5) + .5

        self.dist_P = rospy.get_param("~dist_P", 0.45)
        self.dist_D = rospy.get_param("~dist_D", 0.25)
        self.ang_P = rospy.get_param("~ang_P", 1)
        self.ang_D = rospy.get_param("~ang_D", 0.2)

        dist_err = current_distance - self.parking_distance
        ang_err = target_angle
        self.speed = self.dist_P*dist_err + self.dist_D*(dist_err - self.prev_dist_err)
        if abs(self.speed) > 1:
            self.speed = self.speed/abs(self.speed)
        self.steer_angle = self.ang_P*ang_err + self.ang_D*(ang_err - self.prev_ang_err)

        self.prev_ang_err = ang_err
        self.prev_dist_err = dist_err
        
        drive_cmd.drive.speed = self.speed
        drive_cmd.drive.steering_angle = self.steer_angle
        if self.can_publish:
            self.drive_pub.publish(drive_cmd)
            self.error_publisher()


if __name__ == '__main__':
    try:
        rospy.init_node('wash_driver', anonymous=True)
        carwashcontroller = CarWashController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

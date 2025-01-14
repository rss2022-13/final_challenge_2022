#!/usr/bin/env python2

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *

class SafetyController:
    def __init__(self):
        
        self.scan = rospy.Subscriber("/scan", LaserScan, self.scan_data)
        self.pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=1)
        rospy.Subscriber("/state", State, self.state_callback)
        
        self.ranges = None
        self.angle_min = -2.35
        self.angle_max = 2.35
        self.increment = 0
        self.max_speed = 4.0

        self.desired_width = 2
        self.breaking_distance = 0.2
        self.apply_safety = False
        
    def state_callback(self, state):
        if state.state == 2:
            self.apply_safety = False
            
    def scan_data(self, data):
        self.ranges = data.ranges
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.increment = data.increment
        self.simple_publish(data)

    def simple_publish(self, data):
        count = 0
        # This is measuring how much to the side vs in front of us we look at
        # Right now, we look at the middle Fifth...this used to be middle Third
        # TODO: Test on both, make sure middle Third doesn't think a car racing next to it activates safety
        for sample_dist in data.ranges[int(2.0*len(data.ranges)/5.0):int(3.0*len(data.ranges)/5.0)]:
            if sample_dist < self.breaking_distance:
                count += 1
        if count >= 5:
            self.apply_safety = True
        else:
            self.apply_safety = False
        if self.apply_safety:
            rospy.loginfo("SAFETY CONTROLLER APPLIED")
            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time(0)
            msg.header.frame_id = "base_link"
            msg.drive.speed = 0
            msg.drive.steering_angle = 0
            msg.drive.steering_angle_velocity = 0
            self.pub.publish(msg)
    
if __name__ == "__main__":
    rospy.init_node("safety_controller")
    safety_controller = SafetyController()
    rospy.spin()

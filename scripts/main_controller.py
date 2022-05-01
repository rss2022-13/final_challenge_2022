#!/usr/bin/env python

import rospy
import numpy as np
from final_challenge.msg import ObjectLocation, ObjectLocationPixel, State, CarWash, Finish
from ackermann_msgs.msg import AckermannDriveStamped
from computer_vision.color_segmentation import cd_color_segmentation

class Controller:
    '''
    The main idea of this centralized controller is to listen to the outputs of the various subprocesses,
    and decide which process should be taking control of the car at a given point.

    State Values are as follows:
    0: Line Following/ City Navigation
    1: Stop Sign Behavior
    2: Car Wash behavior
    '''
    def __init__(self):
        DRIVE_TOPIC = rospy.get_param("~drive_topic")
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 10)
        self.state_pub = rospy.Publisher(rospy.get_param("~state_topic", "/state"), State, queue_size = 10)
        # self.image_pub = rospy.Subscriber("/line_debug_img", Image, self.line_callback)
        self.line_sub = rospy.Subscriber("relative_line", ObjectLocation, self.line_callback)
        self.sign_sub = rospy.Subscriber("/relative_sign", ObjectLocation, self.sign_callback)
        self.wash_sub = rospy.Subscriber("/relative_carwash_px", ObjectLocationPixel, self.wash_callback)
        self.finish_sub = rospy.Subscriber("/finished", Finish, self.end_process_callback)
        
        self.state = 0
        self.prev_state = 0

        # 3 Second cooldown when we change states
        self.cooldown = 3

    def line_callback(self, data):
        # if self.state == 3: #exiting carwash
        #     rows, cols, channels  = data.shape
        #     image_left = self.bridge.imgmsg_to_cv2(data, "bgr8")[:cols/2, :]
        
        #     image_right = self.bridge.imgmsg_to_cv2(data, "bgr8")[cols/2:, :]
        
        #     (x1_left,y1_left), (x2_left,y2_left) = cd_color_segmentation(image_left, None, "orange")
        #     (x1_right,y1_right), (x2_right,y2_right) = cd_color_segmentation(image_right, None, "orange")

        #     drive_cmd = AckermannDriveStamped()
            
        #     if y1_left < y1_right:    #positive slope line -> turn right -> only look at right image
        #         drive_cmd.drive.steering_angle = -1.5
        #     else:   #neg slope -> turn left -> look at left image
        #         drive_cmd.drive.steering_angle = 1.5
        #     self.drive_pub.publish(drive_cmd)
            
        
        if self.state != 0:
            out = State()
            out.state = 0
            self.prev_state = self.state
            self.state = 0
            self.state_pub.publish(out)
        
        
    def sign_callback(self,data):
        '''
        Listens to the sign locator, and if a sign is detected we transfer to the sign parking
        state (1) and publish.
        '''
        if self.state != 1:
            out = State()
            out.state = 1
            self.prev_state = self.state
            self.state = 1
            self.state_pub.publish(out)

    def wash_callback(self,data):
        '''
        Listens to the car wash detector, and if something is detected by it we change our state
        to the car wash state (2) and publish.
        '''
        if self.state != 2:
            out = State()
            out.state = 2
            self.prev_state = self.state
            self.state = 2
            self.state_pub.publish(out)
            

    def end_process_callback(self,process):
        '''
        Just takes us back to the previous state we were at to resume whatever process we were on.
        '''
        self.prev_state, self.state = self.state, self.prev_state

        

        out = State()
        out.state = self.state
        self.state_pub.publish(out)

if __name__ == '__main__':
    try:
        rospy.init_node('main_controller', anonymous=True)
        carwashcontroller = Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


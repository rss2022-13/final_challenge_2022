#!/usr/bin/env python

import queue
import rospy
import math
import time


from ackermann_msgs.msg import AckermannDriveStamped
from final_challenge.msg import ObjectLocation, FollowerError, State, Finish
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import Image

class StopController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_sign", ObjectLocation, self.relative_sign_callback)
        #rospy.Subscriber("") make sure to add a subscriber from the state machine activates controller
        self.stop_bbox_sub = rospy.Subscriber("/stop_sign_bbox", Float32MultiArray, self.stop_sign_detected)
        #self.depth_sub = rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, self.depth_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic", "vesc/ackermann_cmd_mux/input/navigation") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/stopping_error", FollowerError, queue_size=10)
        self.finish_pub = rospy.Publisher("/finished", Finish, queue_size=1)
        self.stop_pub = rospy.Publisher("/stop", Bool, queue_size=1)

        self.state_sub = rospy.Subscriber("/state", State, self.state_callback)
        self.can_publish = True

        self.parking_distance = .8 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.speed = 0.5
        self.angle = 0
        self.dist_P = 1
        self.dist_D = 1.1
        self.ang_P = rospy.get_param("~ang_P", .45)
        self.ang_D = rospy.get_param("~ang_D", .25)
        self.prev_dist_err = 0
        self.prev_ang_err = 0

        self.x_stop = None
        self.y_stop = None
        self.max_stop = 0.8 #maximum distance away from the stop sign you can stop
        self.stopped = False
        self.stop_duration = 1.5
        self.prev_stop_time = None
        self.buffer_time =  4.0 #time buffer between stops, gives time to get away from current stop
        self.timer_set = False

    def stop_sign_detected(self, msg):
        #collect (xmin, ymin, xmax, ymax) from message to get bounding box
        xmin, ymin, xmax, ymax = msg.data
        #get the center point of the line through (xmin, ymin), (xmax, ymax)
        slope = (ymax - ymin)/(xmax - xmin)
        length = ((ymax - ymin)**2 + (xmax - xmin)**2)**(1/2)
        phi = math.atan2(slope)
        self.x_stop = xmin + length*0.5*math.cos(phi)
        self.y_stop = ymin + length*0.5*math.sin(phi)
    
#    def depth_callback(self, img_msg):
#        self.depth_map = self.bridge.imgmsg_to_cv2(img_msg, "32FC1")
#        # Convert the depth image to a Numpy array since most cv2 functions
#        # require Numpy arrays.
#        # Depths are in METERS
#        depths = np.array(self.depth_map, dtype = np.dtype('f8'))
#        if not self.y_stop:
#            stop_dist = depths[self.y_stop, self.x_stop]
#            print("stop distance: ", stop_dist)
#            if (stop_dist <= self.max_stop) and not self.stopped:
#                msg_stop = Bool()
#                msg_stop.data = True
#                self.stop_pub.publish(msg_stop)
    def go(self):
        if self.prev_stop_time:
            since_stop = rospy.Time.now() - self.prev_stop_time
        else:
            since_stop = self.buffer_time

        if self.stopped and since_stop >= self.buffer_time:
            rospy.sleep(self.stop_duration) #sleep for 1 sec
            msg_stop = Bool()
            msg_stop.data = False
            self.stop_pub.publish(msg_stop)
            self.stopped = False
            self.prev_stop_time = rospy.Time.now()

    def state_callback(self,state):
        if state.state == 1:
            self.can_publish = True
        else:
            self.can_publish = False

    def relative_sign_callback(self, msg):
        #relative x and y wrt frame of racecar
        self.relative_x = msg.x
        self.relative_y = msg.y
        drive_cmd = AckermannDriveStamped()

        target_angle = math.atan2(self.relative_y, self.relative_x)
        current_distance = (self.relative_x**2 + self.relative_y**2)**(0.5) + .5

        self.dist_P = rospy.get_param("~dist_P", 1.0)
        self.dist_D = rospy.get_param("~dist_D", .2)

        dist_err = current_distance - self.parking_distance
        ang_err = target_angle
        self.speed = self.dist_P*dist_err + self.dist_D*(dist_err - self.prev_dist_err)
        if abs(self.speed) > 1:
            self.speed = self.speed/abs(self.speed)
        self.steer_angle = self.ang_P*ang_err + self.ang_D*(ang_err - self.prev_ang_err)

        #TODO: change so that if youre far and angle is large, just go forward
        #or increase the threshold for a large angle
        if abs(ang_err) >= math.pi/4: #prioritize fixing large angle error, back up and rotate
            self.speed = -0.5
            self.steer_angle = -abs(ang_err)/ang_err * abs(ang_err)

        
        #stop if close enough
        if abs(dist_err) < 0.05 and abs(ang_err) < 0.1:
            self.speed = 0
            self.steer_angle = 0
            if not self.timer_set and self.can_publish:
                self.stop_time = time.time()
                self.timer_set = True

        

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
            self.drive_pub.publish(drive_cmd)
            if self.timer_set and time.time() - self.stop_time > .5:
                out = Finish()
                out.process = "stop"
                self.timer_set = False
                self.finish_pub.publish(out)
            self.error_publisher()

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
        rospy.init_node('stop_controllerarkingController', anonymous=True)
        StopController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

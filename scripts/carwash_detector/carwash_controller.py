#!/usr/bin/env python

import rospy
import math


from ackermann_msgs.msg import AckermannDriveStamped
from final_challenge.msg import ObjectLocation, State

class CarWashController():
    """
    A controller for detecting and navigating to the carwash.
    Listens for blue carwash indicator and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    # variables for inside carwash wall following
    VELOCITY = rospy.get_param("carwash_velocity")
    DESIRED_DISTANCE = rospy.get_param("carwash_desired_distance")
    integral = 0
    previous_error = 0
    kp = rospy.get_param("wall_follower/kp")
    kd = rospy.get_param("wall_follower/kd")
    N=1
    
    def __init__(self):
        rospy.Subscriber("/relative_carwash", ObjectLocation, self.relative_carwash_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

        rospy.Subscriber("/state", State, self.state_callback)

        self.can_publish = False

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.line_pub = rospy.Publisher("/wall", Marker, queue_size=1)
        
        self.inside_wash_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.inside_wash_callback)
        
        self.can_publish_inside = False

    def state_callback(self,state):
        if state.state == 2:
            self.can_publish = True
        if state.state == 3:
            self.can_publish_inside = True
        else:
            self.can_publish = False
            self.can_publish_inside = False

    def relative_carwash_callback(self, msg):
        #relative x and y wrt frame of racecar
        self.relative_x = msg.x
        self.relative_y = msg.y
        drive_cmd = AckermannDriveStamped()

        target_angle = math.atan2(self.relative_y, self.relative_x)
        current_distance = (self.relative_x**2 + self.relative_y**2)**(0.5) + .5

        self.dist_P = rospy.get_param("~dist_P")
        self.dist_D = rospy.get_param("~dist_D")
        self.ang_P = rospy.get_param("~ang_P")
        self.ang_D = rospy.get_param("~ang_D")

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

    def inside_wash_callback(self, msg):
        x = data.ranges * np.cos(np.linspace(data.angle_min, data.angle_max, len(data.ranges)))
        y = data.ranges * np.sin(np.linspace(data.angle_min, data.angle_max, len(data.ranges)))
        length = x.size
        
        x = x[:length//2-ang_window]
        y = y[:length//2-ang_window]

        wall = np.polyfit(x,y,1)

        a = wall[0]
        b = wall[1]

        #VISUALIATION wall black
        VisualizationTools.plot_line(x, a*x+b, self.line_pub, frame="/laser", color = (0,0,1))
       
        theta = np.arctan(a)
        
        error = self.DESIRED_DISTANCE + b*np.cos(theta)
        
        loss = 1.0/self.N*(self.existing_error + np.abs(error))
        self.N+=1
        self.existing_error+=np.abs(error)
        self.plot_loss+=[loss]
        self.err_pub.publish(error)

        P = self.kp #proportion
        I = self.ki #integral
        D = self.kd #derivative

        dt = 1.0/500
        self.integral += error * dt

        derivative = (error - self.previous_error)/dt
        self.previous_error = error
        
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time(0)
        msg.header.frame_id = "base_link"
        msg.drive.speed = self.VELOCITY
        msg.drive.acceleration = 0
        msg.drive.steering_angle = P*error + D*derivative + max(min(I*self.integral, 0.34), -0.34)
        
        if self.can_publish_inside:
            self.drive_pub.publish(msg)

    if __name__ == '__main__':
        try:
            rospy.init_node('CarwashController', anonymous=True)
            CarwashController()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

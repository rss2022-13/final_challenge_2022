import cv2
import rospy

import numpy as np
from sensor_msgs.msg import Image
from detector import StopSignDetector

class SignDetector:
    def __init__(self):
        self.detector = StopSignDetector()
        DRIVE_TOPIC = rospy.get_param("~drive_topic") 
        self.publisher = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.subscriber = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)
        self.state = None   #states: stop-0, line follow-1, collision avoidance-2

        self.stop_distance = .5 
        self.relative_x = 0
        self.relative_y = 0
        self.speed = 0.5
        self.angle = 0
        self.dist_P = rospy.get_param("~dist_P")
        self.dist_D = rospy.get_param("~dist_D")
        self.ang_P = rospy.get_param("~ang_P")
        self.ang_D = rospy.get_param("~ang_D")
        self.prev_dist_err = 0
        self.prev_ang_err = 0

        self.stop_begin = None
        self.stop_length = 3

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
    
    def callback(self, img_msg):
        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:,:,:-1]
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        #TODO:
        stop_detected = self.detector.predict(rbg_img)[0]
        stop_sign_location = self.detector.predict(rbg_img)[1]

        drive_cmd = AckermannDriveStamped()
        
        if stop_detected and self.speed == 0:
            if self.stop_begin is None: 
                self.stop_begin = rospy.Time.now()
            if rospy.Time.now() - self.stop_begin >= self.stop_length: #stopped for long enough, continue following path
                self.stop_begin = None
                self.state = 1 #start line following
                
        if stop_detected and self.speed != 0:
            self.state = 0 #stopping state

            #use homography to calculate distance to stop sign and get ready to park/stop the car
            u = stop_sign_location[0]
            v = stop_sign_location[1]

            #Call to main function
            x, y = self.transformUvToXy(u, v)

            #Publish relative xy position of object in real world
            self.relative_x = x
            self.relative_y = y
        
            target_angle = math.atan2(self.relative_y, self.relative_x)
            current_distance = (self.relative_x**2 + self.relative_y**2)**(0.5)

        
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
        self.drive_pub.publish(drive_cmd)

if __name__=="__main__":
    rospy.init_node("stop_sign_detector")
    detect = SignDetector()
    rospy.spin()

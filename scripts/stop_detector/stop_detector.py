#!/usr/bin/env python

import cv2
import rospy

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from final_challenge.msg import ObjectLocationPixel
#from detector import StopSignDetector

class SignDetector:
    def __init__(self):
        #self.detector = StopSignDetector()
        self.publisher = rospy.Publisher("/relative_sign_px", ObjectLocationPixel, queue_size=10)
        #self.subscriber = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)
        self.other = rospy.Subscriber("/stop_sign_bbox", Float32MultiArray, self.cb2)
    
    def cb2(self,bBox):
        out = ObjectLocationPixel()
        out.u = (bBox.data[2] - bBox.data[0])/2
        out.v = bBox.data[3]
        self.publisher.publish(out)

    def callback(self, img_msg):
        # Process image without CV Bridge
        '''
        for using depth mapping, output bouding box coordinates, then in locator just take the depth image
        and use bounding box to get overall depth of the sign

        learn which values are associated with which distances
        '''
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:,:,:-1]
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

       # is_box, coords = self.detector.predict(rgb_img)
        return
        if not is_box:
            return

        out = ObjectLocationPixel()
        out.u = (coords[2] - coords[0])/2
        out.v = coords[1]

        cv2.rectangle(rgb_img,(coords[0],coords[1]),(coords[2],coords[3]),(255,255,255),2)
        self.image_print(rgb_img)

        self.publisher.publish(out)
    def image_print(self,img):
        """
        Helper function to print out images, for debugging. Pass them in as a list.
        Press any key to continue.
        """
        cv2.imshow("image", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__=="__main__":
    rospy.init_node("stop_detector")
    detect = SignDetector()
    rospy.spin()

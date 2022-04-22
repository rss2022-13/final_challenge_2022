import cv2
import rospy

import numpy as np
from sensor_msgs.msg import Image
from final_challenge.msg import ObjectLocationPixel
from detector import StopSignDetector

class SignDetector:
    def __init__(self):
        self.detector = StopSignDetector()
        self.publisher = rospy.Publisher("/relative_sign_px", ObjectLocationPixel, queue_size=10)
        self.subscriber = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)
    
    def callback(self, img_msg):
        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:,:,:-1]
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        is_box, coords = self.detector.predict(rgb_img)

        if not is_box:
            return

        out = ObjectLocationPixel()
        out.u = (coords[2] - coords[0])/2
        out.v = coords[1]

        self.publisher.publish(out)

if __name__=="__main__":
    rospy.init_node("stop_detector")
    detect = SignDetector()
    rospy.spin()

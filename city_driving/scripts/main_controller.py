import rospy
from city_driving.msg import SignLocation
from ackermann_msgs.msg import AckermannDriveStamped

class Controller:
    def __init__(self):
        DRIVE_TOPIC = rospy.get_param("~drive_topic")
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 10)
        self.sign_sub = rospy.Subscriber("/relative_sign_px")

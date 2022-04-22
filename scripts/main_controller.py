import rospy
from final_challenge_2022.msg import ObjectLocation, State, CarWash, Finish
from ackermann_msgs.msg import AckermannDriveStamped


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
        # DRIVE_TOPIC = rospy.get_param("~drive_topic")
        # self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 10)
        self.state_pub = rospy.Publisher(rospy.get_param("~state_topic", "/state"), State, queue_size = 10)
        self.sign_sub = rospy.Subscriber("/relative_sign", ObjectLocation, self.sign_callback)
        self.wash_sub = rospy.Subscriber(rospy.get_param("~car_wash_topic", "/car_wash"), CarWash, self.wash_callback)
        self.finish_sub = rospy.Subscriber("/finished", Finish, self.end_process_callback)
        
        self.state = 0
        self.prev_state = 0

    
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
        if self.prev_state == 2:
            self.prev_state, self.state = self.state, self.prev_state
        else:
            self.prev_state, self.state = self.state, 0

        out = State()
        out.state = self.state
        self.state_pub.publish(out)



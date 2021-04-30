#!/usr/bin/env python3

import rospy
from pid_class import pid_class
from std_msgs.msg import Float32
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import LanePose

class lab5:
    def __init__(self):
        rospy.Subscriber("fsm_node/mode", FSMState, self.state)
        rospy.Subscriber("/lane_filter_node/lane_pose", LanePose, self.callback)
        self.pub = rospy.Publisher("/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.d = pid_class(Kp = -4, Ki = 0, Kd = 0)  
        self.phi = pid_class(Kp = -4, Ki = 0, Kd = 0) 
        self.d_value = 0
        self.phi_value = 0
        car_control_msg = Twist2DStamped()
        self.lane_following_is_ON = False
        
    def state (self, mode):
        if mode.state == 'LANE_FOLLOWING':
            self.lane_following_is_ON = True
            rospy.logwarn("NAYE'S NODE")
        else: 
            self.lane_following_is_ON = False
    
    def callback(self, error):
        self.phi_value = error.phi 
        self.d_value = error.d
        if self.lane_following_is_ON == True:
            acc1 = self.p.update(self.d_value, 0.1)
            acc2 = self.p.update(self.phi_value, 0.1)
            car_control_msg.v = 0.1
            car_control_msg.omega = acc1 + acc2
            self.pub.publish(car_control_msg)
        
if __name__ == '__main__':
    rospy.init_node('lab5', anonymous=True)
    lab5()
    rospy.spin()

#!/usr/bin/env python3

import rospy
import std_msgs.msg
from std_srvs.srv import SetBool
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState

class line:
    def __init__(self):
        self.flag = SetBool() 
        rospy.Subscriber("/nayebot/fsm_node/mode", FSMState, self.callback)
        self.pub = rospy.Publisher("/nayebot/lane_controller_node/car_cmd", Twist2DStamped, queue_size=10)
        pub_msg = Twist2DStamped()
        pub_msg.header = std_msgs.msg.Header()
        pub_msg.header.stamp = rospy.Time.now()
	
    def move_straight(self):
        pub_msg.v=0.4099999964237213
        pub_msg.omega = 0
        self.pub.publish(pub_msg)
        
    def stop(self):
        pub_msg.v=0
        pub_msg.omega = 0
        self.pub.publish(pub_msg)

    def callback(self, mode):
        if mode.state == 'LANE_FOLLOWING': 
           self.flag = True
        else:
           self.flag = False  

if __name__ == '__main__':
    rospy.init_node('straight_line' , anonymous=True)
    l = line()
    time  = rospy.Time.now().secs
    if self.flag == True:
        while (rospy.Time.now().secs - time) < 4:
            l.move_straight()
        l.stop()
    else:
        rospy.spin()

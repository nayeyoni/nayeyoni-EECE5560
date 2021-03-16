#!/usr/bin/env python3

import rospy
import std_msgs.msg
from std_srvs.srv import SetBool
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState


flag = False
class line:
    
    def __init__(self,flag):
        rospy.Subscriber("/nayebot/fsm_node/mode", FSMState, self.callback)
        self.pub = rospy.Publisher("/nayebot/lane_controller_node/car_cmd", Twist2DStamped, queue_size=10)
        self.pub_msg = Twist2DStamped()
        self.start = 0
        self.pub_msg.header = std_msgs.msg.Header()
        
    def callback(self, mode):
        if mode.state == 'LANE_FOLLOWING':
           while self.start < 5:   
               self.pub_msg.header.stamp = rospy.Time.now()
               self.pub_msg.v=0.4099999964237213
               self.pub_msg.omega = 0
               self.pub.publish(self.pub_msg)
           self.start = self.start+1
           flag = True
        else:
           flag = False  

if __name__ == '__main__':
    rospy.init_node('straight_line' , anonymous=True)
    line()
    if flag == False:
        rospy.spin()
    if flag == True:
        rospy.signal_shutdown('Path is done')
    

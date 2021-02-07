#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class Listener:
    def __init__(self):
        rospy.Subscriber("output1", Float32, self.callback1)
        rospy.Subscriber("output2", Float32, self.callback2)
        
    def callback1(self, data):
        rospy.logininfo(rospy.get_caller_id() + "published", data.data)

    def callback2(self, data):
        rospy.logininfo(rospy.get_caller_id() + "published", data.data)
        
if __name__ == '__main__':
    rospy.init_node('sub_node' , anonymous=True)
    Listener()
    
    rospy.spin()

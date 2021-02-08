#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from mystery_package.msg import UnitsLabelled
class Listener:
    def __init__(self):
    	
    	rospy.Subscriber("/mystery/output1", Float32, self.callback1)
    	rospy.Subscriber("/mystery/output2", UnitsLabelled, self.callback2)
    	   
    
    def callback1(self, data):
        rospy.loginfo("/mystery/output1 published %s", data.data)
    def callback2(self, data):
        rospy.loginfo("/mystery/output2 published %s", data.units)

        
if __name__ == '__main__':
    rospy.init_node('sub_node' , anonymous=True)
    Listener()
    
    rospy.spin()

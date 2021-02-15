#!/usr/bin/env python3

import rospy
from mystery_package.msg import UnitsLabelled

class homework3:
    def __init__(self):
        rospy.Subscriber("/mystery/output2", UnitsLabelled, self.callback)
        self.pub = rospy.Publisher("homework3_output", UnitsLabelled, queue_size=10)
        self.total = 0;
        self.pub_msg = UnitsLabelled()
        self.pub_msg.units = "feet"
        
    def callback(self, msg):
        self.total = msg.value * 3.28084 
        self.pub_msg.value = self.total
        self.pub.publish(self.pub_msg)
        
if __name__ == '__main__':
    rospy.init_node('homework3')
    homework3()

    rospy.spin()


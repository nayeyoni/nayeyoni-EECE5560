#!/usr/bin/env python3

import rospy
from mystery_package.msg import Unitslabelled

class homework4:
    def __init__(self):
        rospy.Subscriber("/mystery/output2", Unitslabelled, self.callback)
        self.pub = rospy.Publisher("homework3_output", Unitslabelled, queue_size=10)
        self.total = 0;
        self.pub_msg = Unitslabelled()
        
    def callback(self, msg):
        if rospy.has_param("units"):
            self.units = rospy.get_param("units")
            if self.units == "smoots":
                self.pub_msg.units = "smoots"
                self.total = msg.value * 1.7018
                self.pub_msg.value = self.total
                self.pub.publish(self.pub_msg)
            elif self.units == "feet":
                self.pub_msg.units = "feet"
                self.total = msg.value * 3.28084 
                self.pub_msg.value = self.total
                self.pub.publish(self.pub_msg)
            else:
                self.pub_msg.units = "meters"
                self.total = msg.value  
                self.pub_msg.value = self.total
                self.pub.publish(self.pub_msg)
        else:
            self.units = "default"
if __name__ == '__main__':
    rospy.init_node('homework4')
    homework4()
    
    rospy.spin()
    
    

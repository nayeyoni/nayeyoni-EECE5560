#!/usr/bin/env python3

import rospy
from mystery_package.msg import UnitsLabelled

class homework4:
    def __init__(self):
        rospy.Subscriber("/mystery/output2", UnitsLabelled, self.callback)
        self.pub = rospy.Publisher("homework4_output", UnitsLabelled, queue_size=10)
        self.total = 0;
        self.pub_msg = UnitsLabelled()
        
        if rospy.has_param("units"):
            self.units = rospy.get_param("units")
                if self.units = "smoots":
                    def callback(self, msg):
                        self.total = msg.value * 1.7018
                        self.pub_msg.value = self.total
                        self.pub.publish(self.pub_msg)
                elif self.units = "feet":
                    def callback(self, msg):
                        self.total = msg.value * 3.28084 
                        self.pub_msg.value = self.total
                        self.pub.publish(self.pub_msg)
                else:
                    def callback(self, msg):
                        self.total = msg.value  
                        self.pub_msg.value = self.total
                        self.pub.publish(self.pub_msg)
        else:
            self.units = "default"
            
if __name__ == '__main__':
    rospy.init_node('homework4')
    homework3()

    rospy.spin()


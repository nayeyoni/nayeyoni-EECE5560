#!/usr/bin/env python3

import rospy
from pid_class import pid_class
from std_msgs.msg import Float32

class pid_controller:
    def __init__(self):
        
        self.p = pid_class(0.1, 0.45, 0.0056)  
        self.pub1 = rospy.Publisher("/control_input", Float32, queue_size=10)
        rospy.Subscriber("/error", Float32, self.callback)
        
    def callback(self, error):
        acc = self.p.update(error, 0.1)
        self.pub1.publish(acc)

if __name__ == '__main__':
   

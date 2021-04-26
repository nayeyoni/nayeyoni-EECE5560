#!/usr/bin/env python3

import rospy
from pid_class import pid_class
from std_msgs.msg import Float32

class pid_controller:
    def __init__(self):
        rospy.set_param("controller_ready", "true")
        self.p = pid_class(Kp = 0.025, Ki = 0.0001, Kd = 1.9)  
        self.pub1 = rospy.Publisher("/control_input", Float32, queue_size=10)
        rospy.Subscriber("/error", Float32, self.callback)
        
    def callback(self, error):
        print(error)
        acc = self.p.update(error.data, 0.1)
        self.pub1.publish(acc)

if __name__ == '__main__':
    rospy.init_node('pid_controller', anonymous=True)
    pid_controller()
    rospy.spin()

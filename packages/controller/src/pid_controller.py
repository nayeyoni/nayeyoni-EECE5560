#!/usr/bin/env python3

import rospy
from pid_class import pid_class
from std_msgs.msg import Float32

class pid_controller:
    def __init__(self):
        rospy.set_param("controller_ready", "true")
 
if __name__ == '__main__':
    rospy.init_node('pid_controller', anonymous=True)
    pid_controller()
    rospy.spin()

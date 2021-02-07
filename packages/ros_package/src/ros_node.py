#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

def fib():
    pub = rospy.Publisher("input", Float32, queue_size=10)
    a = 0
    b = 1
    #rospy.loginfo(a)
    pub.publish(a)
    #rospy.loginfo(b)
    pub.publish(b)
    rate = rospy.Rate(1) #1hz
    while not rospy.is_shutdown():
    	c = a + b
    	a = b
    	b = c
    	#rospy.loginfo(c)
    	pub.publish(c)
    	rate.sleep()
    	
if __name__ == '__main__':
    try:
    	rospy.init_node('ros_node' , anonymous=True)
    	fib()	
    except rospy.ROSInterruptException:
        pass

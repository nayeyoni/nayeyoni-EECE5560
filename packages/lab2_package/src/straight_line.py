#!/usr/bin/env python3

import rospy
from duckietown_msgs import Twist2DStamped

class line:
    def __init__(self,msg):
        self.pub = rospy.Publisher("/nayebot/car_cmd_switch_node", Twist2DStamped, queue_size=10)
	self.start=self.msg.header.sec
	self.end=self.start + 2.5
        while x < self.end
	    self.msg.omega = 0;
	    self.msg.v=0.4099999964237213
            self.pub.publish(self.msg.v)
	    self.pub.publish(self.msg.omega)
            x = x+1

if __name__ == '__main__':
    rospy.init_node('straight_line' , anonymous=True)
    line()

    rospy.spin()


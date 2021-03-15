#!/usr/bin/env python3

import rospy
import std_msgs.msg
from duckietown_msgs.msg import Twist2DStamped

class line:
    def __init__(self):
        self.pub = rospy.Publisher("/nayebot/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.start = 0
        self.header = std_msgs.msg.Header()
        while self.start < 6:
            self.header.stamp = rospy.Time.now()
            self.v=0.4099999964237213
            self.omega = 0
            self.pub.publish(self.header)
            self.pub.publish(self.v)
            self.pub.publish(self.omega)
            self.start = self.start+1

if __name__ == '__main__':
    rospy.init_node('straight_line' , anonymous=True)
    line()

    rospy.spin()


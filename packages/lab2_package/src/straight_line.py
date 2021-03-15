#!/usr/bin/env python3

import rospy
import std_msgs.msg
from duckietown_msgs.msg import Twist2DStamped

class line:
    def __init__(self):
        self.pub = rospy.Publisher("/nayebot/joy_mapper_node/car_cmd", Twist2DStamped, queue_size=10)
        self.pub_msg = Twist2DStamped()
        self.start = 0
        self.pub_msg.header = std_msgs.msg.Header()
        while self.start < 10:
            self.pub_msg.header.stamp = rospy.Time.now()
            self.pub_msg.v=0.4099999964237213
            self.pub_msg.omega = 0
            self.pub.publish(self.pub_msg)
            self.start = self.start+1

if __name__ == '__main__':
    rospy.init_node('straight_line' , anonymous=True)
    line()

    rospy.spin()


#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped

class line:
    def __init__(self):
        self.pub = rospy.Publisher("/nayebot/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.start = 0
        self.frame_id = ' '
        self.header.seq = 0
        self.header.stamp.secs = 0
        self.header.stamp.nsecs = 0
        while self.start < 6:
            self.omega = 0
            self.v=0.4099999964237213
            self.pub.publish(self.v)
            self.pub.publish(self.omega)
            self.header.seq=self.header.seq+1
            self.header.stamp.secs=self.header.stamp.secs+1
            self.header.stamp.secs=self.header.stamp.nsecs+1
            self.pub.publish(self.header.seq)
            self.pub.publish(self.header.stamp.secs)
            self.pub.publish(self.header.stamp.nsecs)
            self.pub.publish(self.header.frame_id)
            self.start = self.start+1

if __name__ == '__main__':
    rospy.init_node('straight_line' , anonymous=True)
    line()

    rospy.spin()


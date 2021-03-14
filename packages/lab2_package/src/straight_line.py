#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped

class line:
    def __init__(self):
        self.pub = rospy.Publisher("/nayebot/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.start = 0
        self.frame_id = ' '
        self.Header.seq = 0
        self.Header.stamp.secs = 0
        self.Header.stamp.nsecs = 0
        while self.start < 6:
            self.omega = 0
            self.v=0.4099999964237213
            self.pub.publish(self.v)
            self.pub.publish(self.omega)
            self.Header.seq=self.Header.seq+1
            self.Header.stamp.secs=self.Header.stamp.secs+1
            self.Header.stamp.secs=self.Header.stamp.nsecs+1
            self.pub.publish(self.Header.seq)
            self.pub.publish(self.Header.stamp.secs)
            self.pub.publish(self.Header.stamp.nsecs)
            self.pub.publish(self.Header.frame_id)
            self.start = self.start+1

if __name__ == '__main__':
    rospy.init_node('straight_line' , anonymous=True)
    line()

    rospy.spin()


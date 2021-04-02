#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D
from duckietown_msgs.msg import WheelsCmdStamped
from math import sin,cos

class lab3:
    def __init__(self):
        rospy.Subscriber("/nayebot/wheels_driver_node/wheels_cmd", WheelsCmdStamped, self.callback)
        self.pub = rospy.Publisher("/pose", Pose2D, queue_size=10)
        self.x = 0
        self.y = 0
        self.theta = 0

    def callback(self, distance):
        pose = Pose2D()
        dist_right = distance.vel_right * 0.1
        dist_left = distance.vel_left * 0.1
        delta_s=(dist_right + dist_left) / 2
        delta_theta = (dist_right - dist_left) / 0.1
        delta_x = delta_s * cos (self.theta + (delta_theta/2))
        delta_y = delta_s * sin (self.theta + (delta_theta/2))
        self.x = self.x + delta_x
        self.y = self.y + delta_y
        self.theta = self.theta + delta_theta
        pose.x = self.x
        pose.y = self.y
        pose.theta = self.theta * 10
        self.pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('lab3' , anonymous=True)
    lab3()

    rospy.spin()

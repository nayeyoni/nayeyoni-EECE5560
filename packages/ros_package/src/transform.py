#!/usr/bin/env python3

from math import radians,sin,cos
import numpy
import rospy
from duckietown_msgs.msg import Vector2D

class transform:
    def __init__(self):
        rospy.Subscriber("point_sensor_coordinate", Vector2D, self.callback_function)
        self.pub_robot =rospy.Publisher("robot_coordinate_frame", Vector2d)
        self.pub_world =rospy.Publisher("world_coordinate_frame", Vector2d)
        self.angle1=
        
    def callback_funtion(self, msg):
        msg.transform1=num
        
if __name__ == '__main__':
    rospy.init_node('transform')
    transform()
    
    rospy.spin()

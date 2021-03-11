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
        angle1=radians(180)
        angle2=(135)
        self.a=cos(angle1)
        self.b=-sin(angle1)
        self.d=sin(angle1)
        self.e=sin(angle1)
        self.c=-1
        self.f=0
        self.g=0
        self.h=0
        self.i=1
        
        self.j=cos(angle1)
        self.k=-sin(angle1)
        self.m=sin(angle1)
        self.n=sin(angle1)
        self.l=-10
        self.o=5
        self.r=1
        self.p=0
        self.q=0
        
        
     
    def callback_funtion(self, msg):
        self.v = [[msg.x],[msg.y],[1]]
        self.msg.transform1=numpy.matrix([a,b,c],[d,e,f],[g,h,i])
        self.new_v1= self.msg.transform1*self.v
        self.msg.transform2=numpy.matrix([j,k,l],[m,n,o],[p,q,r])
        self.new_v2=self.msg.transform2*self.new_v1
        self.pub_robot.publish(self.new_v1)
        self.pub.world.publish(self.new_v2)
        
if __name__ == '__main__':
    rospy.init_node('transform')
    transform()
    
    rospy.spin()

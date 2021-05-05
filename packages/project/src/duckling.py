#!/usr/bin/env python3

import rospy
import std_msgs.msg
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from sensor_msgs.msg import Imu

class duckling:
    def __init__(self):
        rospy.Subscriber("/nayebot/fsm_node/mode", FSMState, self.mode)
        rospy.Subscriber("/mama/IMU_data", Imu, self.callback)
        self.pub = rospy.Publisher("/nayebot/lane_controller_node/car_cmd", Twist2DStamped, queue_size=10)
        self.pub_msg = Twist2DStamped()
        self.pub_msg.header = std_msgs.msg.Header()
        self.pub_msg.header.stamp = rospy.Time.now()

    def state (self, mode):
        if mode.state == 'LANE_FOLLOWING':
            self.lane_following_is_ON = True
        else: 
            self.lane_following_is_ON = False

    def callback(self, data):
        if self.lane_following_is_ON == True:
            if data.accelaration.x and data.accelaration.y ////////////////////////////////////:
                while (data.accelaration.x and data.accelaration.y////////////////////////////////////):
                    self.pub_msg.v=0.4099999964237213
                    self.pub_msg.omega = 0
                    self.pub.publish(self.pub_msg)
            elif data.accelaration.x and data.accelaration.y ////////////////////////////////////:
                while (data.accelaration.x and data.accelaration.y ////////////////////////////////////):
                    self.pub_msg.v=0
                    self.pub_msg.omega = 2.5
                    self.pub.publish(self.pub_msg)
            elif data.accelaration.x and data.accelaration.y ////////////////////////////////////:
                while (data.accelaration.x and data.accelaration.y ////////////////////////////////////):
                    self.pub_msg.v=0
                    self.pub_msg.omega = -2.5
                    self.pub.publish(self.pub_msg)
            elif data.accelaration.x and data.accelaration.y ////////////////////////////////////:
                while (data.accelaration.x and data.accelaration.y ////////////////////////////////////):
                    self.pub_msg.v= -0.4099999964237213
                    self.pub_msg.omega = 0
                    self.pub.publish(self.pub_msg)

if __name__ == '__main__':
    rospy.init_node('duckling' , anonymous=True)
    duckling()

    rospy.spin()

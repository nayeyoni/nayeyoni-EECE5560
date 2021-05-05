#!/usr/bin/env python3

import rospy
import std_msgs.msg
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from sensor_msgs.msg import Imu

class duckling:
    def __init__(self):
        rospy.Subscriber("/mama/IMU_data", Imu, self.callback)
        self.pub = rospy.Publisher("/nayebot/joy_mapper_node/car_cmd", Twist2DStamped, queue_size=10)
        self.pub_msg = Twist2DStamped()
        self.pub_msg.header = std_msgs.msg.Header()
        self.pub_msg.header.stamp = rospy.Time.now()


    def callback(self, data):
        if (data.linear_acceleration.x > 1 and data.linear_acceleration.x < 5) and (data.angular_velocity.x > 245):
            while ((data.linear_acceleration.x > 1 and data.linear_acceleration.x < 5) and (data.angular_velocity.x > 245)):
                self.pub_msg.v=0.4099999964237213
                self.pub_msg.omega = 0
                self.pub.publish(self.pub_msg)
        elif (data.linear_acceleration.y > 0 and data.linear_acceleration.y < 5) and (data.angular_velocity.x > 245):
            while ((data.linear_acceleration.y > 0 and data.linear_acceleration.y < 5) and (data.angular_velocity.x > 245)):
                self.pub_msg.v=0
                self.pub_msg.omega = 2.5
                self.pub.publish(self.pub_msg)
        elif (data.angular_velocity.z > 15 and data.angular_velocity.z < 25) and (data.angular_velocity.y > 0.5 and data.angular_velocity.y < 2.5):
            while ((data.angular_velocity.z > 15 and data.angular_velocity.z < 25) and (data.angular_velocity.y > 0.5 and data.angular_velocity.y < 2.5)):
                self.pub_msg.v=0
                self.pub_msg.omega = -2.5
                self.pub.publish(self.pub_msg)
        elif (data.angular_velocity.z > 0 and data.angular_velocity.z < 0.5):
            while (data.angular_velocity.z > 0 and data.angular_velocity.z < 0.5):
                self.pub_msg.v= -0.4099999964237213
                self.pub_msg.omega = 0
                self.pub.publish(self.pub_msg)

if __name__ == '__main__':
    rospy.init_node('duckling' , anonymous=True)
    duckling()

    rospy.spin()

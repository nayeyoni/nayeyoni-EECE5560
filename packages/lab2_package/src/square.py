#!/usr/bin/env python3

import rospy
import std_msgs.msg
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
class line:
    def __init__(self):
        rospy.Subscriber("/nayebot/fsm_node/mode", FSMState, self.callback)
        self.pub = rospy.Publisher("/nayebot/lane_controller_node/car_cmd", Twist2DStamped, queue_size=10)
        self.pub_msg = Twist2DStamped()
        self.pub_msg.header = std_msgs.msg.Header()
        self.pub_msg.header.stamp = rospy.Time.now()

    def callback(self, mode):
        time = rospy.Time.now().secs
        i = 0
        if mode.state == 'LANE_FOLLOWING':
            while i < 5:
                while (rospy.Time.now().secs - time) < 2.5:
                    self.pub_msg.v=0.4099999964237213
                    self.pub_msg.omega = 0
                    self.pub.publish(self.pub_msg)

                while (rospy.Time.now().secs - time) < 3:
                    self.pub_msg.v=0
                    self.pub_msg.omega = 0
                    self.pub.publish(self.pub_msg)

                while (rospy.Time.now().secs - time) < 5:
                    self.pub_msg.v=0
                    self.pub_msg.omega = 4.5
                    self.pub.publish(self.pub_msg)

                while (rospy.Time.now().secs - time) < 7:
                    self.pub_msg.v=0
                    self.pub_msg.omega = 0
                    self.pub.publish(self.pub_msg)
                i = i + 1

            rospy.signal_shutdown('Path is done')

if __name__ == '__main__':
    rospy.init_node('straight_line' , anonymous=True)
    line()

    rospy.spin()

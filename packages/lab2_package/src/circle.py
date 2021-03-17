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
        
        if mode.state == 'LANE_FOLLOWING':
            time = rospy.Time.now().secs
            while (rospy.Time.now().secs - time) < 8.8:
                self.pub_msg.v=0.4099999964237213
                self.pub_msg.omega = 2.5
                self.pub.publish(self.pub_msg)
            self.pub_msg.v=0
            self.pub_msg.omega = 0
            self.pub.publish(self.pub_msg)
            rospy.signal_shutdown('Path is done')

if __name__ == '__main__':
    rospy.init_node('straight_line' , anonymous=True)
    line()

    rospy.spin()

#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class homework8:
    def __init__(self):
        rospy.Subscriber("image_cropped", Image, self.callback1)
        rospy.Subscriber("image_white", Image, self.callback2)
        rospy.Subscriber("image_yellow", Image, self.callback3)
        self.pub = rospy.Publisher("image_output", Image, queue_size=10)
        self.bridge = CvBridge()

        
    def callback1(self, msg1):
        self.msg1 = self.bridge.imgmsg_to_cv2(msg1, "bgr8")
        self.canny_edge_img = cv2.Canny(msg1,100, 255)
        self.pub.publish(self.canny_edge_img)
        
    def callback2(self, msg2):
        self.msg2 = self.bridge.imgmsg_to_cv2(msg2, "bgr8")
        
    def callback3(self, msg3):
        self.msg3 = self.bridge.imgmsg_to_cv2(msg3, "bgr8")

if __name__=="__main__":
    
    rospy.init_node("homework8", anonymous=True)
    img = homework8()
    rospy.spin()

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
        self.msg1_hsv = cv2.cvtColor(self.msg1,cv2.COLOR_BGR2HSV)
        self.canny_edge_img = cv2.Canny(self.msg1_hsv,100, 255)
        self.output_canny = self.bridge.cv2_to_imgmsg(self.canny_edge_img, "mono8")
        
        
    def callback2(self, msg2):
        self.msg2 = self.bridge.imgmsg_to_cv2(msg2, "bgr8")
        self.msg2_hsv = cv2.cvtColor(self.msg2,cv2.COLOR_BGR2HSV)
        
        
    def callback3(self, msg3):
        self.msg3 = self.bridge.imgmsg_to_cv2(msg3, "bgr8")
        self.msg3_hsv = cv2.cvtColor(self.msg3,cv2.COLOR_BGR2HSV)
        self.mask = cv2.bitwise_or(self.msg2_hsv, self.msg3_hsv)
        self.output_mask = self.bridge.cv2_to_imgmsg(self.mask, "brg88")
        self.pub.publish(self.output_mask)

if __name__=="__main__":
    
    rospy.init_node("homework8", anonymous=True)
    img = homework8()
    rospy.spin()

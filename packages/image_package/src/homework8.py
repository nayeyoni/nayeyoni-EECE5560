#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
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
        
        
        
    def callback2(self, msg2):
        self.msg2 = self.bridge.imgmsg_to_cv2(msg2, "mono8")
        
        
    def callback3(self, msg3):
        self.msg1_hsv = cv2.cvtColor(self.msg1,cv2.COLOR_BGR2HSV)
        self.canny_edge_img = cv2.Canny(self.msg1_hsv,150, 255)
        self.msg3 = self.bridge.imgmsg_to_cv2(msg3, "mono8")
        self.mask = cv2.bitwise_or(self.msg2, self.msg3)
        self.output_edge = cv2.bitwise_and(self.mask, self.canny_edge_img)
        self.white_lines = cv2.HoughLinesP(self.output_edge, rho = 1, theta = 1*np.pi/180), threshold = 100, minLineLength = 1, maxLineGap = 10)
        self.output_white_lines = output_lines(self, self.msg2, self.white_lines)
        self.output_mask = self.bridge.cv2_to_imgmsg(self.output_white_lines, "mono8")
        self.pub.publish(self.output_mask)

    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output

if __name__=="__main__":
    
    rospy.init_node("homework8", anonymous=True)
    img = homework8()
    rospy.spin()

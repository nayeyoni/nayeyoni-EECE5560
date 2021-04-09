#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class homework8:
    def __init__(self):
        rospy.Subscriber("image_cropped", Image, self.callback1)
        self.pub = rospy.Publisher("image_output", Image, queue_size=10)
        self.bridge = CvBridge()

        
    def callback1(self, msg1):
        self.msg1 = self.bridge.imgmsg_to_cv2(msg1, "bgr8")
        self.canny_edge_img = cv2.Canny(self.msg1,100, 255)
        self.pub.publish(self.canny_edge_img)
        


if __name__=="__main__":
    
    rospy.init_node("homework8", anonymous=True)
    img = homework8()
    rospy.spin()

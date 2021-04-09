#!/usr/bin/env python3

import sys
import rospy
import cv2
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class homework8:
    def __init__(self):
        self.sub_cropped =rospy.Subscriber("image_cropped", Image, queue_size=10)
        self.sub_white = rospy.Subscriber("image_white", Image, queue_size=10)
        self.sub_yellow = rospy.Subscriber("image_yellow", Image, queue_size=10)
        self.pub = rospy.Publisher("image_output", Image, queue_size=10)
        self.bridge = CvBridge()
        ts = message_filters.TimeSynchronizer('[self.sub_cropped, self.sub_white, self.sub_yellow], 10')
        ts.registerCallback(self.callback)
        
    def callback(self, img_cropped, img_white, img_yellow):
        cropped.img = self.bridge.imgmsg_to_cv2(img_cropped, "bgr8")
        canny_edge_img = cv2.Canny(image_hsv,100, 255)
        self.pub.publish(canny_edge_img)
        

if __name__=="__main__":
    
    rospy.init_node("homework8", anonymous=True)
    img = homework8()
    rospy.spin()

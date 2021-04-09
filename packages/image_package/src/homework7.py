#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Image_processing:
    def __init__(self):
        rospy.Subscriber("image", Image, self.callback)
        self.pub_cropped = rospy.Publisher("image_cropped", Image, queue_size=10)
        self.pub_white = rospy.Publisher("image_white", Image, queue_size=10)
        self.pub_yellow = rospy.Publisher("image_yellow", Image, queue_size=10)
        self.bridge = CvBridge()
    
    def callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cropped_img = cv_img[240:480, 0:640]
        ros_output_cropped_img = self.bridge.cv2_to_imgmsg(cropped_img, "bgr8")
        self.pub_cropped.publish(ros_output_cropped_img)
        image_hsv = cv2.cvtColor(cropped_img,cv2.COLOR_BGR2HSV)
        image_filtered_white = cv2.inRange(image_hsv,(0,0,0),(180,30,255))
        image_filtered_yellow=cv2.inRange(image_hsv,(20,100,100),(35,255,255))
        ros_output_img_yellow = self.bridge.cv2_to_imgmsg(image_filtered_yellow, "mono8")
        ros_output_img_white = self.bridge.cv2_to_imgmsg(image_filtered_white, "mono8")
        self.pub_white.publish(ros_output_img_white)
        self.pub_yellow.publish(ros_output_img_yellow)
        

if __name__=="__main__":
    
    rospy.init_node("homework7", anonymous=True)
    img = Image_processing()
    rospy.spin()

#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from duckietown_msgs.msg import SegmentList, Segment
from cv_bridge import CvBridge

class lab4:
    def __init__(self):
        rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.lanefilter_cb, queue_size=1, buff_size=2**24)
        self.pub1 = rospy.Publisher("/nayebot/line_detector_node/segment_list", SegmentList, queue_size=1, buff_size=2**24)
        self.bridge = CvBridge()
    
    def lanefilter_cb(self, msg):
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        img_size = (160, 120)
        offset = 40
        new_image = cv2.resize(cv_img, img_size, interpolation=cv2.INTER_NEAREST)
        cropped_img = new_image[offset:, :]
        
        
        
        
        image_hsv = cv2.cvtColor(cropped_img,cv2.COLOR_BGR2HSV)
        image_filtered_white = cv2.inRange(image_hsv,(0,0,0),(180,40,255))
        image_filtered_yellow=cv2.inRange(image_hsv,(20,100,100),(35,255,255))
        
        
        canny_edge_img = cv2.Canny(cropped_img,150, 255)
        mask = cv2.bitwise_or(image_filtered_white, image_filtered_yellow)
        edge = cv2.bitwise_and(mask, canny_edge_img)
        edge_lines = cv2.HoughLinesP(edge, rho = 1, theta = 1*np.pi/180, threshold = 1, minLineLength = 1, maxLineGap = 10)
        arr_cutoff = np.array([0, offset, 0, offset])
        arr_ratio = np.array([1. / img_size[0], 1. / img_size[1], 1. / img_size[0], 1. / img_size[1]])
        line_normalized = (edge_lines + arr_cutoff) * arr_ratio
        self.pub1.publish(line_normalized)
        
        self.output_white_lines = output_lines(self, self.msg1, self.white_lines)
        self.output_white = self.bridge.cv2_to_imgmsg(self.output_white_lines, "bgr8")
        self.pub_white.publish(self.output_white)
        self.yellow_edge = cv2.bitwise_and (self.msg3, self.canny_edge_img)
        self.yellow_lines = cv2.HoughLinesP(self.yellow_edge, rho = 1, theta = 1*np.pi/180, threshold = 1, minLineLength = 1, maxLineGap = 10)
        self.output_yellow_lines = output_lines(self, self.msg1, self.yellow_lines)
        self.output_yellow = self.bridge.cv2_to_imgmsg(self.output_yellow_lines, "bgr8")
        self.pub_yellow.publish(self.output_yellow)

if __name__=="__main__":
    
    rospy.init_node("homework7", anonymous=True)
    img = Image_processing()
    rospy.spin()

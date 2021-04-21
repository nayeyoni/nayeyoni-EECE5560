#!/usr/bin/env python3

import sys
import rospy
import std_msgs.msg
import cv2
import numpy as np
import itertools
from sensor_msgs.msg import Image, CompressedImage
from duckietown_msgs.msg import SegmentList, Segment
from cv_bridge import CvBridge

def output_lines(self, original_image, lines):
    output = np.copy(original_image)
    if lines is not None:
        for i in range(len(lines)):
            l = lines[i][0]
            cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
            cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
            cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
    return output


class lab4:
    def __init__(self):
        rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.lanefilter_cb, queue_size=1, buff_size=2**24)
        self.pub1 = rospy.Publisher("/nayebot/line_detector_node/segment_list", SegmentList, queue_size=10)
        self.pub2 = rospy.Publisher("lab4_lines", Image, queue_size=10)
        self.bridge = CvBridge()
    
    def lanefilter_cb(self, msg):
        rospy.logwarn("NAYE'S NODE")
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
        edge_white = cv2.bitwise_and(image_filtered_white, canny_edge_img)
        edge_yellow = cv2.bitwise_and(image_filtered_yellow, canny_edge_img)
        edge_whitelines = cv2.HoughLinesP(edge_white, rho = 1, theta = 1*np.pi/180, threshold = 1, minLineLength = 1, maxLineGap = 10)
        edge_yellowlines = cv2.HoughLinesP(edge_yellow, rho = 1, theta = 1*np.pi/180, threshold = 1, minLineLength = 1, maxLineGap = 10)
        arr_cutoff = np.array([0, offset, 0, offset])
        arr_ratio = np.array([1. / img_size[0], 1. / img_size[1], 1. / img_size[0], 1. / img_size[1]])
        whiteline_normalized = (edge_whitelines + arr_cutoff) * arr_ratio
        yellowline_normalized = (edge_yellowlines + arr_cutoff) * arr_ratio
        white_list_normalized = [list(itertools.chain(*sub)) for sub in whiteline_normalized]
        yellow_list_normalized = [list(itertools.chain(*sub)) for sub in yellowline_normalized]
        self.output_lines1 = output_lines(self, cv_img, edge_whitelines)
        self.output_lines2 = output_lines(self, self.output_lines1, edge_yellowlines)
        self.output = self.bridge.cv2_to_imgmsg(self.output_lines2, "bgr8")
        self.pub2.publish(self.output)

        pub_msg = SegmentList()
        seg1 = Segment()
        seg2 = Segment()
        for x0,y0,x1,y1 in white_list_normalized:
            seg1.color = Segment.RED
            seg1.pixels_normalized[0].x = x0 
            seg1.pixels_normalized[0].y = y0
            seg1.pixels_normalized[1].x = x1 
            seg1.pixels_normalized[1].y = y1
            
            pub_msg.segments.append(seg1)

        for x0,y0,x1,y1 in yellow_list_normalized:
            seg2.color = Segment.RED
            seg2.pixels_normalized[0].x = x0 
            seg2.pixels_normalized[0].y = y0
            seg2.pixels_normalized[1].x = x1 
            seg2.pixels_normalized[1].y = y1
            
            pub_msg.segments.append(seg2)
        self.pub1.publish(pub_msg)

                 

if __name__=="__main__":
    
    rospy.init_node("lab4", anonymous=True)
    img = lab4()
    rospy.spin()

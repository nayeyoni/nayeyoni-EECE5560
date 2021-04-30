#!/usr/bin/env python3
import IMU
import datetime
import time
import math
import IMU
import datetime
import os
import sys
import rospy
import std_msgs.msg
from sensor_msgs.msg import Imu

class IMU_calibration:
    def __init__(self):
        IMU.detectIMU()
        IMU.initIMU()


        a = datetime.datetime.now()

        magXmin = 32767
        magYmin = 32767
        magZmin = 32767
        magXmax = -32767
        magYmax = -32767
        magZmax = -32767

        while True:

            MAGx = IMU.readMAGx()
            MAGy = IMU.readMAGy()
            MAGz = IMU.readMAGz()

            if MAGx > magXmax:
                magXmax = MAGx
            if MAGy > magYmax:
               magYmax = MAGy
            if MAGz > magZmax:
               magZmax = MAGz
            if MAGx < magXmin:
               magXmin = MAGx
            if MAGy < magYmin:
               magYmin = MAGy
            if MAGz < magZmin:
               magZmin = MAGz

            rospy.logwarn((" magXmin  %i  magYmin  %i  magZmin  %i  ## magXmax  %i  magYmax  %i  magZmax %i  " %(magXmin,magYmin,magZmin,magXmax,magYmax,magZmax)))


            time.sleep(1)

if __name__ == '__main__':
    rospy.init_node('IMU_calibration' , anonymous=True)
    IMU_calibration()
    rospy.spin()

#!/usr/bin/env python3

import smbus			
import time 
import math
import os
import sys
import rospy
import std_msgs.msg
from sensor_msgs.msg import Imu


class mpu_data:
    def __init__(self):
        self.pub = rospy.Publisher("/mama/IMU_data", Imu, queue_size=10)
        self.data = Imu()
        self.data.header = std_msgs.msg.Header()
        self.data.header.stamp = rospy.Time.now()
         


        PWR_MGMT_1   = 0x6B
        SMPLRT_DIV   = 0x19
        CONFIG       = 0x1A
        GYRO_CONFIG  = 0x1B
        INT_ENABLE   = 0x38
        ACCEL_XOUT_H = 0x3B
        ACCEL_YOUT_H = 0x3D
        ACCEL_ZOUT_H = 0x3F
        GYRO_XOUT_H  = 0x43
        GYRO_YOUT_H  = 0x45
        GYRO_ZOUT_H  = 0x47


        def MPU_Init():
            bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
            bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
            bus.write_byte_data(Device_Address, CONFIG, 0)
            bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
            bus.write_byte_data(Device_Address, INT_ENABLE, 1)

        def read_raw_data(addr):
            high = bus.read_byte_data(Device_Address, addr)
            low = bus.read_byte_data(Device_Address, addr+1)
            value = ((high << 8) | low)
            if(value > 32768):
                value = value - 65536
            return value


        bus = smbus.SMBus(1) 	
        Device_Address = 0x68   

        MPU_Init()

        rospy.logwarn(" Reading Data of Gyroscope and Accelerometer")

        while True:
	
	    #Read Accelerometer raw value
            self.acc_x = read_raw_data(ACCEL_XOUT_H)
            self.acc_y = read_raw_data(ACCEL_YOUT_H)
            self.acc_z = read_raw_data(ACCEL_ZOUT_H)
	
	    #Read Gyroscope raw value
            self.gyro_x = read_raw_data(GYRO_XOUT_H)
            self.gyro_y = read_raw_data(GYRO_YOUT_H)
            self.gyro_z = read_raw_data(GYRO_ZOUT_H)
	
	    #Full scale range +/- 250 degree/C as per sensitivity scale factor
            self.Ax = self.acc_x/16384.0
            self.Ay = self.acc_y/16384.0
            self.Az = self.acc_z/16384.0
            self.Gx = self.gyro_x/131.0
            self.Gy = self.gyro_y/131.0
            self.Gz = self.gyro_z/131.0
	
            self.data.orientation.x = 0
            self.data.orientation.y = 0
            self.data.orientation.z = 0
            self.data.orientation.w = 0
            self.data.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.data.angular_velocity.x = self.Gx
            self.data.angular_velocity.y = self.Gy
            self.data.angular_velocity.z = self.Gz
            self.data.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.data.linear_acceleration.x = self.Ax
            self.data.linear_acceleration.y = self.Ay
            self.data.linear_acceleration.z = self.Az
            self.data.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.pub.publish(self.data)

            time.sleep(1)

if __name__ == '__main__':
    rospy.init_node('mpu_data' , anonymous=True)
    mpu_data()
    rospy.spin()

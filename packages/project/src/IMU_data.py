#!/usr/bin/env python3
import time
import math
import IMU
import datetime
import os
import sys
import rospy
import std_msgs.msg
from sensor_msgs.msg import Imu

class IMU_data:
    def __init__(self):
        self.pub = rospy.Publisher("/mama/IMU_data", Imu, queue_size=10)
        self.data = Imu()
        self.data.header = std_msgs.msg.Header()
        self.data.header.stamp = rospy.Time.now()

        RAD_TO_DEG = 57.29578
        M_PI = 3.14159265358979323846
        G_GAIN = 0.070  
        AA =  0.40      


        ################# Compass Calibration values ############

        magXmin =  31261
        magYmin =  1382
        magZmin =  29534
        magXmax =  32621
        magYmax =  2377
        magZmax =  31588

        ############### END Calibration offsets #################


        #Kalman filter variables
        global KFangleY
        global Q_angle
        global Q_gyro
        global y_bias
        global YP_00
        global YP_01
        global YP_10
        global YP_11
        global KFangleX
        global x_bias
        global XP_00
        global XP_01
        global XP_10
        global XP_11
        global R_angle
        Q_angle = 0.02
        Q_gyro = 0.0015
        R_angle = 0.005
        y_bias = 0.0
        x_bias = 0.0
        XP_00 = 0.0
        XP_01 = 0.0
        XP_10 = 0.0
        XP_11 = 0.0
        YP_00 = 0.0
        YP_01 = 0.0
        YP_10 = 0.0
        YP_11 = 0.0
        KFangleX = 0.0
        KFangleY = 0.0




        def kalmanFilterY ( accAngle, gyroRate, DT):
            y=0.0
            S=0.0

            global KFangleY
            global Q_angle
            global Q_gyro
            global y_bias
            global YP_00
            global YP_01
            global YP_10
            global YP_11

            KFangleY = KFangleY + DT * (gyroRate - y_bias)

            YP_00 = YP_00 + ( - DT * (YP_10 + YP_01) + Q_angle * DT )
            YP_01 = YP_01 + ( - DT * YP_11 )
            YP_10 = YP_10 + ( - DT * YP_11 )
            YP_11 = YP_11 + ( + Q_gyro * DT )

            y = accAngle - KFangleY
            S = YP_00 + R_angle
            K_0 = YP_00 / S
            K_1 = YP_10 / S

            KFangleY = KFangleY + ( K_0 * y )
            y_bias = y_bias + ( K_1 * y )

            YP_00 = YP_00 - ( K_0 * YP_00 )
            YP_01 = YP_01 - ( K_0 * YP_01 )
            YP_10 = YP_10 - ( K_1 * YP_00 )
            YP_11 = YP_11 - ( K_1 * YP_01 )

            return KFangleY

        def kalmanFilterX ( accAngle, gyroRate, DT):
            x=0.0
            S=0.0

            global KFangleX
            global Q_angle
            global Q_gyro
            global x_bias
            global XP_00
            global XP_01
            global XP_10
            global XP_11


            KFangleX = KFangleX + DT * (gyroRate - x_bias)

            XP_00 = XP_00 + ( - DT * (XP_10 + XP_01) + Q_angle * DT )
            XP_01 = XP_01 + ( - DT * XP_11 )
            XP_10 = XP_10 + ( - DT * XP_11 )
            XP_11 = XP_11 + ( + Q_gyro * DT )

            x = accAngle - KFangleX
            S = XP_00 + R_angle
            K_0 = XP_00 / S
            K_1 = XP_10 / S

            KFangleX = KFangleX + ( K_0 * x )
            x_bias = x_bias + ( K_1 * x )

            XP_00 = XP_00 - ( K_0 * XP_00 )
            XP_01 = XP_01 - ( K_0 * XP_01 )
            XP_10 = XP_10 - ( K_1 * XP_00 )
            XP_11 = XP_11 - ( K_1 * XP_01 )

            return KFangleX


        IMU.detectIMU()     #Detect if BerryIMU is connected.
        if(IMU.BerryIMUversion == 99):
            rospy.logwarn(" No BerryIMU found... exiting ")
            sys.exit()
        IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass

        gyroXangle = 0.0
        gyroYangle = 0.0
        gyroZangle = 0.0
        CFangleX = 0.0
        CFangleY = 0.0
        kalmanX = 0.0
        kalmanY = 0.0

        a = datetime.datetime.now()

        while True:


            #Read the accelerometer,gyroscope and magnetometer values
            ACCx = IMU.readACCx()
            ACCy = IMU.readACCy()
            ACCz = IMU.readACCz()
            GYRx = IMU.readGYRx()
            GYRy = IMU.readGYRy()
            GYRz = IMU.readGYRz()
            MAGx = IMU.readMAGx()
            MAGy = IMU.readMAGy()
            MAGz = IMU.readMAGz()


            #Apply compass calibration
            MAGx -= (magXmin + magXmax) /2
            MAGy -= (magYmin + magYmax) /2
            MAGz -= (magZmin + magZmax) /2


            ##Calculate loop Period(LP). How long between Gyro Reads
            b = datetime.datetime.now() - a
            a = datetime.datetime.now()
            LP = b.microseconds/(1000000*1.0)
            outputString = "Loop Time %5.2f " % ( LP )



            #Convert Gyro raw to degrees per second
            rate_gyr_x =  GYRx * G_GAIN
            rate_gyr_y =  GYRy * G_GAIN
            rate_gyr_z =  GYRz * G_GAIN


            #Calculate the angles from the gyro.
            gyroXangle+=rate_gyr_x*LP
            gyroYangle+=rate_gyr_y*LP
            gyroZangle+=rate_gyr_z*LP



            #Convert Accelerometer values to degrees
            AccXangle =  (math.atan2(ACCy,ACCz)*RAD_TO_DEG)
            AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG

            #convert the values to -180 and +180
            if AccYangle > 90:
                AccYangle -= 270.0
            else:
                AccYangle += 90.0


            #Complementary filter used to combine the accelerometer and gyro values.
            CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
            CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle

            #Kalman filter used to combine the accelerometer and gyro values.
            kalmanY = kalmanFilterY(AccYangle, rate_gyr_y,LP)
            kalmanX = kalmanFilterX(AccXangle, rate_gyr_x,LP)


            #Calculate heading
            heading = 180 * math.atan2(MAGy,MAGx)/M_PI

            #Only have our heading between 0 and 360
            if heading < 0:
                heading += 360





            ####################################################################
            ###################Tilt compensated heading#########################
            ####################################################################
            #Normalize accelerometer raw values.
            accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
            accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)


            #Calculate pitch and roll
            pitch = math.asin(accXnorm)
            roll = -math.asin(accYnorm/math.cos(pitch))


            #Calculate the new tilt compensated values
            #The compass and accelerometer are orientated differently on the the BerryIMUv1, v2 and v3.
            #This needs to be taken into consideration when performing the calculations

            #X compensation
            if(IMU.BerryIMUversion == 3):            
                magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
            
            #Y compensation
            if(IMU.BerryIMUversion == 3):            
                magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)




            #Calculate tilt compensated heading
            tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI

            if tiltCompensatedHeading < 0:
                tiltCompensatedHeading += 360


            ##################### END Tilt Compensation ########################


            self.data.orientation.x = 0
            self.data.orientation.y = 0
            self.data.orientation.z = 0
            self.data.orientation.w = 0
            self.data.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.data.angular_velocity.x = gyroXangle
            self.data.angular_velocity.y = gyroYangle
            self.data.angular_velocity.z = gyroZangle
            self.data.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.data.linear_acceleration.x = AccXangle
            self.data.linear_acceleration.y = AccYangle
            self.data.linear_acceleration.z = 0
            self.data.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.pub.publish(self.data)


            time.sleep(1)

if __name__ == '__main__':
    rospy.init_node('IMU_data' , anonymous=True)
    IMU_data()
    rospy.spin()

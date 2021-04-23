#!/usr/bin/env python3

import time
from std_msgs.msg import Float32

class pid_class:
    def __init__(self, Kp, Ki, Kd, initial_time = None):
        if initial_time is None:
            initial_time = time.time()

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.derivative = 0
        self.integral = 0
 
    def update(self, error, dt, current_time =None):
        if current_time is None:
            current_time = time.time()
        dt = current_time - initial_time
        de = error - self.prev_error
        self.derivative = de / dt
        self.integral += error * dt
        control = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * self.derivative)
        self.prev_error = error
        return control

if __name__ == '__main__':
    rospy.init_node('pid_class')
    pid_class()
    rospy.spin()
    
    

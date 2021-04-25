#!/usr/bin/env python3

import time
from std_msgs.msg import Float32

class pid_class:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.derivative = 0
        self.integral = 0
 
    def update(self, error, dt):
        de = error - self.prev_error
        self.derivative = de / dt
        self.integral += error * dt
        control = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * self.derivative)
        self.prev_error = error
        return control
    

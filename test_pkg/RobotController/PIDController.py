#!/usr/bin/env python3
#4개의 점의 어깨 높이를 기준으로, 바디 밸런싱을 PID제어로 보정정

import rclpy
from rclpy.clock import Clock, ClockType
import numpy as np

import rclpy.time

class PID_controller(object):
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.desired_roll_pitch = np.array([0.0, 0.0])
        self.desired_rpy = np.array([0.0, 0.0, 0.0])

        # TODO : Tune max_I
        self.max_I = 0.2
        self.last_error = np.array([0.0,0.0])

        self.clock = Clock(clock_type=ClockType.ROS_TIME)
        

    def reset(self):
        self.last_time = self.clock.now()
        self.I_term = np.array([0.0,0.0])
        self.D_term = np.array([0.0,0.0])
        self.last_error = np.array([0.0,0.0])
    
    def run(self, roll, pitch):
        error = self.desired_roll_pitch - np.array([roll, pitch])

        t_now = self.clock.now()
        step, step_millis = (t_now - self.last_time).seconds_nanoseconds()

        step = step+step_millis

        self.I_term = self.I_term + error*step
        
        for i in range(2):
            if(self.I_term[i] < -self.max_I):
                self.I_term[i] = -self.max_I
            elif(self.I_term[i] > self.max_I):
                self.I_term[i] = self.max_I
        
        self.D_term = (error - self.last_error) / step

        self.last_time = t_now
        self.last_error = error

        P_ret = self.kp * error
        I_ret = self.ki * error
        D_ret = self.kd * error

        return P_ret+I_ret+D_ret
    
    # def run_rpy(self, roll, pitch, yaw)

    def desired_RP_angles(des_roll, des_pitch):
        self.desired_roll_pitch = np.array([des_roll, des_pitch])
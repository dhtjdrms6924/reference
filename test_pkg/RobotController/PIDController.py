#!/usr/bin/env python3
#4개의 점의 어깨 높이를 기준으로, 바디 밸런싱을 PID제어로 보정
#PID: P(proportion-비례), I(integral-적분), D(differential-미분) -> 비례와 적분, 미분을 사용해서 on/off보다 더 정확하게 원하는 값에 도달하도록 조절절

import rclpy
from rclpy.clock import Clock, ClockType
import numpy as np

import rclpy.time

class PID_controller(object):
    def __init__(self, kp, ki, kd):
        #kp: Proportion 값
        #ki: Integral 값
        #kp: Differential 값
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.desired_roll_pitch = np.array([0.0, 0.0])
        self.desired_rpy = np.array([0.0, 0.0, 0.0])
        #roll: x축 회전, pitch: y축 회전, yaw: z축 회전 -> rpy
        #desired_roll_pitch를 통해 x, y축 회전을 알고, z축 회전은 필요없으므로 0.0으로 한다.
        #desired_rpy의 초기값은 0.0, 0.0, 0.0이지만 나중에 desired_roll_pitch에 반영되는 값으로 x, y축 회전값이 들어간다.

        # TODO : Tune max_I
        self.max_I = 0.2
        #I의 max값?
        self.last_error = np.array([0.0,0.0])
        #직전 수행에 대한 오차값

        self.clock = Clock(clock_type=ClockType.ROS_TIME)
        #시간 -> 적분할때 x축 값, y축은 error의 크기기
        

    def reset(self):
        self.last_time = self.clock.now()
        self.I_term = np.array([0.0,0.0])
        self.D_term = np.array([0.0,0.0])
        self.last_error = np.array([0.0,0.0])
        #초기화 하기
    
    def run(self, roll, pitch):
        #(self, x축 회전, y축 회전)
        error = self.desired_roll_pitch - np.array([roll, pitch])
        #error -> desired_roll_pitch에서 실제 회전한 값을 뺀다.

        t_now = self.clock.now()
        #현재 시각
        step, step_millis = (t_now - self.last_time).seconds_nanoseconds()
        #step: 두 시간차이의 초 단위 차이
        #step_millis: 두 시간차이의 나노초 단위 차이

        step = step+step_millis

        self.I_term = self.I_term + error*step
        #error*step으로 적분값 구하기, I_term에 누적으로 저장
        
        for i in range(2):
            if(self.I_term[i] < -self.max_I):
                self.I_term[i] = -self.max_I
            elif(self.I_term[i] > self.max_I):
                self.I_term[i] = self.max_I
                #I_term값을 보정해야하나봄?
        
        self.D_term = (error - self.last_error) / step
        #(error - self.last_error) / step -> 기울기를 의미, step값이 매우 작아서 미분으로 봐도 무방함

        self.last_time = t_now
        self.last_error = error
        #다음 차례에 넘겨줄 값들: t_now -> last_time, error -> last_error

        P_ret = self.kp * error
        I_ret = self.ki * error
        D_ret = self.kd * error

        return P_ret+I_ret+D_ret
    
    # def run_rpy(self, roll, pitch, yaw)

    def desired_RP_angles(des_roll, des_pitch):
        self.desired_roll_pitch = np.array([des_roll, des_pitch])
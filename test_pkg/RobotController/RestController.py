#서 있을 때, 즉 state일 때 발 위치 결정; 균형만 잡으면 됨

import rclpy
import numpy as np
from InverseKinematics import robot_IK
from PIDController import PID_controller
from RobotUtilities.Transformations import rotxyz

class RestController(object):
    def __init__(self, default_stance):
        self.def_stance = default_stance

        # TODO : 게인값 조율
        self.pid_controller = PID_controller(0., 0., 0.)
        self.use_imu = False
        self.use_button = True
        self.pid_controller.reset()
    def updateStateCommand(self, msg, state, command):
        # local body position / orientation
        state.body_local_position[0] = msg.axes[7]*0.04
        state.body_local_position[1] = msg.axes[6]*0.03
        state.body_local_position[2] = msg.axes[1]*0.03

        state.body_local_orientation[0] = msg.axes[0]*0.4
        state.body_local_orientation[1] = msg.axes[4]*0.5
        state.body_local_orientation[2] = msg.axes[3]*0.4
        
        if self.use_button:
            if msg.buttons[7]:
                self.use_imu = not self.use_imu
                self.use_button = False
                print(f"RESTController - Use rp compensation : {self.use_imu}")
        if not self.use_button:
            if not (msg.buttons[7]):
                self.use_button = True
    @property
    def default_stance(self):
        return self.def_stance
    
    def step(self, state, command):
        temp = self.default_stance
        temp[2] = [command.robot_height]*4

        # rp compensation
        # 나중에 이 부분 수정하면 됨
        if self.use_imu:
            compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)
            roll_compensation = -compensation[0]
            pitch_compensation = -compensation[1]

            rot = rotxyz(roll_compensation, pitch_compensation, 0)
            temp = np.matmul(rot, temp)
        return temp
    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        return state.foot_locations
                



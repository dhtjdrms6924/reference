#서 있을 때, 즉 state일 때 발 위치 결정; 균형만 잡으면 됨

import rclpy
import numpy as np
from InverseKinematics import robot_IK
from PIDController import PID_controller
from RobotUtilities.Transformations import rotxyz

class RestController(object):
    def __init__(self, default_stance):
        #default_stance -> ([[0, 0, 0, 0],
        #                    [0, 0, 0, 0], 
        #                    [0, 0, 0, 0],
        #                    [0, 0, 0, 0]])
        self.def_stance = default_stance

        # TODO : 게인값 조율
        self.pid_controller = PID_controller(0., 0., 0.) #(kp, ki, kd), rest상태이므로 다 0.으로 둔다.
        self.use_imu = False
        #imu:관성 보정 -> rest상태이므로 필요 없음음
        self.use_button = True
        #button기능 사용?
        self.pid_controller.reset()
        #PID 초기화
    def updateStateCommand(self, msg, state, command):
        #button조작을 담당하는 함수.....필요없....?
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
        #(self, 현재 상태, 명령)
        temp = self.default_stance
        temp[2] = [command.robot_height]*4 #robot_height를 n이라고 하자
        #             1번 2번 3번 4번 (다리)
        #temp[0] =  x|[0, 0, 0, 0]
        #temp[1] =  y|[0, 0, 0, 0]
        #temp[2] =  z|[n, n, n, n]    -->열백터

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
                



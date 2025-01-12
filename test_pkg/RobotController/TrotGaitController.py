#움직일 때 발 위치 결정
#TrotGaitController = TrotSwingController + TrotStanceController  -> swing과 stance 부분을 나누어서 만든 후 TrotGaitController로 합침

import rclpy
import numpy as np
from geometry_msgs.msg import Twist, TwistStamped
from InverseKinematics import robot_IK
from GaitController import GaitController
from PIDController import PID_controller
from CommandManager.ParamsAndCmds import LegParam

class TrotGaitController(GaitController):
    def __init__(self, default_stance, stance_time, swing_time, time_step, use_imu):
        #(self, 초기 상태, stance 시간, swing 시간, 각 절차의 소요시간, 관성 측정값)
        self.use_imu = use_imu
        self.use_button = True
        self.autoRest = True
        self.trotNeeded = True
        
        leg = LegParam()

        contact_phases = np.array([[1, 1, 1, 0],  # 0: Leg swing
                                   [1, 0, 1, 1],  # 1: Moving stance forward
                                   [1, 0, 1, 1],   
                                   [1, 1, 1, 0]])
                                   #step1: 모든 다리 지면
                                   #step2: 2, 3다리 swing
                                   #step3: 모든 다리 지면
                                   #step4: 1, 4다리 swing

        z_error_constant = 0.02 * 4    # This constant determines how fast we move
                                       # toward the goal in the z direction

        z_leg_lift = leg.gait.z_leg_lift

        super().__init__(stance_time, swing_time, time_step, contact_phases, default_stance)
        #parent class(GaitController)에서 상속 받는 것들 (stance 시간, swing 시간, 움직임 절차에 따른 시간, 다리 움직임 순서, 초기 상태)

        self.max_x_vel = leg.gait.max_x_vel
        self.max_y_vel = leg.gait.max_y_vel
        self.max_yaw_rate = leg.gait.max_yaw_rate
        #모터로 움직일 수 있는 최대치를 적어두는 곳?
        
        self.swingController = TrotSwingController(self.stance_ticks, self.swing_ticks, self.time_step,
                                                   self.phase_length, z_leg_lift, self.default_stance)
        #TrotSwingController는 아래에서 정의함

        self.stanceController = TrotStanceController(self.phase_length, self.stance_ticks, self.swing_ticks,
                                                     self.time_step, z_error_constant)
        #TrotStanceController는 아래에서 정의함

        # TODO : 게인값 조율
        self.pid_controller = PID_controller(0., 0., 0.)

    def updateStateCommand(self, msg, state, command):
        command.cmd_vel[0] = msg.linear.x /1.5 * self.max_x_vel #x축 속도
        command.cme_vel[1] = msg.linear.y /1.5 * self.max_y_vel #y축 속도  //*****************command.cmd(?)_vel[1].....?
        command.cmd_yaw_rate = msg.angular.z * self.max_yaw_rate #z축 속도

        # if self.use_button:
        #     if msg.buttons[7]:
        #         self.use_imu = not self.use_imu
        #         self.use_button = False
        #         rospy.loginfo(f"Trot Gait Controller - Use roll/pitch compensation: {self.use_imu}")

        #     elif msg.buttons[6]:
        #         self.autoRest = not self.autoRest
        #         if not self.autoRest:
        #             self.trotNeeded = True
        #         self.use_button = False
        #         rospy.loginfo(f"Trot Gait Controller - Use autorest: {self.autoRest}")
            
        # if not self.use_button:
        #     if not(msg.buttons[6] or msg.buttons[7]):
        #         self.use_button = True

    def step(self, state, command):
        #(self, 현재 상태, 수행 과제)
        if self.autoRest:
            if command.cmd_vel[0] == 0 and command.cmd_vel[1] == 0 and command.cmd_yaw_rate ==0:
            #x축 회전, y축 회전, z축 회전에 대해 수행 과제가 모두 0일 때
                if state.ticks % (2*self.phase_length) ==0:
                #실행중인 phase를 다 끝냈을 때 (phase에 남은 tick이 있을 때를 제외하기 위함)
                    self.trotNeeded = False
                    #위의 조건을 만족시 trot을 하지 않음 -> rest 모드
            else:
                self.trotNeeded = True
        if self.trotNeeded:
            #rest 모드가 아닐 때
            contact_modes = self.contacts(state.ticks)  #발의 이동 순서

            new_foot_locations = np.zeros((3,4))  #새로운 발 상태(명령 수행) 저장
            for leg_index in range(4):
                contact_modes = contact_modes[leg_index]
                if contact_modes ==1:
                    #contact_phase에서 정했듯이 1 -> swing, 0 -> stance
                    new_location = self.stanceController.next_foot_location(leg_index, state, command)
                else:
                    swing_proportion = float(self.subphase_ticks(state.ticks))/float(self.swing_ticks)
                    #남은 tick들의 수를 남은 swing_tick들의 수로 나눔 -> swing의 진행 정도?

                    new_location = self.swingController.next_foot_location(swing_proportion, leg_index, state, command)
                new_foot_locations[:, leg_index] = new_location
            
            # imu compensation
            if self.use_imu:
                compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)
                roll_compensation = -compensation[0]
                pitch_compensation = -compensation[1]
                #관성으로 인한 error를 잡기위한 식?

                rot = rotxyz(roll_compensation, pitch_compensation, 0)
                new_foot_locations = np.matmul(rot, new_foot_locations)
            state.ticks +=1
            return new_foot_locations
        else:
            temp = self.default_stance
            temp[2] = [command.robot_height]*4
            return temp
    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        state.robot_height = command.robot_height

        return state.foot_locations


        
class TrotSwingController(object):
    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance):
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.phase_length = phase_length
        self.z_leg_lift = z_leg_lift
        self.default_stance = default_stance

    def raibert_touchdwon_location(self, leg_index, command):            #//****************raibert_touchdown_location().......?
        delta_pos_2d = command.cmd_vel * self.phase_length * self.time_step
        delta_pos = np.array([delta_pos_2d[0], delta_pos_2d[1], 0])

        theta = self.stance_ticks * self.time_step * command.cmd_yaw_rate
        rotation = rotz(theta)

        return np.matmul(rotation, self.default_stance[:, leg_index]) + delta_pos
    
    def swing_height(self, swing_phase):
        #swing을 수행하는 주기를 swing_phase로 받고 아래의 계산 수행
        if swing_phase < 0.5:
            swing_height_ = swing_phase / 0.5 * self.z_leg_lift
        else:
            swing_height_ = self.z_leg_lift * (1 - (swing_phase - 0.5) / 0.5)
        return swing_height_
    
    def next_foot_location(self, swing_prop, leg_index, state, command):
        #(self, swing_proportion, 원하는 다리의 leg_index, state, command)
        assert swing_prop >= 0 and swing_prop <= 1  #assert로 혹시 swing_proportion의 값이 이상하면 중단 // swing_proportion의 값은 항상 0~1사이 값
        foot_location = state.foot_locations[:, leg_index]
        #원하는 다리의 위치
        swing_height_ = self.swing_height(swing_prop)
        #swing_proportion을 통해 현재 다리 높이를 구함
        touchdown_location = self.raibert_touchdown_location(leg_index, command) #//*****************touchdown_location......!

        time_left = self.time_step* self.swing_ticks * (1.0 - swing_prop)
        #swing의 남은 시간
        
        velocity = (touchdown_location - foot_location) / float(time_left) *\
             np.array([1, 1, 0])

        delta_foot_location = velocity * self.time_step
        #이동 후 위치
        z_vector = np.array([0, 0, swing_height_ + command.robot_height])
        #높이
        return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location
    
class TrotStanceController(object):
    def __init__(self,phase_length, stance_ticks, swing_ticks, time_step, z_error_constant):
        self.phase_length = phase_length
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.z_error_constant = z_error_constant


    def position_delta(self, leg_index, state, command):
        z = state.foot_locations[2, leg_index]

        step_dist_x = command.cmd_vel[0] *\
                      (float(self.phase_length)/self.swing_ticks)

        step_dist_y = command.cmd_vel[1] *\
                      (float(self.phase_length)/self.swing_ticks)

        velocity = np.array([-(step_dist_x/4)/(float(self.time_step)*self.stance_ticks), 
                             -(step_dist_y/4)/(float(self.time_step)*self.stance_ticks), 
                             1.0 / self.z_error_constant * (state.robot_height - z)])

        delta_pos = velocity * self.time_step
        delta_ori = rotz(-command.cmd_yaw_rate * self.time_step)
        return (delta_pos, delta_ori)
    
    def next_foot_location(self, leg_index, state, command):
        foot_location = state.foot_locations[:, leg_index]
        (delta_pos, delta_ori) = self.position_delta(leg_index, state, command)
        next_foot_location = np.matmul(delta_ori, foot_location) + delta_pos
        return next_foot_location

        


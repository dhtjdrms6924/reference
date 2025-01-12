#!/usr/bin/env python3
#Author: mike4192 https://github.com/mike4192/spotMicro
#Modified by: lnotspotl
#보형 자체의 전반적인 변수들을 제어
#발을 들면 어드정도 높이로 들 것인지
#각 GaitController에 사용되는 함수들의 정의
#4팀과 얘기 많이 해야함
#stance: 발이 지면과 닿아있는 상태, swing: 발이 지면에서 떨어진 상태태
import numpy as np

class GaitController(object):
    def __init__(self, stance_time, swing_time, time_step, contact_phases, default_stance): 
        #(self, stance 시간, swing 시간, 움직임 절차에 따른 시간, 다리 움직임 순서, 초기 상태)
        self.stance_time = stance_time
        self.swing_time = swing_time
        self.time_step = time_step
        self.contact_phases = contact_phases
        self.def_stance = default_stance

    @property
    def default_stance(self):
        #초기 상태 반환(4발로 가만히 서있는 상태)
        return self.def_stance

    @property
    def stance_ticks(self):
        #걷기의 한 동작 -> stance 시간과 움직임 절차에 따른 시간 반환
        return int(self.stance_time / self.time_step)

    @property 
    def swing_ticks(self): 
        #회전의 한 동작 -> swing 시간과 움직임 절차에 따른 시간 반환
        return int(self.swing_time / self.time_step)

    @property
    def phase_ticks(self):
        #움직임 동작의 모임 -> temp에 contact_phases를 이용해서 각 다리가 각각의 step에서 (stance/swing)의 시간은 얼마인지 저장 후 반환
        temp = []
        for i in range(len(self.contact_phases[0])):
            if 0 in self.contact_phases[:,i]:
                temp.append(self.swing_ticks)
            else:
                temp.append(self.stance_ticks)
        return temp

    @property
    def phase_length(self):
        #phase_ticks를 수행하는데 걸리는 시간의 합 -> 한 주기를 의미
        return sum(self.phase_ticks)

    def phase_index(self, ticks):
        #(self, 남은 tick들의 총 시간), 현재 phase에서 몇번 째 tick에 있는지 계산
        """ Calculate, which part of the gait cycle the robot should be in """
        phase_time = ticks % self.phase_length
        #남은 tick들의 총 시간을 주기로 나눔 
        phase_sum = 0
        phase_ticks = self.phase_ticks
        #각 tick마다의 소요시간
        for i in range(len(self.contact_phases[0])):
            phase_sum += phase_ticks[i]
            if phase_time < phase_sum:
                return i
                #몇번 째 tick인지 반환
                #       tick1 tick2 tick3 tick4
                #phase1-1sec  2sec  1sec  2sec 
                #주기 6초, ticks / self.phase_length는 지금까지 수행한 phase의 수를 나타냄
                #ticks % self.phase_length는 현재 하고 있는 phase에서 남은 tick들의 시간의 합을 나타냄
                #phase_sum에 tick1, tick2....의 시간을 순서대로 더해가면서 phase_time과 비교 -> 몇번 째 tick인지 알아 낼 수 있음음
        assert False

    def subphase_ticks(self, ticks):
        #(self, 남은 tick들의 총 시간), 현재 phase에서 수행한 tick의 수수
        """ Calculate the number of ticks (timesteps)
            since the begining of the current phase """
        phase_time = ticks % self.phase_length
        #남은 tick들의 총 시간을 주기로 나눔
        phase_sum = 0
        phase_ticks = self.phase_ticks
        #각 tick마다의 소요시간
        for i in range(len(self.contact_phases[0])):
            phase_sum += phase_ticks[i]
            if phase_time < phase_sum:
                subphase_ticks = phase_time - phase_sum + phase_ticks[i]
                #현재 phase에 있는 tick의 개수에서 현재 tick을 순서를 빼면 지금까지 수행한 tick의 개수가 나옴
                return subphase_ticks

        assert False
    
    def contacts(self, ticks):
        #현재 phase의 현재 tick에서의 contact(발의 상태)를 반환
        """ Calculate which feet should be in contact """
        return self.contact_phases[:, self.phase_index(ticks)]


# for dex3-1
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_                               # idl
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_

import numpy as np
from enum import IntEnum
import time
import os
import sys
from data_utils.params import DEFAULT_HAND_POSE
parent2_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(parent2_dir)


Dex3_Num_Motors = 7
kTopicDex3LeftCommand = "rt/dex3/left/cmd"
kTopicDex3RightCommand = "rt/dex3/right/cmd"
kTopicDex3LeftState = "rt/dex3/left/state"
kTopicDex3RightState = "rt/dex3/right/state"


DEFAULT_QPOS_LEFT = DEFAULT_HAND_POSE["unitree_g1"]["left"]["open"]
DEFAULT_QPOS_RIGHT = DEFAULT_HAND_POSE["unitree_g1"]["right"]["open"]

# thumb, middle, index
QPOS_LEFT_MAX = [1.0472, 1.0472, 1.74533,  0, 0, 0, 0]
QPOS_LEFT_MIN  = [-1.0472, -0.724312, 0, -1.5708, -1.74533, -1.5708, -1.74533]
QPOS_RIGHT_MAX = [1.0472, 0.724312, 0, 1.5708, 1.74533, 1.5708, 1.74533]
QPOS_RIGHT_MIN = [-1.0472, -1.0472, -1.74533, 0, 0, 0, 0]


class Dex3_1_Controller:
    def __init__(self, net, re_init=True):
        """
        [note] A *_array type parameter requires using a multiprocessing Array, because it needs to be passed to the internal child process

        left_hand_array: [input] Left hand skeleton data (required from XR device) to hand_ctrl.control_process

        right_hand_array: [input] Right hand skeleton data (required from XR device) to hand_ctrl.control_process

        dual_hand_data_lock: Data synchronization lock for dual_hand_state_array and dual_hand_action_array

        dual_hand_state_array: [output] Return left(7), right(7) hand motor state

        dual_hand_action_array: [output] Return left(7), right(7) hand motor action

        fps: Control frequency

        Unit_Test: Whether to enable unit testing
        """
        print("Initialize Dex3_1_Controller...")

        if re_init:
            ChannelFactoryInitialize(0, net)

        # initialize handcmd publisher and handstate subscriber
        self.LeftHandCmb_publisher = ChannelPublisher(kTopicDex3LeftCommand, HandCmd_)
        self.LeftHandCmb_publisher.Init()
        self.RightHandCmb_publisher = ChannelPublisher(kTopicDex3RightCommand, HandCmd_)
        self.RightHandCmb_publisher.Init()

        self.LeftHandState_subscriber = ChannelSubscriber(kTopicDex3LeftState, HandState_)
        self.LeftHandState_subscriber.Init()
        self.RightHandState_subscriber = ChannelSubscriber(kTopicDex3RightState, HandState_)
        self.RightHandState_subscriber.Init()

        # Arrays for additional hand states
        self.Ltemp = np.zeros((Dex3_Num_Motors, 2))
        self.Rtemp = np.zeros((Dex3_Num_Motors,2))
        self.Ltau = np.zeros(Dex3_Num_Motors)
        self.Rtau = np.zeros(Dex3_Num_Motors)
        self.Lpos = np.zeros(Dex3_Num_Motors)
        self.Rpos = np.zeros(Dex3_Num_Motors)

        # Arrays for hand states
        self.left_hand_state_array  = np.zeros(Dex3_Num_Motors)
        self.right_hand_state_array = np.zeros(Dex3_Num_Motors)
        self.get_hand_state()
        print(f"[Dex3_1_Controller] left_hand_state_array: {self.left_hand_state_array} \nright_hand_state_array: {self.right_hand_state_array}")
        self.initialize()

        print("Initialize Dex3_1_Controller OK!\n")

    def get_hand_state(self):
        left_hand_msg  = self.LeftHandState_subscriber.Read()
        right_hand_msg = self.RightHandState_subscriber.Read()
        if left_hand_msg is not None and right_hand_msg is not None:
            # Update left hand state
            for idx, id in enumerate(Dex3_1_Left_JointIndex):
                self.left_hand_state_array[idx] = left_hand_msg.motor_state[id].q
                self.Ltemp[idx,0] = left_hand_msg.motor_state[id].temperature[0]
                self.Ltemp[idx,1] = left_hand_msg.motor_state[id].temperature[1]
                self.Ltau[idx] = left_hand_msg.motor_state[id].tau_est
                self.Lpos[idx] = left_hand_msg.motor_state[id].q
            # Update right hand state
            for idx, id in enumerate(Dex3_1_Right_JointIndex):
                self.right_hand_state_array[idx] = right_hand_msg.motor_state[id].q
                self.Rtemp[idx,0] = right_hand_msg.motor_state[id].temperature[0]
                self.Rtemp[idx,1] = right_hand_msg.motor_state[id].temperature[1]
                self.Rtau[idx] = right_hand_msg.motor_state[id].tau_est
                self.Rpos[idx] = right_hand_msg.motor_state[id].q
        return self.left_hand_state_array.copy(), self.right_hand_state_array.copy()

    def get_hand_all_state(self):
        return self.Lpos.copy(), self.Rpos.copy(), self.Ltemp.copy(), self.Rtemp.copy(), self.Ltau.copy(), self.Rtau.copy()
    
    class _RIS_Mode:
        def __init__(self, id=0, status=0x01, timeout=0):
            self.motor_mode = 0
            self.id = id & 0x0F  # 4 bits for id
            self.status = status & 0x07  # 3 bits for status
            self.timeout = timeout & 0x01  # 1 bit for timeout

        def _mode_to_uint8(self):
            self.motor_mode |= (self.id & 0x0F)
            self.motor_mode |= (self.status & 0x07) << 4
            self.motor_mode |= (self.timeout & 0x01) << 7
            return self.motor_mode

    def ctrl_dual_hand(self, left_q_target, right_q_target):
        """set current left, right hand motor state target q"""
        for idx, id in enumerate(Dex3_1_Left_JointIndex):
            self.left_msg.motor_cmd[id].q = left_q_target[idx]
        for idx, id in enumerate(Dex3_1_Right_JointIndex):
            self.right_msg.motor_cmd[id].q = right_q_target[idx]

        self.LeftHandCmb_publisher.Write(self.left_msg)
        self.RightHandCmb_publisher.Write(self.right_msg)
        # print("hand ctrl publish ok.")
    
    def initialize(self):
        q = 0.0
        dq = 0.0
        tau = 0.0
        kp = 0.3
        kd = 0.1

        # initialize dex3-1's left hand cmd msg
        self.left_msg  = unitree_hg_msg_dds__HandCmd_()
        for id in Dex3_1_Left_JointIndex:
            print(f"id: {id}")
            ris_mode = self._RIS_Mode(id = id, status = 0x01)
            motor_mode = ris_mode._mode_to_uint8()
            self.left_msg.motor_cmd[id].mode = motor_mode
            self.left_msg.motor_cmd[id].q    = DEFAULT_QPOS_LEFT[id]
            self.left_msg.motor_cmd[id].dq   = dq
            self.left_msg.motor_cmd[id].tau  = tau
            self.left_msg.motor_cmd[id].kp   = kp
            self.left_msg.motor_cmd[id].kd   = kd
        
        # initialize dex3-1's right hand cmd msg
        self.right_msg  = unitree_hg_msg_dds__HandCmd_()
        for id in Dex3_1_Right_JointIndex:
            print(f"id: {id}")
            ris_mode = self._RIS_Mode(id = id, status = 0x01)
            motor_mode = ris_mode._mode_to_uint8()
            self.right_msg.motor_cmd[id].mode = motor_mode
            self.right_msg.motor_cmd[id].q    = DEFAULT_QPOS_RIGHT[id]
            self.right_msg.motor_cmd[id].dq   = dq
            self.right_msg.motor_cmd[id].tau  = tau
            self.right_msg.motor_cmd[id].kp   = kp
            self.right_msg.motor_cmd[id].kd   = kd
        
        self.LeftHandCmb_publisher.Write(self.left_msg)
        self.RightHandCmb_publisher.Write(self.right_msg)
 
class Dex3_1_Left_JointIndex(IntEnum):
    kLeftHandThumb0 = 0
    kLeftHandThumb1 = 1
    kLeftHandThumb2 = 2
    kLeftHandMiddle0 = 3
    kLeftHandMiddle1 = 4
    kLeftHandIndex0 = 5
    kLeftHandIndex1 = 6

class Dex3_1_Right_JointIndex(IntEnum):
    kRightHandThumb0 = 0
    kRightHandThumb1 = 1
    kRightHandThumb2 = 2
    kRightHandIndex0 = 3
    kRightHandIndex1 = 4
    kRightHandMiddle0 = 5
    kRightHandMiddle1 = 6


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--net', type=str, default='eno1', help='Network interface used by G1RealWorldEnv.')
    args = parser.parse_args()

    hand_ctrl = Dex3_1_Controller(args.net)

    import matplotlib.pyplot as plt
    state_list = []
    for i in range(10):
        hand_ctrl.ctrl_dual_hand([0.01*i, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0])
        # get hand state
        left_hand_state, right_hand_state = hand_ctrl.get_hand_state()
        print(f"left_hand_state: {left_hand_state} \nright_hand_state: {right_hand_state}")
        state_list.append(left_hand_state)
        time.sleep(0.05)
    state_list = np.array(state_list)
    plt.plot(state_list)
    plt.show()

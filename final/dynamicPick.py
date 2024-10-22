from IntermediatePoints import *
import rospy
import time
import numpy as np
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

class lineSlot:
    def __init__(self, start_world_frame: np.ndarray, end_world_frame: np.ndarray) -> None:
        self.start_world_frame = start_world_frame
        self.end_world_frame = end_world_frame

class RedOrientations:
    default = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]
    ])
    face_turnable = np.array([
        [0, 1, 0],
        [1, 0, 0],
        [0, 0, -1]
    ])

    face_back = np.array([
        [-1, 0, 0],
        [0, 1, 0],
        [0, 0, -1]
    ])

    catch_turnable = np.array([
        [0, 0, -1],
        [0, 1, 0],
        [1, 0, 0]
    ])

class RedDynamic:
    slot0 = lineSlot(start_world_frame=np.array([0, -0.255, 0.240]), end_world_frame=np.array([0, -0.255, 0.225]))
    
    def __init__(self, arm: ArmController, slotId=0) -> None:
        self.slotId = slotId
        self.arm = arm
    
    def move_to_slot(self):
        # q_target = np.array([ 0.85653,  1.24441,  0.80996, -1.04397,  0.73898,  1.60664, -1.20087]) # z=0.240
        # q_target = np.array([ 0.85414,  1.23553,  0.81162, -1.0491 ,  0.74261,  1.60688, -1.19672]) # z=0.245
        arm = self.arm
        arm.exec_gripper_cmd(0.12)
        q_target = np.array([ 0.86785,  1.18132,  0.77884, -1.08684,  0.63779,  1.68238, -1.19171]) # tilt=10
        q_source = q_target.copy()
        q_source[0] -= pi/180*25
        arm.safe_move_to_position(q_source)
        arm.safe_move_to_position(q_target)
        time.sleep(5)
        arm.exec_gripper_cmd(0.049, force=100)

        dis = np.abs(arm.get_gripper_state()['position']).mean()
        while dis < 0.01:
            gripper_state = arm.get_gripper_state()['position']
            print('gripper_state', gripper_state)

            arm.exec_gripper_cmd(0.12)
            time.sleep(5)
            arm.exec_gripper_cmd(0.049, force=100)
            dis = np.abs(arm.get_gripper_state()['position']).mean()
        
        arm.safe_move_to_position(q_source)

class BlueDynamic:
    def __init__(self, arm: ArmController) -> None:
        self.arm = arm
    
    def move_to_slot(self):
        arm = self.arm
        arm.exec_gripper_cmd(0.12)
        q_target = np.array([-1.94755,  1.04262,  0.15504, -1.04685,  1.15531,  1.46477, -1.32736])
        q_safe = np.array([-1.94755-pi/180*25,  1.04262-pi/180*45,  0.15504, -1.04685,  1.15531,  1.46477, -1.32736])
        q_source = q_target.copy()
        q_source[0] -= pi/180*25
        arm.safe_move_to_position(q_safe)
        arm.safe_move_to_position(q_source)
        arm.safe_move_to_position(q_target)

        time.sleep(5)
        arm.exec_gripper_cmd(0.049, force=100)
        dis = np.abs(arm.get_gripper_state()['position']).mean()
        while dis < 0.01:
            gripper_state = arm.get_gripper_state()['position']
            print('gripper_state', gripper_state)

            arm.exec_gripper_cmd(0.12)
            time.sleep(5)
            arm.exec_gripper_cmd(0.049, force=100)
            dis = np.abs(arm.get_gripper_state()['position']).mean()


        arm.safe_move_to_position(q_source)
        arm.safe_move_to_position(q_safe)

if __name__ == "__main__":
    dynamic_pick = RedDynamic(arm=None)


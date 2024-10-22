import sys
import numpy as np
from copy import deepcopy
from math import pi
from lib.calculateFK import FK
from lib.solveIK import IK
# import rospy
# # Common interfaces for interacting with both the simulation and real environments!
# from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# # for timing that is consistent with simulation or real time as appropriate
# from core.utils import time_in_seconds


def block_pos(p, q):
    # p- pose of block in camera frame returned using OpenCV library
    # q- robot currect joint configuration

    H_ee_camera = detector.get_H_ee_camera() # the function is provided by TAs\
    # This is pose of camera with respect to the end effector

    T0e = FK().forward(q)
    pose = T0e @ H_ee_camera @ p

    return pose  #returns 4x4 matrix of the block with respect to robot base frame


# Function for scaning pose of each static block
def scan():    #scans cube pos return array of seen cube pos
    r = np.array([[1, 0, 0],
              [0, -1, 0],
              [0, 0, -1],
              [0, 0, 0]])
    pose_array= []
    for (name, pose) in detector.get_detections():
        position = block_pos(pose, q) 
        pose_array.append(position)
    for i in pose_array: # To get the target orientation for the end effector before grabbing the block
        i[:3, :3] = r
        i[3, 3] = 1
        i[2, 3] += 0.2
    return pos_array
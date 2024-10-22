import sys
import numpy as np
from copy import deepcopy
from math import pi
from lib.calculateFK import FK
from lib.solveIK import IK
from core.interfaces import ObjectDetector
from copy import *
from scipy.spatial.transform import Rotation


def transform_from_camera_frame_to_robot_frame(p, q, detector: ObjectDetector):
    H_ee_camera = detector.get_H_ee_camera()
    jointPositions, T0e = FK().forward(q)
    pose = T0e @ H_ee_camera @ p
    return pose#robot frame


def scan_for_blocks_pose_in_robot_frame(q: np.ndarray, detector: ObjectDetector):
    pose_array= []
    for (name, pose) in detector.get_detections():
        position = transform_from_camera_frame_to_robot_frame(pose, q, detector) 
        pose_array.append(position)
    return pose_array


def get_angle_along_z_axis_from_block_pose(block_pose: np.ndarray) -> np.ndarray:
    block_rotation = Rotation.from_matrix(block_pose[0:3, 0:3])
    yaw, pitch, roll = block_rotation.as_euler('zyx')

    angle_along_z_axis = yaw
    rotation_not_we_want = []
    threshold = 1e-3#1e-2#
    for angle in [roll, pitch, yaw]:
        for special_angle in np.deg2rad(np.array([-360,-270,-180,-90,0, 90, 180, 270, 360])):
            if np.abs(angle - special_angle) < threshold:
                rotation_not_we_want.append(angle)
    
    for angle in [roll, pitch, yaw]:
        if angle not in rotation_not_we_want:
            angle_along_z_axis = angle
            break
    print('angle_along_z_axis0', angle_along_z_axis)
    if angle_along_z_axis>=0:
        angle_along_z_axis=angle_along_z_axis%(np.pi/2)
        print('angle_along_z_axis1',angle_along_z_axis)
        #if angle_along_z_axis>(np.pi/4):
            #angle_along_z_axis=angle_along_z_axis-np.pi/2
            #print('angle_along_z_axis2', angle_along_z_axis)
    else:
        angle_along_z_axis = angle_along_z_axis % (-np.pi / 2)
        print('angle_along_z_axis3', angle_along_z_axis)
        #if angle_along_z_axis<(-np.pi/4):
            #angle_along_z_axis=np.pi/2-angle_along_z_axis
            #print('angle_along_z_axis4', angle_along_z_axis)
    return angle_along_z_axis
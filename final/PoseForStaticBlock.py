import sys
import math
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
    return pose


def scan_for_blocks_pose_in_robot_frame(q: np.ndarray, detector: ObjectDetector):
    pose_array = []
    for (name, pose) in detector.get_detections():
        position = transform_from_camera_frame_to_robot_frame(pose, q, detector)
        pose_array.append(position)
    return pose_array


def filter_z_axis(v1, v2, v3):
    z_hat = np.array([0, 0, 1])
    if np.abs(np.dot(v1, z_hat)) > 0.9:
        return v2, v3
    elif np.abs(np.dot(v2, z_hat)) > 0.9:
        return v1, v3
    elif np.abs(np.dot(v3, z_hat)) > 0.9:
        return v1, v2
    else:
        assert False, "No z axis found"


def filter_odd(u1, u2):
    a1, b1 = u1[0], u1[1]
    a2, b2 = u2[0], u2[1]
    if a1 * b1 >= 0:
        return u1 if a1 >= 0 else -u1
    else:
        assert a2 * b2 >= 0
        return u2 if a2 >= 0 else -u2


def get_angle_along_z_axis_from_block_pose(block_pose: np.ndarray) -> np.ndarray:
    block_rotation = block_pose[0:3, 0:3]  # robot frame
    assert block_rotation.shape == (3, 3)
    v1, v2, v3 = block_rotation[:, 0], block_rotation[:, 1], block_rotation[:, 2]
    u1, u2 = filter_z_axis(v1, v2, v3)
    w = filter_odd(u1, u2)
    w = w / np.linalg.norm(w)
    angle = np.arctan2(w[1], w[0])

    assert angle >= 0 and angle <= pi / 2
    if angle>pi/4:#Jianning Cui make this change
        angle=angle-pi/2#to make this angle between -45 degrees to 45 degrees
    return angle
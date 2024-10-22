import numpy as np
from lib.solveIK import *
from numba import jit
from RotationForRed import *
from RotationForBlue import *


class Pose:
    def __init__(self, x_in_world_frame, y_in_world_frame, z_in_world_frame, configure_in_robot_frame: np.ndarray=None) -> None:
        self.x = x_in_world_frame
        self.y = y_in_world_frame
        self.z = z_in_world_frame
        self.configure = configure_in_robot_frame

    def toNdarray(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])
        
    def get_configure_in_robot_frame(self) -> np.ndarray:
        return self.configure

class RedConfigInWorldFrame:
    pose_near_static_block_platform = Pose(0.5, -1.159, 0.45, 
                                           configure_in_robot_frame=np.array([-0.17702284114381645, -0.11198700888262901, -0.14061996927118595, -1.8835080821017354, -0.01598806988024311, 1.7725922806772838, 0.4718294740282761,]))
    pose_near_goal_block_platform = Pose(0.562, -0.731, 0.4,
                                         configure_in_robot_frame=np.array([0.3183341340729499, 0.18385853881088152, 0.12493503165087301, -1.6476295848267086, -0.02357143843162585, 1.8300179549052185, 1.2326259938203858]))
    pose_near_turntable = Pose(0.1859, -0.2418, 0.3,
                               configure_in_robot_frame=np.array([1.3063137471456028, 0.8464578956056538, 0.0416820058219431, -0.8615901083815637, -0.03150752200146092, 1.707549680542213, 2.123637791157575]))
    pose_to_observe_turntable = Pose(0, -0.25, 0.4,
                                     configure_in_robot_frame=np.array([1.5199519255149756, 0.7237382285177999, 0.10170677107374718, -0.8175516723799074, -0.06731848274038786, 1.5387924714548873, 2.3795205548010596])) # the z of end need to be tuned
    default_end_effactor_orientation = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]
    ])
    turnable_catching_orientation = np.array([
        [0, 0, -1],
        [0, 1, 0],
        [1, 0, 0]
    ])

class BlueConfigInWorldFrame:
    pose_near_static_block_platform = Pose(0.5, 1.159, 0.45, 
                                           configure_in_robot_frame=np.array([0.10727766087042014, -0.11335505960059783, 0.20637978588031514, -1.8834746903959825, 0.023660364009918274, 1.7724502702569918, 1.0930282642403593]))
    pose_near_goal_block_platform = Pose(0.562, 0.731, 0.4,
                                         configure_in_robot_frame=np.array([-0.226615297366726, 0.1868852940581928, -0.22613146786011062, -1.6475023066665337, 0.04310531981172839, 1.8295057216136847, 0.3254251607574425]))
    pose_near_turntable = Pose(0.1859, 0.2418, 0.3,
                               configure_in_robot_frame=np.array([-1.2310664109291896, 0.8536318255916197, -0.19256688116416423, -0.8599068142843307, 0.14602284723485032, 1.7028816927066872, -0.5924930343513377]))
    pose_to_observe_turntable = Pose(0, 0.25, 0.4,
                                     configure_in_robot_frame=np.array([-1.4499457613474582, 0.7324917181872047, -0.24371998102092363, -0.8156392732560933, 0.16219386088663096, 1.533733885985877, -0.8413036916474111])) # the z of end need to be tuned
    default_end_effactor_orientation = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]
    ])
    turnable_catching_orientation = np.array([
        [0, 0, -1],
        [0, 1, 0],
        [1, 0, 0]
    ])
    
def print_IK_solution(pose_in_world_frame: np.ndarray, 
                      orientation: np.ndarray=RedConfigInWorldFrame.default_end_effactor_orientation, 
                      seed_in_robot_frame: np.ndarray=np.array([0,0,0,-pi/2,0,pi/2,pi/4]),
                      rotation_for_red_or_blue: object=RotationForRed()):
    ik = IK(max_steps=1000)
    target_in_world_frame = Rotation.form_transformation_matrx(orientation=orientation, pose=pose_in_world_frame)
    target_in_robot_frame = rotation_for_red_or_blue.get_robot_frame_from_world_frame(target_in_world_frame)
    solution, is_succeed, config_path = ik.inverse(target=target_in_robot_frame, seed=seed_in_robot_frame)
    
    print("Success: ",is_succeed)
    print("Solution: ", "".join(str(num) + ", " for num in solution))
    print("Iterations:", len(config_path))
    # for q in config_path:
    #     print("".join(str(num) + ", " for num in q))

def get_IK_solution(pose_in_world_frame: np.ndarray, 
                    orientation: np.ndarray=RedConfigInWorldFrame.default_end_effactor_orientation, 
                    seed_in_robot_frame: np.ndarray=np.array([0,0,0,-pi/2,0,pi/2,pi/4]),
                    rotation_for_red_or_blue: object=RotationForRed()):
    ik = IK(max_steps=1000)
    target_in_world_frame = Rotation.form_transformation_matrx(orientation=orientation, pose=pose_in_world_frame)
    target_in_robot_frame = rotation_for_red_or_blue.get_robot_frame_from_world_frame(target_in_world_frame)
    solution, is_succeed, config_path = ik.inverse(target=target_in_robot_frame, seed=seed_in_robot_frame)
    if is_succeed == False:
        raise Exception("no solution for IK")
    return solution

def get_IK_solution_from_pose_and_orientation(pose_orientation_in_robot_frame: np.ndarray, 
                                              seed_in_robot_frame: np.ndarray=np.array([0,0,0,-pi/2,0,pi/2,pi/4])):
    ik = IK(max_steps=1000)
    solution, is_succeed, config_path = ik.inverse(target=pose_orientation_in_robot_frame, seed=seed_in_robot_frame)
    if is_succeed == False:
        raise Exception("no solution for IK")
    return solution

if __name__ == "__main__":
    # get pre-defined configure
    print("get pre-defined configure    ---------------------------------------------------------------------------------")
    print("configure for red robot---------------------------------------------------------------------------------------")
    print_IK_solution(pose_in_world_frame=RedConfigInWorldFrame.pose_near_static_block_platform.toNdarray())
    print_IK_solution(pose_in_world_frame=RedConfigInWorldFrame.pose_near_goal_block_platform.toNdarray())
    print_IK_solution(pose_in_world_frame=RedConfigInWorldFrame.pose_near_turntable.toNdarray(), seed_in_robot_frame=RedConfigInWorldFrame.pose_near_goal_block_platform.get_configure_in_robot_frame())
    print_IK_solution(pose_in_world_frame=RedConfigInWorldFrame.pose_to_observe_turntable.toNdarray(), seed_in_robot_frame=RedConfigInWorldFrame.pose_near_goal_block_platform.get_configure_in_robot_frame())

    print("configure for blue robot--------------------------------------------------------------------------------------")
    print_IK_solution(pose_in_world_frame=BlueConfigInWorldFrame.pose_near_static_block_platform.toNdarray(), rotation_for_red_or_blue=RotationForBlue())
    print_IK_solution(pose_in_world_frame=BlueConfigInWorldFrame.pose_near_goal_block_platform.toNdarray(), rotation_for_red_or_blue=RotationForBlue())
    print_IK_solution(pose_in_world_frame=BlueConfigInWorldFrame.pose_near_turntable.toNdarray(), seed_in_robot_frame=BlueConfigInWorldFrame.pose_near_goal_block_platform.get_configure_in_robot_frame(), rotation_for_red_or_blue=RotationForBlue())
    print_IK_solution(pose_in_world_frame=BlueConfigInWorldFrame.pose_to_observe_turntable.toNdarray(), seed_in_robot_frame=BlueConfigInWorldFrame.pose_near_goal_block_platform.get_configure_in_robot_frame(), rotation_for_red_or_blue=RotationForBlue())

    # Example for get pre-defined configure
    print("Example for getting pre-defined configure    ----------------------------------------------------------------------")
    print(RedConfigInWorldFrame.pose_near_static_block_platform.get_configure_in_robot_frame())
    print(BlueConfigInWorldFrame.pose_near_static_block_platform.get_configure_in_robot_frame())

    # Example for using get_IK_solution()
    print("Example for using get_IK_solution()  --------------------------------------------------------------------------")
    print(get_IK_solution(pose_in_world_frame=RedConfigInWorldFrame.pose_near_static_block_platform.toNdarray()))
    print(get_IK_solution(pose_in_world_frame=RedConfigInWorldFrame.pose_near_turntable.toNdarray(), seed_in_robot_frame=RedConfigInWorldFrame.pose_near_goal_block_platform.get_configure_in_robot_frame()))
    print(get_IK_solution(pose_in_world_frame=BlueConfigInWorldFrame.pose_near_static_block_platform.toNdarray(), rotation_for_red_or_blue=RotationForBlue()))
    print(get_IK_solution(pose_in_world_frame=BlueConfigInWorldFrame.pose_near_turntable.toNdarray(), seed_in_robot_frame=BlueConfigInWorldFrame.pose_near_goal_block_platform.get_configure_in_robot_frame(), rotation_for_red_or_blue=RotationForBlue()))
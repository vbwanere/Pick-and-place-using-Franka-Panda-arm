import sys
import numpy as np
from copy import deepcopy
from math import pi

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

from IntermediatePoints import *
from dynamicPick import RedDynamic

def move_to_predefined_points_demo(arm: ArmController) -> None:
    arm.safe_move_to_position(joint_angles=RedConfigInWorldFrame.pose_near_turntable.get_configure_in_robot_frame())



if __name__ == "__main__":
    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    detector = ObjectDetector()

    start_position = np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
    arm.safe_move_to_position(start_position) # on your mark!

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
        assert False, "Blue team not implemented yet!"
    else:
        print("**  RED TEAM  **")
        dynamic_pick = RedDynamic(arm=arm)
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    # STUDENT CODE HERE

    # get the transform from camera to panda_end_effector
    H_ee_camera = detector.get_H_ee_camera()

    # Detect some blocks...
    for (name, pose) in detector.get_detections():
         print(name,'\n',pose)
    
    # Move around...

    # Move to a dynamic pick slot
    dynamic_pick.move_to_slot()

    # END STUDENT CODE
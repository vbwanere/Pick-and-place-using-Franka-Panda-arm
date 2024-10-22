import sys
from math import pi, sin, cos
import numpy as np
from time import perf_counter
from copy import deepcopy
from math import pi
import rospy
import roslib
import tf
import geometry_msgs.msg
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector
# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds
from lib.solveIK import IK
from lib.calculateFK import FK
#from ../../labs.final.Poseforstaticblock import *
from PoseForStaticBlock import *
from dynamicPick import RedDynamic, BlueDynamic
from IntermediatePoints import *

# Using your solution code
ik = IK()
#############################
##  Transformation Helpers ##
#############################

def trans(d):
    """
    Compute pure translation homogenous transformation
    """
    return np.array([
        [ 1, 0, 0, d[0] ],
        [ 0, 1, 0, d[1] ],
        [ 0, 0, 1, d[2] ],
        [ 0, 0, 0, 1    ],
    ])
def roll(a):
    """
    Compute homogenous transformation for rotation around x axis by angle a
    """
    return np.array([
        [ 1,     0,       0,  0 ],
        [ 0, cos(a), -sin(a), 0 ],
        [ 0, sin(a),  cos(a), 0 ],
        [ 0,      0,       0, 1 ],
    ])
def pitch(a):
    """
    Compute homogenous transformation for rotation around y axis by angle a
    """
    return np.array([
        [ cos(a), 0, -sin(a), 0 ],
        [      0, 1,       0, 0 ],
        [ sin(a), 0,  cos(a), 0 ],
        [ 0,      0,       0, 1 ],
    ])
def yaw(a):
    """
    Compute homogenous transformation for rotation around z axis by angle a
    """
    return np.array([
        [ cos(a), -sin(a), 0, 0 ],
        [ sin(a),  cos(a), 0, 0 ],
        [      0,       0, 1, 0 ],
        [      0,       0, 0, 1 ],
    ])
def transform(d,rpy):
    """
    Helper function to compute a homogenous transform of a translation by d and
    rotation corresponding to roll-pitch-yaw euler angles
    """
    return trans(d) @ roll(rpy[0]) @ pitch(rpy[1]) @ yaw(rpy[2])

#################
##  IK Targets ##
#################
# TODO: Try testing your own targets!
# Note: below we are using some helper functions which make it easier to generate
# valid transformation matrices from a translation vector and Euler angles, or a
# sequence of successive rotations around z, y, and x. You are free to use these
# to generate your own tests, or directly write out transforms you wish to test.
fk=FK()


def static_targets(rb):#get the static blocks based on red or blue
    targets0=[
        transform( np.array([.492, -0.119*rb, 0.48]),    np.array([0,pi,pi])            ),#
        transform( np.array([.498, -0.115*rb, .230]),    np.array([0,pi,pi])            ),#the block!
    ]
    targets1=[
        transform( np.array([.562, -0.119*rb, 0.48]),    np.array([0,pi,pi])            ),#
        transform( np.array([.624, -0.114*rb, .230]),    np.array([0,pi,pi])            ),#the block!
    ]
    targets2=[
        transform( np.array([.492, -0.224*rb, 0.48]),    np.array([0,pi,pi])            ),#
        transform( np.array([.502, -0.224*rb, .230]),    np.array([0,pi,pi])            ),#the block!
    ]
    targets3=[
        transform( np.array([.562, -0.224*rb, 0.48]),    np.array([0,pi,pi])            ),#
        transform(np.array([.632, -0.233*rb, .230]), np.array([0, pi, pi])),  # the block!#by waibhav
    ]
    targetslist=[targets0,targets1,targets2,targets3]#[]#[targets0]#
    return targetslist

def placestatic(ik,j,qseedinit,rb):
    heightj=j*0.05+0.24#height according to j
    dh=0.075
    dx3 = 0
    dy3 = 0
    dz3 = 0.085
    if j==3:
        dx3=-0.07#retreat further to prepare to pick the dynamic blocks
        dy3=0.01
        dz3=0.105#move higher to prepare to pick the dynamic blocks
    targets_d = [
        transform(np.array([.562, 0.169*rb, heightj+dh]), np.array([0, pi, pi])),  #
        transform(np.array([.562, 0.169*rb, heightj]), np.array([0, pi, pi])),  #
        transform(np.array([.562+dx3, (0.169+dy3)*rb, heightj+dz3]), np.array([0, pi, pi])),  #
    ]
    qseed=qseedinit
    for i, target in enumerate(targets_d):
        print("Target " + str(i) + " located at:")
        print(target)  # this target is a 4*4 matrix!
        print("Solving... ")
        q, success, rollout = ik.inverse(target, qseed)  # always starts solving from the neutral position?
        qseed = q  # target

        if success:
            targetpos = target[0:3, 3]
            if targetpos[0] == 0.562 and targetpos[1] == 0.169*rb and (
                    targetpos[2] - heightj) < 0.04:  # ((targetpos[2]-0.2)%0.025)<0.01:#
                arm.safe_move_to_position(q)
                arm.exec_gripper_cmd(0.12)  ##arm.open_gripper()#
            else:
                arm.safe_move_to_position(q)
        else:
            print('IK Failed for this target using this seed.')


def placedynamic(ik,j,qseedinit,detector,rb):
    heightj = j * 0.05 + 0.435#0.44
    dh = 0.075
    dfront = 0.012
    dy = 0
    dj3=0
    if j == 3:
        dh = 0.095
        heightj = 0.24#0.25#0.24
        dfront = -0.1
        dy = 0.1#0.08  #
        dj3=-0.1
    targets_d = [
        # transform(np.array([.554, 0.269 * rb, heightj - dh]), np.array([0, pi, pi])),  #
        transform(np.array([.562 + dfront, (0.169 + dy) * rb, heightj + dh]), np.array([0, pi, pi])),
        # 0.574>0.562 is to account for the dynamic block
        transform(np.array([.562 + dfront, (0.169 + dy) * rb, heightj]), np.array([0, pi, pi])),  #
        transform(np.array([.482+dj3, (0.179-dj3) * rb, heightj + 0.105]), np.array([0, pi, pi])),  #
    ]
    qseed=qseedinit
    for i, target in enumerate(targets_d):
        #print("Target " + str(i) + " located at:")
        #print(target)  # this target is a 4*4 matrix!
        if i==1 and j!=3:#(i == 1 and (j==0 or j==1)) or (i==1 and (j==2 or j==3)):  # 1:#2:#needs vision!
            pose_array = []#this is to detect the current highest block
            for (name, pose) in detector.get_detections():
                position = transform_from_camera_frame_to_robot_frame(pose, qseed, detector)  # block_pos(pose, q)#
                pose_array.append(position)  # position is a 4*4 matrix
            # pose_array=scan_for_blocks_pose_in_robot_frame(q,detector)
            realposelist = pose_array  # scan(qseed)
            if len(realposelist) == 0:  # safety measure to avoid abortion!
                target = target
            else:  #
                #print('realposelist', realposelist)
                realheight=0.225
                for i in range(len(realposelist)):
                    realposei = realposelist[i]
                    # print('realpose0',realpose0)
                    realheighti = realposei[2, 3]
                    if realheighti>realheight:
                        realheight=realheighti#find the highest block
                print('realheight',realheight)
                target = transform(np.array([0.562 + dfront, (0.169 + dy) * rb, realheight+0.065]), np.array([0, pi, pi])) #
        print("Solving... ")
        q, success, rollout = ik.inverse(target, qseed)  # always starts solving from the neutral position?
        qseed = q  # target

        if success:
            targetpos = target[0:3, 3]
            # if i==1:#2:#i==1:#(targetpos==np.array([.562, -0.169, .225])).all():
            # arm.safe_move_to_position(q)
            if targetpos[0] == (0.562 + dfront) and targetpos[1] == (0.169 + dy)*rb and (
                    targetpos[2] - heightj) < 0.04:  # ((targetpos[2]-0.2)%0.025)<0.01:#
                arm.safe_move_to_position(q)
                arm.exec_gripper_cmd(0.12)  ##arm.open_gripper()#
            else:
                arm.safe_move_to_position(q)
        else:
            print('IK Failed for this target using this seed.')

def block_pos(p, q):
    # p- pose of block in camera frame returned using OpenCV library
    # q- robot currect joint configuration

    H_ee_camera = detector.get_H_ee_camera() # the function is provided by TAs\
    # This is pose of camera with respect to the end effector

    jp,T0e = FK().forward(q)
    #print('p',p)
    #print('H_ee_camera',H_ee_camera)
    #print('T0e',T0e)
    pose = T0e @ H_ee_camera @ p
    #print('pose',pose)
    return pose  #returns 4x4 matrix of the block with respect to robot base frame

####################
## Test Execution ##
####################

np.set_printoptions(suppress=True)

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
        print("** BLUE TEAM  **")#waiting for the BlueDynamic
        rb=-1#rb means red or blue. It determines the y coordinates to reach relative to the robot
        dynamic_pick = BlueDynamic(arm=arm)
    else:
        dynamic_pick = RedDynamic(arm=arm)
        print("**  RED TEAM  **")
        rb=1
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    # STUDENT CODE HERE

    # get the transform from camera to panda_end_effector
    H_ee_camera = detector.get_H_ee_camera()

    # Detect some blocks...
    for (name, pose) in detector.get_detections():
         print('name of block',name,'\n','pose of block',pose)
    
    # Move around...
    targetslist=static_targets(rb)
    seed = start_position#arm.neutral_position()  # use neutral configuration as seed
    #print('seed', seed)  # seed [-0.01779206 -0.76012354  0.01978261 -2.34205014  0.02984053  1.54119353  0.75344866]
    qseed = seed#start_position#
    arm.open_gripper()
    for j,targets in enumerate(targetslist):
        for i, target in enumerate(targets):
            #print("Target " + str(i) + " located at:")
            print(target)  # this target is a 4*4 matrix!
            if i==1:#this is the one that is to pick/grab the static blocks#1:#2:#needs vision!
                pose_array = []
                for (name, pose) in detector.get_detections():
                    position = transform_from_camera_frame_to_robot_frame(pose,q,detector)#block_pos(pose, q)#
                    pose_array.append(position)#position is a 4*4 matrix
                realposelist=pose_array#scan(qseed)
                if len(realposelist)==0:#safety measure to avoid abortion!
                    target=target
                else:#
                    k=0#which block to pick?the kth block to pick!
                    for pose in realposelist:
                        xk=pose[0,3]
                        yk=pose[1,3]
                        if j==0:
                            if (xk<=0.562) and (np.abs(yk)<0.169):
                                break
                        elif j==1:
                            if (xk>0.562) and (np.abs(yk)<0.169):
                                break
                        elif j==2:
                            if (xk<=0.562) and (np.abs(yk)>=0.169):
                                break
                        elif j==3:
                            if (xk>0.562) and (np.abs(yk)>=0.169):
                                break
                        k+=1
                    if k>=len(realposelist):
                        k=len(realposelist)-1
                    realpose0=realposelist[k]#print('realpose0',realpose0)
                    realcenter=realpose0[0:3,3]#print('realcenter',realcenter)
                    yawangle=get_angle_along_z_axis_from_block_pose(realpose0)#print('yawangle',yawangle)
                    target = transform(np.array(realcenter), np.array([0, pi, pi - yawangle]))  # realpose0#
            print("Solving... ")
            # q, success, rollout = ik.inverse(target, seed)#always starts solving from the neutral position?
            q, success, rollout = ik.inverse(target, qseed)  # always starts solving from the neutral position?
            qseed = q  # target
            if success:
                targetpos=target[0:3,3]
                if i==1:#2:#i==1:#(targetpos==np.array([.562, -0.169, .225])).all():
                    arm.safe_move_to_position(q)
                    arm.exec_gripper_cmd(0.049, force=100)#close_gripper()#change this!
                elif targetpos[0]==0.562 and targetpos[1]==0.169 and (targetpos[2]-0.240-j*0.05)<0.04:#((targetpos[2]-0.2)%0.025)<0.01:#
                    arm.safe_move_to_position(q)
                    arm.exec_gripper_cmd(0.12)##arm.open_gripper()#
                else:
                    arm.safe_move_to_position(q)
            else:
                print('IK Failed for this target using this seed.')
        #print('qseed',qseed)
        placestatic(ik,j,qseed,rb)

    for j in range(4):
        dynamic_pick.move_to_slot()#pick dynamic blocks
        if team == 'blue':
            qseed = BlueConfigInWorldFrame.pose_near_goal_block_platform.get_configure_in_robot_frame()
        else:
            qseed = RedConfigInWorldFrame.pose_near_goal_block_platform.get_configure_in_robot_frame()#providing init configuration
        placedynamic(ik,j,qseed,detector,rb)#use this function to pick dynamic plocks

    # END STUDENT CODE
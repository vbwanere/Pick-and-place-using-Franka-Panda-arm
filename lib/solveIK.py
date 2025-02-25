import numpy as np
from math import pi, acos
from scipy.linalg import null_space

from lib.calcJacobian import calcJacobian
from lib.calculateFK import FK
from lib.IK_velocity import IK_velocity

from lib.Rotation import *
from math import pi, sin, cos

class IK:

    # JOINT LIMITS
    lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    center = lower + (upper - lower) / 2 # compute middle of range of motion of each joint
    fk = FK()

    def __init__(self,linear_tol=1e-4, angular_tol=1e-3, max_steps=1500, min_step_size=1e-5):
        """
        Constructs an optimization-based IK solver with given solver parameters.
        Default parameters are tuned to reasonable values.

        PARAMETERS:
        linear_tol - the maximum distance in meters between the target end
        effector origin and actual end effector origin for a solution to be
        considered successful
        angular_tol - the maximum angle of rotation in radians between the target
        end effector frame and actual end effector frame for a solution to be
        considered successful
        max_steps - number of iterations before the algorithm must terminate
        min_step_size - the minimum step size before concluding that the
        optimizer has converged
        """

        # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU

        # solver parameters
        self.linear_tol = linear_tol
        self.angular_tol = angular_tol
        self.max_steps = max_steps
        self.min_step_size = min_step_size


    ######################
    ## Helper Functions ##
    ######################

    @staticmethod
    def displacement_and_axis(target, current):
        """
        Helper function for the End Effector Task. Computes the displacement
        vector and axis of rotation from the current frame to the target frame

        This data can also be interpreted as an end effector velocity which will
        bring the end effector closer to the target position and orientation.

        INPUTS:
        target - 4x4 numpy array representing the desired transformation from
        end effector to world
        current - 4x4 numpy array representing the "current" end effector orientation

        OUTPUTS:
        displacement - a 3-element numpy array containing the displacement from
        the current frame to the target frame, expressed in the world frame
        axis - a 3-element numpy array containing the axis of the rotation from
        the current frame to the end effector frame. The magnitude of this vector
        must be sin(angle), where angle is the angle of rotation around this axis
        """

        ## STUDENT CODE STARTS HERE

        displacement = np.zeros(3)
        axis = np.zeros(3)

        displacement = Rotation.get_pose_from_tranformation_matrix(target) - Rotation.get_pose_from_tranformation_matrix(current)

        rotation_from_current_to_target = Rotation.get_rotation_from_transformation_matrix(target) @ Rotation.get_rotation_from_transformation_matrix(current).T
        skew_symmetric = (rotation_from_current_to_target - rotation_from_current_to_target.T) / 2
        vector_coeff = Rotation.get_vector_coefficients_from_skew_symmetric(skew=skew_symmetric)
        axis = vector_coeff

        ## END STUDENT CODE

        return displacement, axis

    @staticmethod
    def distance_and_angle(G, H):
        """
        Helper function which computes the distance and angle between any two
        transforms.

        This data can be used to decide whether two transforms can be
        considered equal within a certain linear and angular tolerance.

        Be careful! Using the axis output of displacement_and_axis to compute
        the angle will result in incorrect results when |angle| > pi/2

        INPUTS:
        G - a 4x4 numpy array representing some homogenous transformation
        H - a 4x4 numpy array representing some homogenous transformation

        OUTPUTS:
        distance - the distance in meters between the origins of G & H
        angle - the angle in radians between the orientations of G & H


        """

        ## STUDENT CODE STARTS HERE

        distance = 0
        angle = 0

        displacement = Rotation.get_pose_from_tranformation_matrix(G) - Rotation.get_pose_from_tranformation_matrix(H)
        distance = np.linalg.norm(x=displacement, ord=2)

        rotation_from_current_to_target = Rotation.get_rotation_from_transformation_matrix(G) @ Rotation.get_rotation_from_transformation_matrix(H).T
        angle = np.arccos(np.clip(a=(np.trace(rotation_from_current_to_target) - 1) / 2, a_min=-1, a_max=1))

        ## END STUDENT CODE

        return distance, angle

    def is_valid_solution(self,q,target):
        """
        Given a candidate solution, determine if it achieves the primary task
        and also respects the joint limits.

        INPUTS
        q - the candidate solution, namely the joint angles
        target - 4x4 numpy array representing the desired transformation from
        end effector to world

        OUTPUTS:
        success - a Boolean which is True if and only if the candidate solution
        produces an end effector pose which is within the given linear and
        angular tolerances of the target pose, and also respects the joint
        limits.
        """

        ## STUDENT CODE STARTS HERE

        success = False

        success = self.is_solution_in_joint_limits(q) and self.is_candidate_similar_to_desired_transformation(actual_joint_angles=q, desired_transformation=target)

        ## END STUDENT CODE

        return success
    
    def is_solution_in_joint_limits(self, actual_joint_angles: np.ndarray) -> bool:
        for i in range(actual_joint_angles.shape[0]):
            if actual_joint_angles[i] < IK.lower[i] or actual_joint_angles[i] > IK.upper[i]:
                return False
            
        return True

    def is_candidate_similar_to_desired_transformation(self, actual_joint_angles: np.ndarray, desired_transformation: np.ndarray) -> bool:
        _, actual_transformation = FK().forward(q=actual_joint_angles)

        pose_difference = np.abs(Rotation.get_pose_from_tranformation_matrix(actual_transformation) - Rotation.get_pose_from_tranformation_matrix(desired_transformation))
        orientation_difference = np.abs(Rotation.get_rotation_from_transformation_matrix(actual_transformation) - Rotation.get_rotation_from_transformation_matrix(desired_transformation))

        return np.all(pose_difference < self.linear_tol) and np.all(orientation_difference < self.angular_tol)

    ####################
    ## Task Functions ##
    ####################

    @staticmethod
    def end_effector_task(q,target):
        """
        Primary task for IK solver. Computes a joint velocity which will reduce
        the error between the target end effector pose and the current end
        effector pose (corresponding to configuration q).

        INPUTS:
        q - the current joint configuration, a "best guess" so far for the final answer
        target - a 4x4 numpy array containing the desired end effector pose

        OUTPUTS:
        dq - a desired joint velocity to perform this task, which will smoothly
        decay to zero magnitude as the task is achieved
        """

        ## STUDENT CODE STARTS HERE

        dq = np.zeros(7)

        _, current_end_effactor_pose = FK().forward(q=q)
        displacement, axis = IK.displacement_and_axis(target=target, current=current_end_effactor_pose)
        def get_linear_velocity_from_displacement(displacement):
            return displacement
        def get_angular_velocity_from_axis(axis):
            return axis
        dq = IK_velocity(q_in=q, v_in=get_linear_velocity_from_displacement(displacement),
                        omega_in=get_angular_velocity_from_axis(axis))

        ## END STUDENT CODE

        return dq

    @staticmethod
    def joint_centering_task(q,rate=5e-1):
        """
        Secondary task for IK solver. Computes a joint velocity which will
        reduce the offset between each joint's angle and the center of its range
        of motion. This secondary task acts as a "soft constraint" which
        encourages the solver to choose solutions within the allowed range of
        motion for the joints.

        INPUTS:
        q - the joint angles
        rate - a tunable parameter dictating how quickly to try to center the
        joints. Turning this parameter improves convergence behavior for the
        primary task, but also requires more solver iterations.

        OUTPUTS:
        dq - a desired joint velocity to perform this task, which will smoothly
        decay to zero magnitude as the task is achieved
        """

        # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU

        # normalize the offsets of all joints to range from -1 to 1 within the allowed range
        offset = 2 * (q - IK.center) / (IK.upper - IK.lower)
        dq = rate * -offset # proportional term (implied quadratic cost)

        return dq

    ###############################
    ## Inverse Kinematics Solver ##
    ###############################

    def inverse(self, target, seed):
        """
        Uses gradient descent to solve the full inverse kinematics of the Panda robot.

        INPUTS:
        target - 4x4 numpy array representing the desired transformation from
        end effector to world
        seed - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6], which
        is the "initial guess" from which to proceed with optimization

        OUTPUTS:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6], giving the
        solution if success is True or the closest guess if success is False.
        success - True if the IK algorithm successfully found a configuration
        which achieves the target within the given tolerance. Otherwise False
        rollout - a list containing the guess for q at each iteration of the algorithm
        """

        q = seed
        rollout = []

        while True:

            rollout.append(q)

            # Primary Task - Achieve End Effector Pose
            dq_ik = self.end_effector_task(q,target)

            # Secondary Task - Center Joints
            dq_center = self.joint_centering_task(q, rate=1)

            ## STUDENT CODE STARTS HERE

            # Task Prioritization
            jacobian = calcJacobian(q_in=q)
            dq = dq_ik + (np.identity(n=jacobian.shape[1]) - np.linalg.pinv(jacobian) @ jacobian) @ dq_center

            # Termination Conditions
            if len(rollout) > self.max_steps or np.linalg.norm(x=dq, ord=2) < self.min_step_size: # TODO: check termination conditions
                break # exit the while loop if conditions are met!

            ## END STUDENT CODE

            q = q + dq

        success = self.is_valid_solution(q,target)

        # printIKResults(q, success, rollout, IK(), target)

        return q, success, rollout

################################
## Simple Testing Environment ##
################################


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

def printIKResults(q, success, rollout, ik, target):
    for i, q in enumerate(rollout):
        joints, pose = ik.fk.forward(q)
        d, ang = IK.distance_and_angle(target,pose)
        print('iteration:',i,' q =',q, ' d={d:3.4f}  ang={ang:3.3f}'.format(d=d,ang=ang))

    print("Success: ",success)
    print("Solution: ",q)
    print("Iterations:", len(rollout))

    _, end_effactor_pose = FK().forward(q=q)
    print("achieved pose: \n", end_effactor_pose)

if __name__ == "__main__":

    np.set_printoptions(suppress=True,precision=5)

    ik = IK()

    # matches figure in the handout
    # seed = np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014,  0.02984053, 1.54119353,  0.75344866])
    # target = transform( np.array([-.2, -.3, .5]), np.array([0,pi,pi]))

    seed = np.array([0,0,0,-pi/2,0,pi/2,pi/4])

    target = np.array([
        [0,-1,0,0.3],
        [-1,0,0,0],
        [0,0,-1,.5],
        [0,0,0, 1],
    ])

    q, success, rollout = ik.inverse(target, seed)

    printIKResults(q, success, rollout, IK(), target)

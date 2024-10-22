import numpy as np
from lib.calculateFK import FK

def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    J = np.zeros((6, 7))

    ## STUDENT CODE GOES HERE
    fk = FK()
    joint_transformation_matrices = fk.get_joint_transformation_matrices(q=q_in)

    On = get_Oi(joint_transformation_matrices[-1])
    jacobian_linear_velocity = []
    jacobian_angular_velocity = []
    for i in range(0, len(joint_transformation_matrices) - 1): # the last one is end-effector, the motion of which won't contribute to the linear of angular velocity of itself
        current_tranformation_matrix = joint_transformation_matrices[i]
        Ri = get_Ri(Ti=current_tranformation_matrix)
        Oi = get_Oi(Ti=current_tranformation_matrix)
        z_hat = np.array([0, 0, 1])
        jacobian_linear_velocity.append( get_skew_symmetric(vector=Ri@z_hat) @ (On - Oi))
        jacobian_angular_velocity.append(Ri @ z_hat)

    J[0:3, :] = np.array(jacobian_linear_velocity).T
    J[3:6, :] = np.array(jacobian_angular_velocity).T

    return J

def get_Ri(Ti: np.ndarray) -> np.ndarray:
    return Ti[0:3, 0:3]

def get_Oi(Ti: np.ndarray) -> np.ndarray:
    return Ti[0:3, 3]

def get_skew_symmetric(vector: np.ndarray) -> np.ndarray:
    return np.array([
        [0, -vector[2], vector[1]],
        [vector[2], 0, -vector[0]],
        [-vector[1], vector[0], 0]
    ])

if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    print(np.round(calcJacobian(q),3))

    q= np.array([0, 0, 0, 0, 0, 0, 0])
    print(np.round(calcJacobian(q),3))

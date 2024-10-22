import numpy as np
from lib.calcJacobian import calcJacobian


def IK_velocity(q_in, v_in, omega_in):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param v_in: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega_in: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 1 x 7 vector corresponding to the joint velocities. If v_in and omega_in
         are infeasible, then dq should minimize the least squares error. If v_in
         and omega_in have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """

    # STUDENT CODE GOES HERE
    dq = np.zeros((1, 7))
    v_in = v_in.reshape((3, 1))
    omega_in = omega_in.reshape((3, 1))

    velocity_jacobian = calcJacobian(q_in=q_in)

    valid_num_index_in_v = ~np.isnan(v_in)
    valid_num_index_in_omega = ~np.isnan(omega_in)

    v_in = v_in[valid_num_index_in_v].reshape((-1,))
    omega_in = omega_in[valid_num_index_in_omega].reshape((-1,))
    velocity_jacobian = np.vstack((velocity_jacobian[0:3, :][valid_num_index_in_v.reshape((3,)), :], 
                                   velocity_jacobian[3:6, :][valid_num_index_in_omega.reshape((3,)), :]))

    dq, residuals, rank, singular_value = np.linalg.lstsq(a=velocity_jacobian, b=np.concatenate((v_in, omega_in), axis=0))
    return dq

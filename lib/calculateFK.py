import numpy as np
from math import pi
# import array_to_latex as a2l

class FK():

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout

        pass

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your Lab 1 code starts here

        jointPositions = np.zeros((8,3))

        joint_transformation_matrices = self.get_joint_transformation_matrices(q=q)

        T0e = joint_transformation_matrices[-1]

        for i in range(len(joint_transformation_matrices)):
            jointPositions[i, :] = joint_transformation_matrices[i][0:3, 3]
        # Your code ends here

        return jointPositions, T0e

    # feel free to define additional helper methods to modularize your solution for lab 1

    def get_joint_transformation_matrices(self, q) -> list:
        Ai = self.compute_Ai(q=q)
        Ti = []
        current_transformation_matrix = np.eye(4)
        for i in range(len(Ai)):
            ai = Ai[i]
            current_transformation_matrix = current_transformation_matrix @ ai
            if i == 2:
                joint_transformation_matrix = current_transformation_matrix @ np.array([
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.195],
                    [0, 0, 0, 1]
                ])
            elif i == 4:
                joint_transformation_matrix = current_transformation_matrix @ np.array([
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.125],
                    [0, 0, 0, 1]
                ])
            elif i == 5:
                joint_transformation_matrix = current_transformation_matrix @ np.array([
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, -0.015],
                    [0, 0, 0, 1]
                ])
            elif i == 6:
                joint_transformation_matrix = current_transformation_matrix @ np.array([
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.051],
                    [0, 0, 0, 1]
                ])
            else:
                joint_transformation_matrix = current_transformation_matrix
            Ti.append(joint_transformation_matrix)
        return Ti
    
    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        # STUDENT CODE HERE: This is a function needed by lab 2
        
        return()
    
    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2
        def rotate_z(angle):
            return np.array([
                [np.cos(angle), -np.sin(angle),         0, 0],
                [np.sin(angle), np.cos(angle),          0, 0],
                [0,                 0,                1,  0],
                [0,                 0,                0,  1]
            ])

        def rotate_x(angle):
            return np.array([
                [1,       0,                0,                0],
                [0,       np.cos(angle), -np.sin(angle),        0],
                [0,       np.sin(angle),   np.cos(angle),       0],
                [0,                 0,                0,      1]
            ])

        def trans_z(distance):
            return np.array([
                [1,0,0,0],
                [0,1,0,0],
                [0,0,1,distance],
                [0,0,0,1]
            ])

        def trans_x(distance):
            return np.array([
            [1,0,0,distance],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1]
            ])

        a = [0,0,0,0.0825,0.0825,0,0.088,0]
        alpha = [0,-np.pi/2,np.pi/2,np.pi/2,np.pi/2,-np.pi/2,np.pi/2,0]
        d = [0.141,0.192,0,0.195+0.121,0,0.125+0.259,0,0.051+0.159]
        q1, q2, q3, q4, q5, q6, q7 = q[0], q[1], q[2], q[3], q[4], q[5], q[6]
        theta = [q1,0,q2,q3,q4+np.pi,q5,q6+np.pi,q7-np.pi/4]

        Ai = []
        for i in range(len(a)):
            Ai.append(rotate_z(angle=theta[i]) @ trans_z(distance=d[i]) @ trans_x(distance=a[i]) @ rotate_x(angle=alpha[i]))
        return Ai
    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    q = np.array([0, 0, 0, -pi/2, 0, pi/2, pi/4])

    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)

    # a2l.to_ltx(T0e, frmt = '{:6.2f}', arraytype = 'bmatrix')

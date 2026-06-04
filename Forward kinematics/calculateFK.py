
import numpy as np
from math import pi

class FK():

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout
        self.a = np.array([0,0,0.0825,0.0825,0,0.088,0]).astype(float)
        self.alpha = np.array([-pi/2, pi/2, pi/2, pi/2, -pi/2, pi/2, 0]).astype(float)
        self.d = np.array([0.192, 0, 0.316, 0, 0.384, 0, 0.21]).astype(float)
        self.theta_offset = np.array([0,0,0,pi,0,pi,-pi/4]).astype(float)
        
        self.offset = [np.eye(4), np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.195],[0,0,0,1]]),np.eye(4),np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.125],[0,0,0,1]]),np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-0.015],[0,0,0,1]]),np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.051],[0,0,0,1]]),np.eye(4)]

        # print(len(self.offset))
        pass

    def dh_A(self, a, alpha, d, theta):
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        A_i = np.array([[ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st], [0, sa, ca, d], [0, 0, 0, 1]])
        return A_i

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
        #T0e = np.identity(4)
        q = np.asarray(q)
        #q[[1, 3]] = -q[[1, 3]]
        T = np.eye(4)
        T[:3,3] = [0,0,0.141]
        jointPositions[0,:] = T[:3,3]
        for i in range(len(self.a)):
            a_i = self.a[i]
            alpha_i = self.alpha[i]
            d_i = self.d[i]
            theta_i = q[i] + self.theta_offset[i]
            Ai = self.dh_A(a_i, alpha_i, d_i, theta_i)
            T = T @ Ai 
            T_pos = T @ self.offset[i]
            jointPositions[i+1,:] = T_pos[0:3,3]
        T0e = T

        # Your code ends here

        return jointPositions, T0e

    # feel free to define additional helper methods to modularize your solution for lab 1

    
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

        return()
    
    def get_T_list(self,q):
        q = np.array(q)
        T = np.eye(4)
        T[:3,3] = [0,0,0.141]
        T_list = [T.copy()]
        for i in range(len(self.a)):
            a_i = self.a[i]
            alpha_i = self.alpha[i]
            d_i = self.d[i]
            theta_i = q[i] + self.theta_offset[i]
            Ai = self.dh_A(a_i, alpha_i, d_i, theta_i)
            T = T @ Ai
            T_list.append(T.copy())
        return T_list

    def calculat_Jacobian(self,q):
        T_list = self.get_T_list(q)
        oe = T_list[-1][:3,3]
        J = np.zeros((6,7))
        for i in range(7):
            T_prev = T_list[i]
            oi = T_prev[:3,3]
            zi = T_prev[:3,2]
            J[:3,i] = np.cross(zi,oe-oi)
            J[3:,i] = zi
        return J

    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    #q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
    q = np.array([0,0,0,0,0,0,0])
    q_dot = np.array([0,0,1,0,0,0,0])

    joint_positions, T0e = fk.forward(q)
    J0 = fk.calculat_Jacobian(q)
    V = J0@q_dot
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
    print("J(0) is", J0)
    print("end effector velocity is:", V)

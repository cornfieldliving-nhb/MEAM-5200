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
    T = np.eye(4)
    T[:3,3] = [0,0,0.141]
    T_list = [T.copy()]
    for i in range(len(fk.a)):
        a_i = fk.a[i]
        d_i = fk.d[i]
        alpha_i = fk.alpha[i]
        theta_i = q_in[i] + fk.theta_offset[i]
        A_i = fk.dh_A(a_i,alpha_i,d_i,theta_i)
        T = T @ A_i
        T_list.append(T.copy())
    oe = T_list[-1][:3,3]
    for j in range(7):
        T_i = T_list[j]
        oi = T_i[:3,3]
        z_i = T_i[:3,2]
        J[:3,j] = np.cross(z_i,(oe-oi))
        J[3:,j] = z_i
    return J


if __name__ == "__main__":
    q = np.array([0, 0, 0, -np.pi / 2, 0, np.pi / 2, np.pi / 4])
    print(np.round(calcJacobian(q), 3))

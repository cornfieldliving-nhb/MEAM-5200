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

    ## STUDENT CODE GOES HERE

    dq = np.zeros((1, 7))

    # v_in = v_in.reshape((3, 1))
    # omega_in = omega_in.reshape((3, 1))
    J = calcJacobian(q_in)
    v_in = np.asarray(v_in).reshape((3,))
    omega_in = np.asarray(omega_in).reshape((3,))
    vel_des = np.hstack((v_in, omega_in))
    J_rows = []
    vel_rows = []
    for i in range(6):
        if np.isnan(vel_des[i]):
            continue
        else:
            J_rows.append(J[i,:])
            vel_rows.append(vel_des[i])
    if len(vel_rows) == 0:
        return np.zeros(7)
    J_reduced = np.asarray(J_rows)
    vel_reduced = np.asarray(vel_rows)
    dq,_,_,_ = np.linalg.lstsq(J_reduced, vel_reduced,rcond=None)
    return dq

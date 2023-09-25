#!/usr/bin/env python

import rospy
import numpy as np
import scipy as sp
import kin_func_skeleton as kfs
from sensor_msgs.msg import JointState

def baxter_forward_kinematics_from_angles(joint_angles):
    """
    Computes the orientation of the Baxter's left end-effector given the joint
    angles in radians.

    Parameters
    ----------
    joint_angles ((7x) np.ndarray): 7 joint angles (s0, s1, e0, e1, w0, w1, w2)

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """

    qs = np.ndarray((3,8)) # points on each joint axis in the zero configuration
    ws = np.ndarray((3,7)) # axis vector of each joint axis
    
    # Assign the q values
    qs[0:3,0] = [0.0635, 0.2598, 0.1188]
    qs[0:3,1] = [0.1106, 0.3116, 0.3885]
    qs[0:3,2] = [0.1827, 0.3838, 0.3881]
    qs[0:3,3] = [0.3682, 0.5684, 0.3181]
    qs[0:3,4] = [0.4417, 0.6420, 0.3177]
    qs[0:3,5] = [0.6332, 0.8337, 0.3067]
    qs[0:3,6] = [0.7152, 0.9158, 0.3063]
    qs[0:3,7] = [0.7957, 0.9965, 0.3058]

    # Assign the w values
    ws[0:3,0] = [-0.0059,  0.0113,  0.9999]
    ws[0:3,1] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,2] = [ 0.7065,  0.7077, -0.0038]
    ws[0:3,3] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,4] = [ 0.7065,  0.7077, -0.0038]
    ws[0:3,5] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,6] = [ 0.7065,  0.7077, -0.0038]

    R = np.array([[0.0076, 0.0001, -1.0000],
                    [-0.7040, 0.7102, -0.0053],
                    [0.7102, 0.7040, 0.0055]]).T # rotation matrix of zero config

    # YOUR CODE HERE (Task 1)
    g_st0 = np.eye(4)
    g_st0[0:3, 0:3] = R 
    g_st0[0:3,3] = qs[0:3,7]
    g_st0[3,3] = 1
    # print('g_st0: ', g_st0)

    xi_array = np.ndarray((8,6))
    xi_array[0] = get_xi(ws[0:3,0], qs[0:3,0])
    xi_array[1] = get_xi(ws[0:3,1], qs[0:3,1])
    xi_array[2] = get_xi(ws[0:3,2], qs[0:3,2])
    xi_array[3] = get_xi(ws[0:3,3], qs[0:3,3])
    xi_array[4] = get_xi(ws[0:3,4], qs[0:3,4])
    xi_array[5] = get_xi(ws[0:3,5], qs[0:3,5])
    xi_array[6] = get_xi(ws[0:3,6], qs[0:3,6])
    xi_array[7] = np.array([ 0.7065,  0.7077, -0.0038, 0, 0, 0])
    xi_array = xi_array.T
    # print('test 1\n ',xi_array)
    # print('test-------------\n',kfs.prod_exp(xi_array, joint_angles),'\nend test-------------\n')
    g = np.matmul(kfs.prod_exp(xi_array, joint_angles), g_st0)

    return g

def get_xi(w, q):
    
    qxw = -np.cross(w, q)
    xi = np.array([qxw[0], qxw[1], qxw[2], w[0], w[1], w[2]])

    # print(-np.cross(w, q), w, q)
    # xi[0:3] = -np.cross(w, q)
    # print('x 1', xi)
    # xi[3:6] = w
    # w = np.array(w)
    # q = np.array(q)
    # print('xi test', np.concatenate(np.cross(q, w), w))
    # print('xi 2', xi)
    return xi

def baxter_forward_kinematics_from_joint_state(joint_state):
    """
    Computes the orientation of the Baxter's left end-effector given the joint
    state.

    Parameters
    ----------
    joint_state (sensor_msgs.JointState): JointState of Baxter robot

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    
    angles = np.zeros(7)

    # YOUR CODE HERE (Task 2)
    angles = np.array(joint_state.position[2:9])
    angles[0:2], angles[2:4] = angles[2:4], angles[0:2]

    # END YOUR CODE HERE
    print(baxter_forward_kinematics_from_angles(angles))


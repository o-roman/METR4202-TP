#!/usr/bin/env python3
from re import T
import numpy as np

def d2r(deg):
    """Simply change degree angle to radius angle
    """
    rad = deg/180*np.pi
    return rad

def third_poly_scale(t,T):
    """Scale the time domain using 3rd order polynomial

       Input: mapping t seconds to s domain in T seconds, sample on each sec, t[0,T] to s[0,1]
    """
    s = 3 * (t/T)**2 - 2 * (t/T)**3
    return s

def fifth_poly_scale(t,T):
    """Scale the time domain using 5th order polynomial
    """
    s = 10 * (t/T)**3 - 15 * (t/T)**4 + 6 * (t/T)**5
    return s

def seventh_poly_scale(t,T):
    """Scale the time domain using 7th order polynomial
    """
    s = 35 * (t/T)**4 - 84 * (t/T)**5 + 70 * (t/T)**6 - 20 * (t/T)**7
    return s


def joint_validation(joint_ans):
    """Validate all joint answers to select the best one to generate joint
       trajecoy

       Input: joint_ans: theta lists in array form

       Implementation for now: Validate answers with angle limits then only rewrite [i] answer with [i+1] answer
    """
    N = len(joint_ans)

    first_valid = np.zeros((N, 4))
    
    for i in range(N):
        joint_angle_1 = joint_ans[i, 0]
        joint_angle_2 = joint_ans[i, 1]
        joint_angle_3 = joint_ans[i, 2]
        joint_angle_4 = joint_ans[i, 3]
        
        if d2r(-40) <= joint_angle_1 <= d2r(40) and 0 <= joint_angle_2 <= d2r(90) and 0 <= joint_angle_3 <= d2r(150) and 0 <= joint_angle_4 <= (np.pi - joint_angle_2 - joint_angle_3):
            first_valid[i] = joint_ans[i]
        else:
            pass

    # n = 0
    
    valid_ans = np.zeros((1,4))
    for i in range(N):
        if list(first_valid[i]) != [0,0,0,0]:
            valid_ans = first_valid[i]
        else:
            pass

    # if n == 1:
    #     valid_ans = np.delete((re_shape,
    #     return valid_ans
    # elif n > 1:
    #     valid_ans = np.zeros

    #return first_valid
    return valid_ans

def joint_traj_gen(theta_end,T,scale):
    """Generate trajectory from valid angle answers, sample at each second
    """
    theta_start = np.array([0,0,0,0])
    theta_s = np.zeros((T,4))
    for i in range(T):
        if scale == 3:
            s = third_poly_scale(i+1,T)
        elif scale == 5:
            s = fifth_poly_scale(i+1,T)
        elif scale == 7:
            s = seventh_poly_scale(i+1,T)
        else:
            print("Please enter a proper scale method from 3,5,7")
        theta_s[i] = theta_start + s * (theta_end - theta_start)
    
    # theta_s[0] = theta_start + s * (theta_start - theta_end) * (1/T)
    # theta_s[1] = theta_start + s * (theta_start - theta_end) * (2/T)
    # theta_s[2] = theta_start + s * (theta_start - theta_end) * (3/T)
    # theta_s[3] = theta_start + s * (theta_start - theta_end) * (4/T)
    # theta_s[4] = theta_start + s * (theta_start - theta_end) * (5/T)
    

    return theta_s

def back2home_traj_gen(theta_end,T,scale):
    """Generate trejactory from current pose to home pose, sample at each second
    """
    joint_traj = joint_traj_gen(theta_end,T,scale)
    theta_b2h = np.zeros((T,4))
    for i in range(T-1):
        theta_b2h[i] = joint_traj[T-2-i]
    
    # theta_b2h[0] = joint_traj[3]
    # theta_b2h[1] = joint_traj[2]
    # theta_b2h[2] = joint_traj[1]
    # theta_b2h[3] = joint_traj[0]
    # theta_b2h[4] = theta_start
    
    
    return theta_b2h

def home2zone_traj_gen(theta_end,T,scale):
    """Generate trajectory from home pose to required zone
    """
    theta_start = np.array([0,0,0,0])
    theta_h2z = np.zeros((T,4))
    for i in range(T):
        if scale == 3:
            s = third_poly_scale(i+1,T)
        elif scale == 5:
            s = fifth_poly_scale(i+1,T)
        elif scale == 7:
            s = seventh_poly_scale(i+1,T)
        else:
            print("Please enter a proper scale method from 3,5,7")
        theta_h2z[i] = theta_start + s * (theta_end - theta_start)
    
    # theta_s = np.zeros((6,4))
    
    # theta_s[0] = theta_start
    # theta_s[1] = theta_start + s * (theta_start - theta_end) * 0.2
    # theta_s[2] = theta_start + s * (theta_start - theta_end) * 0.4
    # theta_s[3] = theta_start + s * (theta_start - theta_end) * 0.6
    # theta_s[4] = theta_start + s * (theta_start - theta_end) * 0.8
    # theta_s[5] = theta_start + s * (theta_start - theta_end) * 1
    
    return theta_h2z
    
    # zone1 = np.asarray([[   0,    0,  0,    0],
    #                     [21.6,  4.2, 14, 15.6],
    #                     [43.2,  8.4, 28, 31.2],
    #                     [64.8, 12.6, 42, 46.8],
    #                     [86.4, 16.8, 56, 62.4],
    #                     [ 108,   21, 70,   78]])
    # zone2 = np.asarray([[    0,    0,  0,    0],
    #                     [ 32.4,  4.2, 14, 15.6],
    #                     [ 64.8,  8.4, 28, 31.2],
    #                     [ 97.2, 12.6, 42, 46.8],
    #                     [129.6, 16.8, 56, 62.4],
    #                     [  162,   21, 70,   78]])
    # zone3 = np.asarray([[    0,    0,  0,    0],
    #                     [-21.6,  4.2, 14, 15.6],
    #                     [-43.2,  8.4, 28, 31.2],
    #                     [-64.8, 12.6, 42, 46.8],
    #                     [-86.4, 16.8, 56, 62.4],
    #                     [ -108,   21, 70,   78]])
    # zone4 = np.asarray([[     0,    0,  0,    0],
    #                     [ -32.4,  4.2, 14, 15.6],
    #                     [ -64.8,  8.4, 28, 31.2],
    #                     [ -97.2, 12.6, 42, 46.8],
    #                     [-129.6, 16.8, 56, 62.4],
    #                     [  -162,   21, 70,   78]])

    # if zone == 1:
    #     return d2r(zone1)
    # elif zone == 2:
    #     return d2r(zone2)
    # elif zone == 3:
    #     return d2r(zone3)
    # elif zone == 4:
    #     return d2r(zone4)
    # else:
    #     print("Please select a proper drop zone 1,2,3,4")
    

def zone2home_traj_gen(theta_end,T,scale):
    """Generate trajectory from zone pose to home pose
    """
    # if zone == 1:
    #     z2h = np.asarray([[ 108,   21, 70,   78],
    #                       [86.4, 16.8, 56, 62.4],
    #                       [64.8, 12.6, 42, 46.8],
    #                       [43.2,  8.4, 28, 31.2],
    #                       [21.6,  4.2, 14, 15.6],
    #                       [   0,    0,  0,    0]])
    # elif zone == 2:
    #     z2h = np.asarray([[  162,   21, 70,   78],
    #                       [129.6, 16.8, 56, 62.4],
    #                       [ 97.2, 12.6, 42, 46.8],
    #                       [ 64.8,  8.4, 28, 31.2],
    #                       [ 32.4,  4.2, 14, 15.6],
    #                       [    0,    0,  0,    0]])
    # elif zone == 3:
    #     z2h = np.asarray([[ -108,   21, 70,   78],
    #                       [-86.4, 16.8, 56, 62.4],
    #                       [-64.8, 12.6, 42, 46.8],
    #                       [-43.2,  8.4, 28, 31.2],
    #                       [-21.6,  4.2, 14, 15.6],
    #                       [    0,    0,  0,    0]])
    # elif zone == 4:
    #     z2h = np.asarray([[  -162,   21, 70,   78],
    #                       [-129.6, 16.8, 56, 62.4],
    #                       [ -97.2, 12.6, 42, 46.8],
    #                       [ -64.8,  8.4, 28, 31.2],
    #                       [ -32.4,  4.2, 14, 15.6],
    #                       [     0,    0,  0,    0]])
    # else:
    #     print("Please select a proper drop zone 1,2,3,4")
    joint_traj = joint_traj_gen(theta_end,T,scale)
    traj_z2h = np.zeros((T,4))
    for i in range(T-1):
        traj_z2h[i] = joint_traj[T-2-i]
   
    return traj_z2h

def end_point_traj_gen(X_start,X_end,t,T,scale):
    """
    Sample Text
    """
    if scale == 3:
        s = third_poly_scale(t,5)
    elif scale == 5:
        s = fifth_poly_scale(t,5)
    else:
        s = seventh_poly_scale(t,5)
    
    X_s = X_start + s * (X_end - X_start)

# def end_point_validation(X_s):
#
#     if X_s 

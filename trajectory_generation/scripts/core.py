#!/usr/bin/env python3
import numpy as np

def third_poly_scale(t,T):
    """
    Sample Text
    """

    s = 3 * (t/T)**2 - 2 * (t/T)**3
    return s

def fifth_poly_scale(t,T):
    """
    Sample Text
    """
    s = 10 * (t/T)**3 - 15 * (t/T)**4 + 6 * (t/T)**5
    return s

def seventh_poly_scale(t,T):
    """
    Sample Text
    """
    s = 35 * (t/T)**4 - 84 * (t/T)**5 + 70 * (t/T)**6 - 20 * (t/T)**7
    return s

def joint_validation(joint_ans, T, scale):
    """
    Sample Text
    """
    N = len(joint_ans)

    valid_ans = np.zeros(N, 4)
    
    for i in range(joint_ans):
        joint_angle_1 = joint_ans[i][0]
        joint_angle_2 = joint_ans[i][1]
        joint_angle_3 = joint_ans[i][2]
        joint_angle_4 = joint_ans[i][3]
        if -40 <= joint_angle_1 <= 40 and 0 <= joint_angle_2 <= 90 and 0 <= joint_angle_3 <= 150 and 0 <= joint_angle_4 <= (180 - joint_angle_2 - joint_angle_3):
            valid_ans[i] = joint_ans(i)
        else:
            pass
    
    return valid_ans

def joint_traj_gen(theta_start,theta_end,t,T,scale):
    """
    Sample Text
    """
    if scale == 3:
        s = third_poly_scale(t,T)
    elif scale == 5:
        s = fifth_poly_scale(t,T)
    elif scale == 7:
        s = seventh_poly_scale(t,T)
    else:
        print("Please enter a proper scale method from 3,5,7")
    
    theta_s = np.zeros(5,4)
    theta_s[0] = theta_start + s * (theta_start - theta_end) * 0.2
    theta_s[1] = theta_start + s * (theta_start - theta_end) * 0.4
    theta_s[2] = theta_start + s * (theta_start - theta_end) * 0.6
    theta_s[3] = theta_start + s * (theta_start - theta_end) * 0.8
    theta_s[4] = theta_start + s * (theta_start - theta_end) * 1

    return theta_s

def back2home_traj(theta_s):
    """
    Sample Text
    """
    theta_b2h = np.zeros(5,4)
    theta_b2h[0] = theta_s[4]
    theta_b2h[1] = theta_s[3]
    theta_b2h[2] = theta_s[2]
    theta_b2h[3] = theta_s[1]
    theta_b2h[4] = theta_s[0]
    
    return theta_b2h

def home2zone(zone):
    """
    Sample Text
    """
    zone1 = np.asarray([[   0,    0,  0,    0],
                        [21.6,  4.2, 14, 15.6],
                        [43.2,  8.4, 28, 31.2],
                        [64.8, 12.6, 42, 46.8],
                        [86.4, 16.8, 56, 62.4],
                        [ 108,   21, 70,   78]])
    zone2 = np.asarray([[    0,    0,  0,    0],
                        [ 32.4,  4.2, 14, 15.6],
                        [ 64.8,  8.4, 28, 31.2],
                        [ 97.2, 12.6, 42, 46.8],
                        [129.6, 16.8, 56, 62.4],
                        [  162,   21, 70,   78]])
    zone3 = np.asarray([[    0,    0,  0,    0],
                        [-21.6,  4.2, 14, 15.6],
                        [-43.2,  8.4, 28, 31.2],
                        [-64.8, 12.6, 42, 46.8],
                        [-86.4, 16.8, 56, 62.4],
                        [ -108,   21, 70,   78]])
    zone4 = np.asarray([[     0,    0,  0,    0],
                        [ -32.4,  4.2, 14, 15.6],
                        [ -64.8,  8.4, 28, 31.2],
                        [ -97.2, 12.6, 42, 46.8],
                        [-129.6, 16.8, 56, 62.4],
                        [  -162,   21, 70,   78]])

    if zone == 1:
        return zone1
    elif zone == 2:
        return zone2
    elif zone == 3:
        return zone3
    elif zone == 4:
        return zone4
    else:
        print("Please select a proper drop zone 1,2,3,4")

def zone2home(zone):
    """
    Sample Text
    """
    if zone == 1:
        z2h = np.asarray([[],
                          [],
                          [],
                          [],
                          []])
    elif zone == 2:
        z2h = np.asarray([[],
                          [],
                          [],
                          [],
                          []])
    elif zone == 3:
        z2h = np.asarray([[],
                          [],
                          [],
                          [],
                          []])
    elif zone == 4:
        z2h = np.asarray([[],
                          [],
                          [],
                          [],
                          []])
    else:
        print("Please select a proper drop zone 1,2,3,4")
    
    return z2h

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
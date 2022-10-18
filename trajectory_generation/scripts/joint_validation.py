#!/usr/bin/env python3

import numpy as np

def joint_validation(joint_ans, T, scale):
   
    N = len(joint_ans)

    valid_scaled_ans = np.zeros(N, 4)
    
    for i in range(joint_ans):
        joint_angle_1 = joint_ans[i][0]
        joint_angle_2 = joint_ans[i][1]
        joint_angle_3 = joint_ans[i][2]
        joint_angle_4 = joint_ans[i][3]
        if -45 <= joint_angle_1 <= 45 and 0 <= joint_angle_2 <= 90 and 0 <= joint_angle_3 <= 150 and 0 <= joint_angle_4 <= 90:
            valid_scaled_ans[i] = joint_traj_gen(np.array([0,0,0,0]), joint_ans[i], t, T, scale)
        else:
            pass
    
    return valid_scaled_ans
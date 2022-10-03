#!/usr/bin/env python3

import numpy as np

def joint_traj_gen(theta_start,theta_end,t,T,scale):
    
    if scale == 3:
        s = third_poly_scale(t,T)
    else if scale == 5:
        s = fifth_poly_scale(t,T)
    else:
        s = seventh_poly_scale(t,T)
    
    theta_s = theta_start + s * (theta_start - theta_end)

    return theta_s

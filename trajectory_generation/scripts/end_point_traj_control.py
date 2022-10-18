#!/usr/bin/env python3

import numpy as np

def end_point_traj_gen(X_start,X_end,t,T,scale):
    
    if scale == 3:
        s = third_poly_scale(t,5)
    elif scale == 5:
        s = fifth_poly_scale(t,5)
    else:
        s = seventh_poly_scale(t,5)
    
    X_s = X_start + s * (X_end - X_start)

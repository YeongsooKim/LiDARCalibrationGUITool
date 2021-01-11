# -*- coding: utf-8 -*-
"""
Created on Sat Jun 16 16:01:41 2018

@author: Chansoo
"""

import numpy as np
from process import utils_gnss

def get_gnss_enu(df_gnss, ref):
    TargetWGS84 = np.vstack((df_gnss['latitude'], df_gnss['longitude'] ))
    TargetWGS84 = np.transpose(TargetWGS84)
    tmpENU = utils_gnss.cvt_llh_enu(ref, TargetWGS84)
    df_gnss['east_m'] = tmpENU[:,0]
    df_gnss['north_m'] = tmpENU[:,1]
    return df_gnss

def get_motion_enu(df_motion, ref):
    # start point enu
    prev_east = ref[0]      # m
    prev_north = ref[1]     # m
    prev_heading = ref[2]   # deg
    
    # Forward Predict
    pred_east = [prev_east]
    pred_north = [prev_north]
    pred_heading = [prev_heading]
    for i in list(range(df_motion.shape[0]-1)):
        dt = df_motion.index[i+1]-df_motion.index[i]
        
        # compute moving
        diff_distance = df_motion['speed_x'].values[i] * dt
        diff_heading = df_motion['yaw_rate'].values[i] * dt
    
        # Get difference
        prev_heading_ = prev_heading + 90
        next_east = prev_east + diff_distance * np.cos(prev_heading_ * np.pi / 180.)
        next_north = prev_north + diff_distance * np.sin(prev_heading_ * np.pi / 180.)
        next_heading = prev_heading + diff_heading
    
        # Set ENH
        pred_east.append(next_east)
        pred_north.append(next_north)
        pred_heading.append(next_heading)
    
        # Update ENU
        prev_east = next_east
        prev_north = next_north
        prev_heading = next_heading
    
    df_motion['dr_east_m'] = pred_east
    df_motion['dr_north_m'] = pred_north
    df_motion['dr_heading'] = pred_heading
    
    return df_motion
        
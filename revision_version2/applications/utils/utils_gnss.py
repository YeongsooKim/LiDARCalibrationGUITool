#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Chansoo Kim (chansoo7857@gmail.com)
"""

import numpy as np

Geod_a = 6378137.0
Geod_e2 = 0.00669437999014
RAD2DEG = 180 / np.pi
DEG2RAD = np.pi / 180
mm2m = 1 / 1000
XPixelMeter = 0.024699619563463528

# Convert WGS84 coordinate to ENU coordinate
def cvt_llh_enu(RefWGS84, TargetWGS84):
    if TargetWGS84.shape[1] == 2:
        height = np.zeros(TargetWGS84.shape[0])
        
    elif TargetWGS84.shape[1] == 3:
        height = TargetWGS84[:,2]

    dKappaLat = get_kappa_lat(RefWGS84[0], height)
    dKappaLon = get_kappa_lon(RefWGS84[0], height)
        
    dEast_m = np.divide((TargetWGS84[:,1] - RefWGS84[1]), dKappaLon)
    dNorth_m = np.divide((TargetWGS84[:,0] - RefWGS84[0]), dKappaLat)
    dHeight_m = height

    TargetENU = dEast_m
    TargetENU = np.vstack((TargetENU, dNorth_m))
    TargetENU = np.vstack((TargetENU, dHeight_m))
    TargetENU = np.transpose(TargetENU)

    return TargetENU

# Convert ENU coordinate to WGS84 coordinate
def cvt_enu_llh(RefWGS84, TargetENU):
    if TargetENU.shape[1] == 2:
        height = np.zeros(TargetENU.shape[0])
        
    elif TargetENU.shape[1] == 3:
        height = TargetENU[:,2]

    dKappaLat = get_kappa_lat(RefWGS84[0], height)
    dKappaLon = get_kappa_lon(RefWGS84[0], height)

    dLatitude_deg = RefWGS84[0] + np.multiply(dKappaLat, TargetENU[:,1])
    dLongitude_deg = RefWGS84[1] + np.multiply(dKappaLon, TargetENU[:,0])
    dHeight_m = height
    
    TargetLLH = dLatitude_deg
    TargetLLH = np.vstack((TargetLLH, dLongitude_deg))
    TargetLLH = np.vstack((TargetLLH, dHeight_m))
    TargetLLH = np.transpose(TargetLLH)

    return TargetLLH

def get_kappa_lat(dLatitude, dHeight):
    dKappaLat = 0
    Denominator = 0
    dM = 0

    Denominator = np.sqrt(1 - Geod_e2 * pow(np.sin(dLatitude * DEG2RAD), 2))
    dM = Geod_a * (1 - Geod_e2) / pow(Denominator, 3)
    
    dKappaLat = 1 / (dM + dHeight) * RAD2DEG
    
    return dKappaLat

def get_kappa_lon(dLatitude, dHeight):
    dKappaLon = 0
    Denominator = 0
    dN = 0
    
    Denominator = np.sqrt(1 - Geod_e2 * pow(np.sin(dLatitude * DEG2RAD), 2))
    dN = Geod_a / Denominator
    
    dKappaLon = 1 / ((dN + dHeight) * np.cos(dLatitude * DEG2RAD)) * RAD2DEG
    
    return dKappaLon

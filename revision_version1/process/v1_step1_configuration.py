# -*- coding: utf-8 -*-
"""
@author: chansoo7857@gmail.com
@date: 2018-07-12
@version: 0.0.1
"""

##############################################################################################################################
# %% Import libraries
##############################################################################################################################

# Basic modules in Anaconda
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import open3d as o3d

import pickle
import random
from sklearn.neighbors import NearestNeighbors
from scipy.optimize import minimize
import configparser

# Additional modules
from tqdm import tqdm

# User defined modules
from process import utils_icp
from process import utils_file
from process import utils_pointcloud
from process import utils_pose

class Configuration:
    def __init__(self):
        # Path and file
        self.configuration_file = ''
        self.PARM_LIDAR = {}
        self.PARM_PC = {}
        self.PARM_MO = {}
        self.PARM_HE = {}
        self.PATH = {}

    ##############################################################################################################################
    # %% 1. Set configuration
    ##############################################################################################################################
    def InitConfiguration(self):
        config_param = configparser.ConfigParser()
        config_param.read(self.configuration_file)

        # LIDAR
        self.PARM_LIDAR['PrincipalSensor'] = int(config_param['LIDAR']['PrincipalSensor'])
        self.PARM_LIDAR['CheckedSensorList'] = list(map(int, config_param['LIDAR']['CheckedSensorList'].split()))
        self.PARM_LIDAR['SensorList'] = list(map(int, config_param['LIDAR']['SensorList'].split()))

        # PointCloud
        self.PARM_PC['MinThresholdDist_m'] = float(config_param['PointCloud']['MinThresholdDist_m'])
        self.PARM_PC['MaxThresholdDist_m'] = float(config_param['PointCloud']['MaxThresholdDist_m'])
        self.PARM_PC['MinThresholdX_m'] = float(config_param['PointCloud']['MinThresholdX_m'])
        self.PARM_PC['MaxThresholdX_m'] = float(config_param['PointCloud']['MaxThresholdX_m'])
        self.PARM_PC['MinThresholdY_m'] = float(config_param['PointCloud']['MinThresholdY_m'])
        self.PARM_PC['MaxThresholdY_m'] = float(config_param['PointCloud']['MaxThresholdY_m'])
        self.PARM_PC['MinThresholdZ_m'] = float(config_param['PointCloud']['MinThresholdZ_m'])
        self.PARM_PC['MaxThresholdZ_m'] = float(config_param['PointCloud']['MaxThresholdZ_m'])

        # Handeye
        self.PARM_HE['SamplingInterval'] = int(config_param['Handeye']['SamplingInterval'])
        self.PARM_HE['MaximumIteration'] = int(config_param['Handeye']['MaximumIteration'])
        self.PARM_HE['Tolerance'] = float(config_param['Handeye']['Tolerance'])
        self.PARM_HE['OutlierDistance_m'] = float(config_param['Handeye']['OutlierDistance_m'])
        self.PARM_HE['filter_HeadingThreshold'] = float(config_param['Handeye']['filter_HeadingThreshold'])
        self.PARM_HE['filter_DistanceThreshold'] = float(config_param['Handeye']['filter_DistanceThreshold'])

        # Optimization
        self.PARM_MO['PoseSamplingRatio'] = float(config_param['MultiOptimization']['PoseSamplingRatio'])
        self.PARM_MO['PointSamplingRatio'] = float(config_param['MultiOptimization']['PointSamplingRatio'])
        self.PARM_MO['NumPointsPlaneModeling'] = int(config_param['MultiOptimization']['NumPointsPlaneModeling'])
        self.PARM_MO['OutlierDistance_m'] = float(config_param['MultiOptimization']['OutlierDistance_m'])

        # Path
        self.PATH['Configuration'] = config_param['Path']['Configuration']
        self.PATH['Logging_file_path'] = config_param['Path']['Logging_file_path']

        print('Initialize configation parameter')

    def WriteDefaultFile(self):
        file = 'configuration/default.ini'
        self.WriteFile(file)
        self.configuration_file = file

    def WriteFile(self, file):
        f = open(file, 'w', encoding=None)
        f.write('[LIDAR]\n')
        f.write('PrincipalSensor = 0\n')
        f.write('CheckedSensorList = 0\n')
        f.write('SensorList = 0\n')
        f.write('\n')
        f.write('[PointCloud]\n')
        f.write('MinThresholdDist_m = 1.0\n')
        f.write('MaxThresholdDist_m = 50.0\n')
        f.write('MinThresholdX_m = -10000.0\n')
        f.write('MaxThresholdX_m = 10000.0\n')
        f.write('MinThresholdY_m = -10000.0\n')
        f.write('MaxThresholdY_m = 10000.0\n')
        f.write('MinThresholdZ_m = -10000.0\n')
        f.write('MaxThresholdZ_m = 10000.0\n')
        f.write('\n')
        f.write('[Handeye]\n')
        f.write('SamplingInterval = 1\n')
        f.write('MaximumIteration = 100\n')
        f.write('Tolerance = 0.0000001\n')
        f.write('OutlierDistance_m = 0.5\n')
        f.write('filter_HeadingThreshold = 0.05\n')
        f.write('filter_DistanceThreshold = 0.05\n')
        f.write('\n')
        f.write('[MultiOptimization]\n')
        f.write('PoseSamplingRatio = 0.1\n')
        f.write('PointSamplingRatio = 0.2\n')
        f.write('NumPointsPlaneModeling = 10\n')
        f.write('OutlierDistance_m = 5.\n')
        f.write('\n')
        f.write('[Path]\n')
        path = os.getcwd().replace('\\', '/')
        configuration_path = 'Configuration = ' + path + '/configuration\n'
        logging_data_path = 'Logging_file_path = ' + path + '/logging_data\n'
        f.write(configuration_path)
        f.write(logging_data_path)
        f.close()

        print('Write default configuration parameter in ' + file)
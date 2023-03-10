# -*- coding: utf-8 -*-
"""
@author: chansoo7857@gmail.com
@date: 2018-07-12
@version: 0.0.1
"""

# Basic modules in Anaconda
import os
import configparser
import copy

class Configuration:
    path = os.getcwd().replace('\\', '/') # path is gui_tool.exe directory
    configuration_path = 'common/configuration/'
    logging_data_path = 'common/logging_data/'
    image_path = 'common/image/'
    vehicle_mesh_path = 'common/meshes/vehicles/'
    lidar_mesh_path = 'common/meshes/lidars/'
    grid_mesh_path = 'common/meshes/grid/'

    def __init__(self):
        # Path and file
        self.configuration_file = ''
        self.PARM_LIDAR = {}
        self.PARM_PC = {}
        self.PARM_IM = {}
        self.PARM_ZRP = {}
        self.PARM_ROI_DICT = {}
        self.PARM_DV = {}
        self.PARM_HE = {}
        self.PARM_SO = {}
        self.PARM_MO = {}
        self.PARM_EV = {}
        self.PATH = {}
        self.VEHICLE_INFO = {}
        self.CalibrationParam = {}

        self.value_changed = False
        self.PARMS = [self.PARM_LIDAR, self.PARM_PC, self.PARM_IM, self.PARM_ZRP,
                      self.PARM_DV, self.PARM_HE, self.PARM_SO,
                      self.PARM_MO, self.PARM_EV, self.PATH]
        self.PREV_PARMS = []

    ##############################################################################################################################
    # %% 1. Set configuration
    ##############################################################################################################################
    def InitConfiguration(self):
        config_param = configparser.ConfigParser()
        config_param.read(self.configuration_file)

        # LIDAR
        self.PARM_LIDAR['SingleSensor'] = int(config_param['LIDAR']['SingleSensor'])
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

        # Import
        self.PARM_IM['SamplingInterval'] = int(config_param['Import']['SamplingInterval'])
        self.PARM_IM['VehicleSpeedThreshold'] = float(config_param['Import']['VehicleSpeedThreshold'])

        # Z, Roll, Pitch: Calibration
        self.PARM_ZRP['MaxDistanceX_m'] = float(config_param['ZRPCalibration']['MaxDistanceX_m'])
        self.PARM_ZRP['MinDistanceX_m'] = float(config_param['ZRPCalibration']['MinDistanceX_m'])
        self.PARM_ZRP['MaxDistanceY_m'] = float(config_param['ZRPCalibration']['MaxDistanceY_m'])
        self.PARM_ZRP['MinDistanceY_m'] = float(config_param['ZRPCalibration']['MinDistanceY_m'])
        self.PARM_ZRP['MaxDistanceZ_m'] = float(config_param['ZRPCalibration']['MaxDistanceZ_m'])
        self.PARM_ZRP['MinDistanceZ_m'] = float(config_param['ZRPCalibration']['MinDistanceZ_m'])

        # Data Validation
        self.PARM_DV['MaximumIteration'] = int(config_param['Validation']['MaximumIteration'])
        self.PARM_DV['Tolerance'] = float(config_param['Validation']['Tolerance'])
        self.PARM_DV['OutlierDistance_m'] = float(config_param['Validation']['OutlierDistance_m'])
        self.PARM_DV['FilterHeadingThreshold'] = float(config_param['Validation']['FilterHeadingThreshold'])
        self.PARM_DV['FilterDistanceThreshold'] = float(config_param['Validation']['FilterDistanceThreshold'])
        self.PARM_DV['MaxThresholdZ_m'] = float(config_param['Validation']['MaxThresholdZ_m'])
        self.PARM_DV['MinThresholdZ_m'] = float(config_param['Validation']['MinThresholdZ_m'])

        # Handeye
        self.PARM_HE['MaximumIteration'] = int(config_param['Handeye']['MaximumIteration'])
        self.PARM_HE['Tolerance'] = float(config_param['Handeye']['Tolerance'])
        self.PARM_HE['OutlierDistance_m'] = float(config_param['Handeye']['OutlierDistance_m'])
        self.PARM_HE['FilterHeadingThreshold'] = float(config_param['Handeye']['FilterHeadingThreshold'])
        self.PARM_HE['FilterDistanceThreshold'] = float(config_param['Handeye']['FilterDistanceThreshold'])
        self.PARM_HE['MaxThresholdZ_m'] = float(config_param['Handeye']['MaxThresholdZ_m'])
        self.PARM_HE['MinThresholdZ_m'] = float(config_param['Handeye']['MinThresholdZ_m'])

        # Single Optimization
        self.PARM_SO['PointSamplingRatio'] = float(config_param['SingleOptimization']['PointSamplingRatio'])
        self.PARM_SO['NumPointsPlaneModeling'] = int(config_param['SingleOptimization']['NumPointsPlaneModeling'])
        self.PARM_SO['OutlierDistance_m'] = float(config_param['SingleOptimization']['OutlierDistance_m'])
        self.PARM_SO['MaxThresholdZ_m'] = float(config_param['SingleOptimization']['MaxThresholdZ_m'])
        self.PARM_SO['MinThresholdZ_m'] = float(config_param['SingleOptimization']['MinThresholdZ_m'])

        # Multi Optimization
        self.PARM_MO['PointSamplingRatio'] = float(config_param['MultiOptimization']['PointSamplingRatio'])
        self.PARM_MO['NumPointsPlaneModeling'] = int(config_param['MultiOptimization']['NumPointsPlaneModeling'])
        self.PARM_MO['OutlierDistance_m'] = float(config_param['MultiOptimization']['OutlierDistance_m'])
        self.PARM_MO['MaxThresholdZ_m'] = float(config_param['MultiOptimization']['MaxThresholdZ_m'])
        self.PARM_MO['MinThresholdZ_m'] = float(config_param['MultiOptimization']['MinThresholdZ_m'])

        # Evaluation
        self.PARM_EV['SamplingInterval'] = int(config_param['Evaluation']['SamplingInterval'])
        self.PARM_EV['DistanceInterval'] = float(config_param['Evaluation']['DistanceInterval'])
        self.PARM_EV['VehicleSpeedThreshold'] = float(config_param['Evaluation']['VehicleSpeedThreshold'])
        self.PARM_EV['MaxThresholdZ_m'] = float(config_param['Evaluation']['MaxThresholdZ_m'])
        self.PARM_EV['MinThresholdZ_m'] = float(config_param['Evaluation']['MinThresholdZ_m'])
        self.PARM_EV['VoxelGridSize_m'] = float(config_param['Evaluation']['VoxelGridSize_m'])

        # Path
        self.PATH['Configuration'] = config_param['Path']['Configuration']
        self.PATH['LoggingFile'] = config_param['Path']['LoggingFile']
        self.PATH['Image'] = config_param['Path']['Image']
        self.PATH['VehicleMesh'] = config_param['Path']['VehicleMesh']
        self.PATH['LidarMesh'] = config_param['Path']['LidarMesh']
        self.PATH['GridMesh'] = config_param['Path']['GridMesh']

        # copy parm to prev parm
        self.PREV_PARMS = copy.deepcopy(self.PARMS)

        # Vehicle info
        config_param.read(self.vehicle_info_file)
        self.VEHICLE_INFO['Evoque'] = [float(config_param['Evoque']['Length']), float(config_param['Evoque']['Width']), float(config_param['Evoque']['Height']),
                                       str(config_param['Evoque']['Front']), str(config_param['Evoque']['Top'])]
        self.VEHICLE_INFO['Sonata'] = [float(config_param['Sonata']['Length']), float(config_param['Sonata']['Width']), float(config_param['Sonata']['Height']),
                                       str(config_param['Sonata']['Front']), str(config_param['Sonata']['Top'])]

        print('Initialize configuration parameter')

    def WriteDefaultFile(self):
        config_file = self.path + '/' + self.configuration_path + 'default.ini'
        self.configuration_file = config_file

        f = open(config_file, 'w', encoding=None)
        f.write('[LIDAR]\n')
        f.write('SingleSensor = 0\n')
        f.write('PrincipalSensor = 0\n')
        f.write('CheckedSensorList = 0\n')
        f.write('SensorList = 0\n')
        f.write('\n')
        f.write('[PointCloud]\n')
        f.write('MinThresholdDist_m = 1.0\n')
        f.write('MaxThresholdDist_m = 50.0\n')
        f.write('MinThresholdX_m = -1000.0\n')
        f.write('MaxThresholdX_m = 1000.0\n')
        f.write('MinThresholdY_m = -1000.0\n')
        f.write('MaxThresholdY_m = 1000.0\n')
        f.write('MinThresholdZ_m = -1000.0\n')
        f.write('MaxThresholdZ_m = 1000.0\n')
        f.write('\n')
        f.write('[Import]\n')
        f.write('SamplingInterval = 1\n')
        f.write('VehicleSpeedThreshold = 1.0\n')
        f.write('\n')
        f.write('[ZRPCalibration]\n')
        f.write('MaxDistanceX_m = 2.0\n')
        f.write('MinDistanceX_m = -2.0\n')
        f.write('MaxDistanceY_m = 2.0\n')
        f.write('MinDistanceY_m = -2.0\n')
        f.write('MaxDistanceZ_m = 2.0\n')
        f.write('MinDistanceZ_m = -2.0\n')
        f.write('[Validation]\n')
        f.write('MaximumIteration = 100\n')
        f.write('Tolerance = 0.0000001\n')
        f.write('OutlierDistance_m = 0.5\n')
        f.write('FilterHeadingThreshold = 100.0\n')
        f.write('FilterDistanceThreshold = 100.0\n')
        f.write('MaxThresholdZ_m = 1000.0\n')
        f.write('MinThresholdZ_m = -1000.0\n')
        f.write('\n')
        f.write('[Handeye]\n')
        f.write('MaximumIteration = 100\n')
        f.write('Tolerance = 0.0000001\n')
        f.write('OutlierDistance_m = 0.5\n')
        f.write('FilterHeadingThreshold = 0.05\n')
        f.write('FilterDistanceThreshold = 0.05\n')
        f.write('MaxThresholdZ_m = 1000.0\n')
        f.write('MinThresholdZ_m = -1000.0\n')
        f.write('\n')
        f.write('[SingleOptimization]\n')
        f.write('PointSamplingRatio = 0.2\n')
        f.write('NumPointsPlaneModeling = 10\n')
        f.write('OutlierDistance_m = 5.\n')
        f.write('MaxThresholdZ_m = 1000.0\n')
        f.write('MinThresholdZ_m = -1000.0\n')
        f.write('\n')
        f.write('[MultiOptimization]\n')
        f.write('PointSamplingRatio = 0.2\n')
        f.write('NumPointsPlaneModeling = 10\n')
        f.write('OutlierDistance_m = 5.\n')
        f.write('MaxThresholdZ_m = 1000.0\n')
        f.write('MinThresholdZ_m = -1000.0\n')
        f.write('\n')
        f.write('[Evaluation]\n')
        f.write('SamplingInterval = 1\n')
        f.write('DistanceInterval = 0\n')
        f.write('VehicleSpeedThreshold = 1.0\n')
        f.write('MaxThresholdZ_m = 1000.0\n')
        f.write('MinThresholdZ_m = -1000.0\n')
        f.write('VoxelGridSize_m = 0.1\n')
        f.write('\n')
        f.write('[Path]\n')
        f.write('Configuration = ' + self.path + '/' + self.configuration_path + '\n')
        f.write('LoggingFile = ' + self.path + '/' + self.logging_data_path + '\n')
        f.write('Image = ' + self.path + '/' + self.image_path + '\n')
        f.write('VehicleMesh = ' + self.path + '/' + self.vehicle_mesh_path + '\n')
        f.write('LidarMesh = ' + self.path + '/' + self.lidar_mesh_path + '\n')
        f.write('GridMesh = ' + self.path + '/' + self.grid_mesh_path + '\n')
        f.close()

        print('Write default configuration parameter in ' + config_file)

    def WriteVehicleInfoFile(self):
        vehicle_info_file = self.path + '/' + self.configuration_path + 'vehicle_info.ini'
        self.vehicle_info_file = vehicle_info_file

        f = open(vehicle_info_file, 'w', encoding=None)
        f.write('[Evoque]\n')
        f.write('Length = 4.371\n')
        f.write('Width = 1.904\n')
        f.write('Height = 1.649\n')
        f.write('Front = Z\n')
        f.write('Top = Y\n')
        f.write('\n')
        f.write('[Sonata]\n')
        f.write('Length = 4.9\n')
        f.write('Width = 1.86\n')
        f.write('Height = 1.445\n')
        f.write('Front = Y\n')
        f.write('Top = Z\n')

        f.close()

    def GetSensorList(self, key):
        lst = ''
        for idxSensor in self.PARM_LIDAR[key]:
            if lst == '':
                lst = str(idxSensor)
            else:
                lst = lst + ' ' + str(idxSensor)

        return lst

    def HasSaveFile(self):
        words = self.configuration_file.split('/')
        if words[-1] != 'default.ini':
            return True
        else:
            return False

    def IsParmChanged(self):
        self.value_changed = False
        # for i, parm in enumerate(self.PARMS):
        for i, parm in enumerate(self.PARMS):
            for key in parm:
                if self.PREV_PARMS[i][key] != parm[key]:
                    self.value_changed = True
                    return True
        return False

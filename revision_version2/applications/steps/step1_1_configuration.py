# -*- coding: utf-8 -*-
"""
@author: chansoo7857@gmail.com
@date: 2018-07-12
@version: 0.0.1
"""

# Basic modules in Anaconda
import os
import configparser

class Configuration:
    path = os.getcwd().replace('\\', '/') # path is gui_tool.exe directory
    configuration_path = 'common/configuration/'
    logging_data_path = 'common/logging_data/'
    image_path = 'common/image/'
    vehicle_mesh_path = 'common/meshes/vehicles/'

    def __init__(self):
        # Path and file
        self.configuration_file = ''
        self.PARM_LIDAR = {}
        self.PARM_PC = {}
        self.PARM_IM = {}
        self.PARM_DV = {}
        self.PARM_HE = {}
        self.PARM_SO = {}
        self.PARM_MO = {}
        self.PARM_EV = {}
        self.PATH = {}
        self.VEHICLE_INFO = {}
        self.CalibrationParam = {}

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

        # Import
        self.PARM_IM['SamplingInterval'] = int(config_param['Import']['SamplingInterval'])
        self.PARM_IM['VehicleSpeedThreshold'] = float(config_param['Import']['VehicleSpeedThreshold'])

        # Data Validation
        self.PARM_DV['MaximumIteration'] = int(config_param['Validation']['MaximumIteration'])
        self.PARM_DV['Tolerance'] = float(config_param['Validation']['Tolerance'])
        self.PARM_DV['OutlierDistance_m'] = float(config_param['Validation']['OutlierDistance_m'])
        self.PARM_DV['FilterHeadingThreshold'] = float(config_param['Validation']['FilterHeadingThreshold'])
        self.PARM_DV['FilterDistanceThreshold'] = float(config_param['Validation']['FilterDistanceThreshold'])

        # Handeye
        self.PARM_HE['MaximumIteration'] = int(config_param['Handeye']['MaximumIteration'])
        self.PARM_HE['Tolerance'] = float(config_param['Handeye']['Tolerance'])
        self.PARM_HE['OutlierDistance_m'] = float(config_param['Handeye']['OutlierDistance_m'])
        self.PARM_HE['FilterHeadingThreshold'] = float(config_param['Handeye']['FilterHeadingThreshold'])
        self.PARM_HE['FilterDistanceThreshold'] = float(config_param['Handeye']['FilterDistanceThreshold'])

        # Single Optimization
        self.PARM_SO['PointSamplingRatio'] = float(config_param['SingleOptimization']['PointSamplingRatio'])
        self.PARM_SO['NumPointsPlaneModeling'] = int(config_param['SingleOptimization']['NumPointsPlaneModeling'])
        self.PARM_SO['OutlierDistance_m'] = float(config_param['SingleOptimization']['OutlierDistance_m'])

        # Multi Optimization
        self.PARM_MO['PointSamplingRatio'] = float(config_param['MultiOptimization']['PointSamplingRatio'])
        self.PARM_MO['NumPointsPlaneModeling'] = int(config_param['MultiOptimization']['NumPointsPlaneModeling'])
        self.PARM_MO['OutlierDistance_m'] = float(config_param['MultiOptimization']['OutlierDistance_m'])

        # Evaluation
        self.PARM_EV['SamplingInterval'] = int(config_param['Evaluation']['SamplingInterval'])
        self.PARM_EV['VehicleSpeedThreshold'] = float(config_param['Evaluation']['VehicleSpeedThreshold'])

        # Path
        self.PATH['Configuration'] = config_param['Path']['Configuration']
        self.PATH['LoggingFile'] = config_param['Path']['LoggingFile']
        self.PATH['Image'] = config_param['Path']['Image']
        self.PATH['VehicleMesh'] = config_param['Path']['VehicleMesh']

        # Vehicle info
        config_param.read(self.vehicle_info_file)
        self.VEHICLE_INFO['evoque_old'] = [float(config_param['evoque_old']['Length']), float(config_param['evoque_old']['Width']),
                                        float(config_param['evoque_old']['Height']), float(config_param['evoque_old']['XRotation']),
                                        float(config_param['evoque_old']['YRotation']), float(config_param['evoque_old']['ZRotation'])]

        self.VEHICLE_INFO['lidar'] = [float(config_param['lidar']['Length']), float(config_param['lidar']['Width']),
                                        float(config_param['lidar']['Height']), float(config_param['lidar']['XRotation']),
                                        float(config_param['lidar']['YRotation']), float(config_param['lidar']['ZRotation'])]

        print('Initialize configuration parameter')

    def WriteDefaultFile(self):
        config_file = self.path + '/' + self.configuration_path + 'default.ini'
        self.WriteDefaultFileBase(config_file)
        self.configuration_file = config_file

    def WriteDefaultFileBase(self, file):
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
        f.write('[Import]\n')
        f.write('SamplingInterval = 1\n')
        f.write('VehicleSpeedThreshold = 1.0\n')
        f.write('\n')
        f.write('[Validation]\n')
        f.write('MaximumIteration = 100\n')
        f.write('Tolerance = 0.0000001\n')
        f.write('OutlierDistance_m = 0.5\n')
        f.write('FilterHeadingThreshold = 100.0\n')
        f.write('FilterDistanceThreshold = 100.0\n')
        f.write('\n')
        f.write('[Handeye]\n')
        f.write('MaximumIteration = 100\n')
        f.write('Tolerance = 0.0000001\n')
        f.write('OutlierDistance_m = 0.5\n')
        f.write('FilterHeadingThreshold = 0.05\n')
        f.write('FilterDistanceThreshold = 0.05\n')
        f.write('\n')
        f.write('[SingleOptimization]\n')
        f.write('PointSamplingRatio = 0.2\n')
        f.write('NumPointsPlaneModeling = 10\n')
        f.write('OutlierDistance_m = 5.\n')
        f.write('\n')
        f.write('[MultiOptimization]\n')
        f.write('PointSamplingRatio = 0.2\n')
        f.write('NumPointsPlaneModeling = 10\n')
        f.write('OutlierDistance_m = 5.\n')
        f.write('\n')
        f.write('[Evaluation]\n')
        f.write('SamplingInterval = 1\n')
        f.write('VehicleSpeedThreshold = 1.0\n')
        f.write('\n')
        f.write('[Path]\n')
        f.write('Configuration = ' + self.path + '/' + self.configuration_path + '\n')
        f.write('LoggingFile = ' + self.path + '/' + self.logging_data_path + '\n')
        f.write('Image = ' + self.path + '/' + self.image_path + '\n')
        f.write('VehicleMesh = ' + self.path + '/' + self.vehicle_mesh_path + '\n')
        f.close()

        print('Write default configuration parameter in ' + file)

    def WriteVehicleInfoFile(self):
        vehicle_info_file = self.path + '/' + self.configuration_path + 'vehicle_info.ini'
        self.vehicle_info_file = vehicle_info_file

        f = open(vehicle_info_file, 'w', encoding=None)
        f.write('[evoque_old]\n')
        f.write('Length = 4371\n')
        f.write('Width = 1904\n')
        f.write('Height = 1649\n')
        f.write('XRotation = 90\n')
        f.write('YRotation = 90\n')
        f.write('ZRotation = 0\n')
        f.write('\n')
        f.write('[lidar]\n')
        f.write('Length = 20\n')
        f.write('Width = 20\n')
        f.write('Height = 20\n')
        f.write('XRotation = 90\n')
        f.write('YRotation = 90\n')
        f.write('ZRotation = 0\n')

        f.close()

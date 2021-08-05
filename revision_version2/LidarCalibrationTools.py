# -*- coding: utf-8 -*-
"""
@author kimkys768@gmail.com
@author yondoo20@gmail.com
@author akila.ben-charrada.ext@valeo.com
@version 0.0.2
@date 22-09-2020
"""
import fileinput
import math
import sys
import copy
#import pdb
from instance_id import *

import matplotlib.pyplot as plt
import element
from applications.utils import get_result
from applications.steps import step1_1_configuration
from applications.steps import step1_2_import_data
from applications.steps import step2_ZRP_calibration
from applications.steps import step3_1_XYYaw_data_validation
from applications.steps import step3_2_XYYaw_handeye
from applications.steps import step3_3_XYYaw_unsupervised
from applications.steps import step4_evaluation
from hmi.visualization import vtk_lidar_calib
from widget.QButton import *
from widget import QThread
from widget.QScrollarea import *
from PyQt5.QtGui import *

import numpy as np
from PIL import Image
import tkinter as Tk
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

## button size
CONST_NEXT_BTN_HEIGHT = 80
CONST_SCROLL_BOX_HEIGHT = 120

class ConfigurationTab(QWidget):
    def __init__(self, form_widget):
        super().__init__()
        self.is_lidar_num_changed = False
        self.form_widget = form_widget
        self.added_stl_dict = {}
        self.added_file_list = []

        self.InitUi()

        self.stl_path_dict = {}
        self.InitPath()

    def InitUi(self):
        main_vbox = QVBoxLayout()

        config_widget = QWidget()
        config_widget.setLayout(self.SetConfiguration_Layout())
        main_vbox.addWidget(config_widget)

        self.next_btn = QPushButton('Next step')
        self.next_btn.clicked.connect(self.NextBtn)
        main_vbox.addWidget(self.next_btn)

        self.setLayout(main_vbox)

    ## Layout
    def SetConfiguration_Layout(self):
        vbox = QVBoxLayout()

        hbox1 = QHBoxLayout()
        hbox1.addWidget(self.SetConfiguration_LidarConfiguration_Groupbox(), 50)
        hbox1.addWidget(self.SetConfiguration_PointCloudConfiguration_Groupbox(), 50)
        vbox.addLayout(hbox1, 40)

        hbox2 = QHBoxLayout()
        self.image_display_widget = element.ImageDisplay(self.form_widget.config.PATH['Image'])
        hbox2.addWidget(self.image_display_widget, 50)
        hbox2.addWidget(self.SetConfiguration_VehicleConfiguration_Groupbox(), 50)
        vbox.addLayout(hbox2, 60)

        return vbox

    ## Groupbox

    def SetConfiguration_LidarConfiguration_Groupbox(self):
        self.lidar_config_groupbox = QGroupBox('LiDAR Configuration')
        vbox = QVBoxLayout()

        self.lidar_num_label_layout = element.SpinBoxLabelLayout('LiDAR Num', self.form_widget)
        vbox.addLayout(self.lidar_num_label_layout)

        self.select_using_sensor_list_layout = element.CheckBoxListLayout(CONST_SELECT_USING_SENSOR_LIST, self.form_widget, 'Select Using Sensor List')
        vbox.addLayout(self.select_using_sensor_list_layout)

        self.lidar_config_groupbox.setLayout(vbox)
        return self.lidar_config_groupbox

    def SetConfiguration_PointCloudConfiguration_Groupbox(self):
        groupbox = QGroupBox('PointCloud Configuration')
        vbox = QVBoxLayout()

        self.minimum_threshold_distance_layout = element.DoubleSpinBoxLabelLayout(CONST_CONFIG_MINIMUM_THRESHOLD_DISTANCE, "Minimum Threshold Distance [m]", self.form_widget)
        vbox.addLayout(self.minimum_threshold_distance_layout)

        self.maximum_threshold_istance_layout = element.DoubleSpinBoxLabelLayout(CONST_CONFIG_MAXIMUM_THRESHOLD_DISTANCE, "Maximum Threshold Distance [m]", self.form_widget)
        vbox.addLayout(self.maximum_threshold_istance_layout)

        self.minimum_threshold_layout_x = element.DoubleSpinBoxLabelLayout(CONST_CONFIG_MINIMUM_THRESHOLD_X, "Minimum Threshold X [m]", self.form_widget)
        vbox.addLayout(self.minimum_threshold_layout_x)

        self.maximum_threshold_layout_x = element.DoubleSpinBoxLabelLayout(CONST_CONFIG_MAXIMUM_THRESHOLD_X, "Maximum Threshold X [m]", self.form_widget)
        vbox.addLayout(self.maximum_threshold_layout_x)

        self.minimum_threshold_layout_y = element.DoubleSpinBoxLabelLayout(CONST_CONFIG_MINIMUM_THRESHOLD_Y, "Minimum Threshold Y [m]", self.form_widget)
        vbox.addLayout(self.minimum_threshold_layout_y)

        self.maximum_threshold_layout_y = element.DoubleSpinBoxLabelLayout(CONST_CONFIG_MAXIMUM_THRESHOLD_Y, "Maximum Threshold Y [m]", self.form_widget)
        vbox.addLayout(self.maximum_threshold_layout_y)

        self.minimum_threshold_layout_z = element.DoubleSpinBoxLabelLayout(CONST_CONFIG_MINIMUM_THRESHOLD_Z, "Minimum Threshold Z [m]", self.form_widget)
        vbox.addLayout(self.minimum_threshold_layout_z)

        self.maximum_threshold_layout_z = element.DoubleSpinBoxLabelLayout(CONST_CONFIG_MAXIMUM_THRESHOLD_Z, "Maximum Threshold Z [m]", self.form_widget)
        vbox.addLayout(self.maximum_threshold_layout_z)

        groupbox.setLayout(vbox)
        return groupbox

    def SetConfiguration_VehicleConfiguration_Groupbox(self):
        groupbox = QGroupBox('Vehicle Configuration')
        vbox = QVBoxLayout()

        # combo box
        self.cb = QComboBox()
        qdir = QDir(self.form_widget.config.PATH['VehicleMesh'])
        self.cb.addItems(qdir.entryList(QDir.Files))
        self.cb.activated[str].connect(self.SelectStl)

        pal = self.cb.palette()
        pal.setColor(QPalette.Button, QColor(255,255,255))
        self.cb.setPalette(pal)
        vbox.addWidget(self.cb)

        # vehicle info box
        vehicle_info_box = QGroupBox()
        vehicle_info_box.setFlat(True)

        id_to_strs1 = {CONST_CONFIG_LENGTH: 'Length [m]', CONST_CONFIG_WIDTH: 'Width [m]', CONST_CONFIG_HEIGHT: 'Hieght [m]'}
        id_to_strs2 = {CONST_CONFIG_FRONT: 'STL Front Axis ', CONST_CONFIG_TOP: 'STL Top Axis'}
        self.vehicle_info_layout = element.EditLabelWithButtonLayout(CONST_CONFIG_VEHICLE_INFO, id_to_strs1, id_to_strs2, self.form_widget)
        vehicle_info_box.setLayout(self.vehicle_info_layout)

        axis = ['X', 'Y', 'Z']
        self.vehicle_info_layout.cb_list[0].cb.addItems(axis)
        self.vehicle_info_layout.cb_list[1].cb.addItems(axis)
        vbox.addWidget(vehicle_info_box)

        # vtk vehicle
        self.vtkWidget = QVTKRenderWindowInteractor()
        vbox.addWidget(self.vtkWidget)
        self.VTKInit(self.cb.currentText())

        hbox = QHBoxLayout()

        btn = QPushButton('xy plane')
        btn.clicked.connect(lambda: self.form_widget.XY(self.ren, self.renWin))
        hbox.addWidget(btn)

        btn = QPushButton('yz plane')
        btn.clicked.connect(lambda: self.form_widget.YZ(self.ren, self.renWin))
        hbox.addWidget(btn)

        btn = QPushButton('zx plane')
        btn.clicked.connect(lambda: self.form_widget.ZX(self.ren, self.renWin))
        hbox.addWidget(btn)
        vbox.addLayout(hbox)

        groupbox.setLayout(vbox)
        return groupbox

    ## Callback Function

    def NextBtn(self):
        self.form_widget.tabs.setTabEnabled(CONST_IMPORTDATA_TAB, True)
        self.form_widget.tabs.setCurrentIndex(CONST_IMPORTDATA_TAB)

    def SelectStl(self, text):
        if text != 'Add new 3d model':
            words = text.split('.')

            if self.form_widget.config.VEHICLE_INFO.get(words[0]) is None:
                self.vehicle_info_layout.BtnEnable()
                self.vehicle_info_layout.Clear()
                self.form_widget.ErrorPopUp('Please input vehicle information and push the import button')
            else:
                self.vehicle_info_layout.Clear(self.form_widget.config.VEHICLE_INFO[words[0]])
                if self.form_widget.config_tab.added_stl_dict.get(words[0]) is not None:
                    self.vehicle_info_layout.BtnEnable()
                else:
                    self.vehicle_info_layout.BtnDisable()

                self.VTKInit(text)
        else:
            self.vehicle_info_layout.BtnEnable()
            self.vehicle_info_layout.Clear()

            widget = QWidget()
            fname = QFileDialog.getOpenFileName(widget, 'Open file', self.form_widget.config.PATH['VehicleMesh'],
                                                "Configuration file (*.stl)")  # exist ini file path

            if fname[0]:
                qdir = QDir(self.form_widget.config.PATH['VehicleMesh'])
                files = qdir.entryList(QDir.Files)

                words = fname[0].split('/')
                adding_file_name = words[-1]
                del words[-1]

                if adding_file_name in files:
                    self.form_widget.ErrorPopUp("Already selected, {}".format(adding_file_name))
                    self.vehicle_info_layout.BtnDisable()
                    return

                self.cb.clear()
                for added_file in self.added_file_list:
                    files.append(added_file)
                select_idx = len(files)
                files.append(adding_file_name)
                self.cb.addItems(files)
                self.cb.addItem('Add new 3d model')
                self.cb.setCurrentIndex(select_idx)

                adding_file_path = ''
                for word in words:
                    adding_file_path += (word + '/')
                self.stl_path_dict[adding_file_name] = adding_file_path
                self.added_file_list.append(adding_file_name)

                words = adding_file_name.split('.')
                adding_stl = words[0]
                self.added_stl_dict[adding_stl] = []

    def VTKInit(self, text):
        vtk.vtkObject.GlobalWarningDisplayOff()

        stl_path = self.form_widget.config.PATH['VehicleMesh']
        stl_path = stl_path + text
        vtk_lidar_calib.SetVehicleStlPath(stl_path)

        words = text.split('.')
        vehicle_info = self.form_widget.config.VEHICLE_INFO[words[0]]
        vtk_lidar_calib.SetVehicleInfo(vehicle_info)

        colors = vtk.vtkNamedColors()
        self.ren = vtk.vtkRenderer()

        self.renWin = self.vtkWidget.GetRenderWindow()
        self.renWin.AddRenderer(self.ren)

        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        self.iren.SetRenderWindow(self.renWin)

        # Assign actor to the renderer
        self.ren.AddActor(vtk_lidar_calib.GetVehicleActor())
        self.ren.AddActor(vtk_lidar_calib.GetGridActor())
        self.widget = vtk.vtkOrientationMarkerWidget()
        vtk_lidar_calib.GetAxis(self.iren, self.widget)

        self.ren.SetBackground(colors.GetColor3d("white"))

        self.iren.Initialize()
        self.ren.ResetCamera()
        self.renWin.Render()

        self.iren.Start()

class ImportDataTab(QWidget):
    def __init__(self, form_widget):
        super().__init__()
        self.form_widget = form_widget

        self.initUi()

    def initUi(self):
        main_vbox = QVBoxLayout()

        main_vbox.addWidget(self.Main_LoggingData_Groupbox())

        hbox = QHBoxLayout()
        hbox.addWidget(self.Main_GnssInit_Groupbox(), 35)
        hbox.addWidget(self.Main_Empty_Groupbox(), 65)
        main_vbox.addLayout(hbox)
        main_vbox.addStretch(round(0.5))

        hbox = QHBoxLayout()
        hbox.addWidget(self.Main_Set_Configuration_Groupbox(), 25)
        hbox.addWidget(self.Main_LimitTime_Groupbox(), 75)
        main_vbox.addLayout(hbox)

        self.next_btn = QPushButton('Next step')
        self.next_btn.clicked.connect(self.NextBtn)
        main_vbox.addWidget(self.next_btn)

        self.setLayout(main_vbox)

    ## Groupbox

    def Main_LoggingData_Groupbox(self):
        groupbox = QGroupBox('Import Logging Data')
        self.logging_vbox = QVBoxLayout()

        self.logging_file_path_layout = element.FileInputWithCheckBtnLayout('Select Logging File Path', self.form_widget)
        self.logging_vbox.addLayout(self.logging_file_path_layout)

        label = QLabel('[ csv File List ]')
        self.logging_vbox.addWidget(label)

        self.gnss_scroll_box = ScrollAreaH()
        self.logging_vbox.addWidget(self.gnss_scroll_box)

        label = QLabel('[ bin File List ]')
        self.logging_vbox.addWidget(label)

        self.lidar_scroll_box = ScrollAreaH()
        self.logging_vbox.addWidget(self.lidar_scroll_box)

        groupbox.setLayout(self.logging_vbox)
        return groupbox

    def Main_GnssInit_Groupbox(self):
        groupbox = QGroupBox('Set Initial Pose in ENU Coordinate')
        vbox = QVBoxLayout()

        hbox = QHBoxLayout()
        label = QLabel()
        hbox.addWidget(label, 50)
        label = QLabel('East [m]')
        hbox.addWidget(label, 16)
        label = QLabel('North [m]')
        hbox.addWidget(label, 17)
        label = QLabel('Heading [deg]')
        hbox.addWidget(label, 17)
        vbox.addLayout(hbox)

        self.scroll_box = ScrollAreaV()
        vbox.addWidget(self.scroll_box)

        groupbox.setLayout(vbox)
        return groupbox

    def Main_Empty_Groupbox(self):
        groupbox = QGroupBox()

        return groupbox

    def Main_Set_Configuration_Groupbox(self):
        groupbox = QGroupBox('Set Configuration')
        vbox = QVBoxLayout()

        self.sampling_interval_layout = element.SpinBoxLabelLayout('Sampling Interval [Count]', self.form_widget)
        vbox.addLayout(self.sampling_interval_layout)

        self.time_speed_threshold_layout = element.DoubleSpinBoxLabelLayout(CONST_IMPORT_VEHICLE_MINIMUM_SPEED, 'Vehicle Minimum Speed [km/h]', self.form_widget)
        vbox.addLayout(self.time_speed_threshold_layout)

        groupbox.setLayout(vbox)
        return groupbox

    def Main_LimitTime_Groupbox(self):
        groupbox = QGroupBox('Limit Time Data')
        vbox = QVBoxLayout()

        self.limit_time_layout = element.SlideLabelLayouts(self.form_widget, '[ Limit Time ]')
        vbox.addLayout(self.limit_time_layout)

        groupbox.setLayout(vbox)
        return groupbox

    ## Callback func

    def NextBtn(self):
        if self.form_widget.importing.is_complete == False:
            self.form_widget.ErrorPopUp('Please import logging file path')
        else:
            self.form_widget.tabs.setTabEnabled(CONST_ZROLLPITCH_TAB, True)
            self.form_widget.tabs.setTabEnabled(CONST_VALIDATION_TAB, True)
            self.form_widget.tabs.setTabEnabled(CONST_HANDEYE_TAB, True)
            self.form_widget.tabs.setCurrentIndex(CONST_ZROLLPITCH_TAB)

    def IterationPercentage(self, percentage_dict):
        idxSensor = list(percentage_dict.keys())[0]
        percentage = list(percentage_dict.values())[0]
        text = 'XYZRGB {}'.format(idxSensor) + ' ' + str(int(percentage)) + '%'

        self.logging_file_path_layout.lidar_buttons[idxSensor].setText(text)

    def EndImport(self):
        parsed_pandas_dataframe = self.form_widget.importing.text_pointcloud
        self.logging_file_path_layout.parsed_bin = parsed_pandas_dataframe.to_string()

        default_start_time = self.form_widget.importing.DefaultStartTime
        default_end_time = self.form_widget.importing.DefaultEndTime

        # Config
        ## initialize config
        self.form_widget.config.PARM_ZRP_DICT.clear()

        for idxSensor in self.form_widget.config.PARM_LIDAR['CheckedSensorList']:
            self.form_widget.config.PARM_ZRP_DICT[idxSensor] = {}
            self.form_widget.config.PARM_ZRP_DICT[idxSensor]['MaxDistanceX_m'] = self.form_widget.config.PARM_ZRP['MaxDistanceX_m']
            self.form_widget.config.PARM_ZRP_DICT[idxSensor]['MinDistanceX_m'] = self.form_widget.config.PARM_ZRP['MinDistanceX_m']
            self.form_widget.config.PARM_ZRP_DICT[idxSensor]['MaxDistanceY_m'] = self.form_widget.config.PARM_ZRP['MaxDistanceY_m']
            self.form_widget.config.PARM_ZRP_DICT[idxSensor]['MinDistanceY_m'] = self.form_widget.config.PARM_ZRP['MinDistanceY_m']
            self.form_widget.config.PARM_ZRP_DICT[idxSensor]['MaxDistanceZ_m'] = self.form_widget.config.PARM_ZRP['MaxDistanceZ_m']
            self.form_widget.config.PARM_ZRP_DICT[idxSensor]['MinDistanceZ_m'] = self.form_widget.config.PARM_ZRP['MinDistanceZ_m']

            self.form_widget.zrollpitch_tab.maximum_x_layout.double_spin_box.setValue(
                self.form_widget.config.PARM_ZRP_DICT[self.form_widget.config.PARM_LIDAR['CheckedSensorList'][0]]['MaxDistanceX_m'])
            self.form_widget.zrollpitch_tab.minimum_x_layout.double_spin_box.setValue(
                self.form_widget.config.PARM_ZRP_DICT[self.form_widget.config.PARM_LIDAR['CheckedSensorList'][0]]['MinDistanceX_m'])
            self.form_widget.zrollpitch_tab.maximum_y_layout.double_spin_box.setValue(
                self.form_widget.config.PARM_ZRP_DICT[self.form_widget.config.PARM_LIDAR['CheckedSensorList'][0]]['MaxDistanceY_m'])
            self.form_widget.zrollpitch_tab.minimum_y_layout.double_spin_box.setValue(
                self.form_widget.config.PARM_ZRP_DICT[self.form_widget.config.PARM_LIDAR['CheckedSensorList'][0]]['MinDistanceY_m'])
            self.form_widget.zrollpitch_tab.maximum_z_layout.double_spin_box.setValue(
                self.form_widget.config.PARM_ZRP_DICT[self.form_widget.config.PARM_LIDAR['CheckedSensorList'][0]]['MaxDistanceZ_m'])
            self.form_widget.zrollpitch_tab.minimum_z_layout.double_spin_box.setValue(
                self.form_widget.config.PARM_ZRP_DICT[self.form_widget.config.PARM_LIDAR['CheckedSensorList'][0]]['MinDistanceZ_m'])

        self.form_widget.tabs.setTabEnabled(CONST_CONFIG_TAB, True)

        # Import tab
        ## Set Configuration
        self.form_widget.RemoveLayout(self.scroll_box.layout)

        if self.form_widget.importing.has_gnss_file and self.form_widget.importing.has_motion_file:
            self.init_gnss_value_layout = element.GnssInitEditLabel('Gnss Initial Pose in ENU Coordinate', self.form_widget)
            self.init_gnss_value_layout.Clear(self.form_widget.importing.df_gnss['east_m'].values[0],
                                              self.form_widget.importing.df_gnss['north_m'].values[0],
                                              self.form_widget.importing.df_gnss['heading'].values[0])
            self.scroll_box.layout.addLayout(self.init_gnss_value_layout)

            self.init_motion_value_layout = element.GnssInitEditLabel('Motion Initial Pose in ENU coordinate    ', self.form_widget)
            self.init_motion_value_layout.Clear(self.form_widget.importing.init[0],
                                                self.form_widget.importing.init[1],
                                                self.form_widget.importing.init[2])
            self.scroll_box.layout.addLayout(self.init_motion_value_layout)

            self.SetUsedData(self.form_widget.datavalidation_tab, 1)
            self.SetUsedData(self.form_widget.handeye_tab, 1)
            self.SetUsedData(self.form_widget.unsupervised_tab, 1)

        elif self.form_widget.importing.has_gnss_file:
            self.init_gnss_value_layout = element.GnssInitEditLabel('Gnss Initial Pose in ENU Coordinate', self.form_widget)
            self.init_gnss_value_layout.Clear(self.form_widget.importing.df_gnss['east_m'].values[0],
                                              self.form_widget.importing.df_gnss['north_m'].values[0],
                                              self.form_widget.importing.df_gnss['heading'].values[0])
            self.scroll_box.layout.addLayout(self.init_gnss_value_layout)

            self.form_widget.datavalidation_tab.radio_label_layout.button_group.button(2).setChecked(True)
            self.form_widget.datavalidation_tab.radio_label_layout.RadioButton()

            self.SetUsedData(self.form_widget.datavalidation_tab, 1)
            self.SetUsedData(self.form_widget.handeye_tab, 1)
            self.SetUsedData(self.form_widget.unsupervised_tab, 1)

            ## Change time_speed_threshold_layout status
            self.time_speed_threshold_layout.double_spin_box.setReadOnly(True)
            self.time_speed_threshold_layout.double_spin_box.setStyleSheet("background-color: #F0F0F0;")
            self.time_speed_threshold_layout.double_spin_box.setButtonSymbols(QAbstractSpinBox.NoButtons)

        elif self.form_widget.importing.has_motion_file:
            self.init_motion_value_layout = element.GnssInitEditLabel('Motion Initial Pose in ENU coordinate    ', self.form_widget)
            self.init_motion_value_layout.Clear(self.form_widget.importing.init[0],
                                                self.form_widget.importing.init[1],
                                                self.form_widget.importing.init[2])
            self.scroll_box.layout.addLayout(self.init_motion_value_layout)

            self.SetUsedData(self.form_widget.datavalidation_tab, 2)
            self.SetUsedData(self.form_widget.handeye_tab, 2)
            self.SetUsedData(self.form_widget.unsupervised_tab, 2)

        ## set slider default time
        self.limit_time_layout.start_time_layout.slider.setMaximum(default_end_time)
        self.limit_time_layout.start_time_layout.slider.setMinimum(default_start_time)
        self.limit_time_layout.end_time_layout.slider.setMaximum(default_end_time)
        self.limit_time_layout.end_time_layout.slider.setMinimum(default_start_time)

        ## set slider and double_spin_box value
        self.limit_time_layout.end_time_layout.slider.setValue(self.limit_time_layout.end_time_layout.slider.maximum())
        self.limit_time_layout.end_time_layout.double_spin_box.setValue(default_end_time)
        self.limit_time_layout.start_time_layout.slider.setValue(self.limit_time_layout.end_time_layout.slider.minimum())
        self.limit_time_layout.start_time_layout.double_spin_box.setValue(default_start_time)

        ## set slider and double_spin_box value start, end time
        self.limit_time_layout.start_time = default_start_time
        self.limit_time_layout.end_time = default_end_time

        # Z, Roll, Pitch tab
        ## Clear
        self.form_widget.zrollpitch_tab.Clear()

        ## set slider default time
        self.form_widget.zrollpitch_tab.limit_time_layout.start_time_layout.slider.setMaximum(default_end_time)
        self.form_widget.zrollpitch_tab.limit_time_layout.start_time_layout.slider.setMinimum(default_start_time)
        self.form_widget.zrollpitch_tab.limit_time_layout.end_time_layout.slider.setMaximum(default_end_time)
        self.form_widget.zrollpitch_tab.limit_time_layout.end_time_layout.slider.setMinimum(default_start_time)

        ## set slider and double_spin_box value
        self.form_widget.zrollpitch_tab.limit_time_layout.end_time_layout.slider.setValue(self.form_widget.zrollpitch_tab.limit_time_layout.end_time_layout.slider.maximum())
        self.form_widget.zrollpitch_tab.limit_time_layout.end_time_layout.double_spin_box.setValue(default_end_time)
        self.form_widget.zrollpitch_tab.limit_time_layout.start_time_layout.slider.setValue(self.form_widget.zrollpitch_tab.limit_time_layout.end_time_layout.slider.minimum())
        self.form_widget.zrollpitch_tab.limit_time_layout.start_time_layout.double_spin_box.setValue(default_start_time)

        ## set slider and double_spin_box value start, end time
        self.form_widget.zrollpitch_tab.limit_time_layout.start_time = default_start_time
        self.form_widget.zrollpitch_tab.limit_time_layout.end_time = default_end_time

        # Data validation tab
        ## Clear
        self.form_widget.datavalidation_tab.Clear()

        # Handeye tab
        ## Clear
        self.form_widget.handeye_tab.Clear()

        # Unsupervised tab
        ## Clear
        self.form_widget.unsupervised_tab.Clear()

        # Evaluation tab
        ## Clear
        self.form_widget.evaluation_tab.Clear()

        ## set slider default time
        self.form_widget.evaluation_tab.limit_time_layout.start_time_layout.slider.setMaximum(default_end_time)
        self.form_widget.evaluation_tab.limit_time_layout.start_time_layout.slider.setMinimum(default_start_time)
        self.form_widget.evaluation_tab.limit_time_layout.end_time_layout.slider.setMaximum(default_end_time)
        self.form_widget.evaluation_tab.limit_time_layout.end_time_layout.slider.setMinimum(default_start_time)

        ## set slider and double_spin_box value
        self.form_widget.evaluation_tab.limit_time_layout.end_time_layout.slider.setValue(self.form_widget.evaluation_tab.limit_time_layout.end_time_layout.slider.maximum())
        self.form_widget.evaluation_tab.limit_time_layout.end_time_layout.double_spin_box.setValue(default_end_time)
        self.form_widget.evaluation_tab.limit_time_layout.start_time_layout.slider.setValue(self.form_widget.evaluation_tab.limit_time_layout.end_time_layout.slider.minimum())
        self.form_widget.evaluation_tab.limit_time_layout.start_time_layout.double_spin_box.setValue(default_start_time)

        ## set slider and double_spin_box value start, end time
        self.form_widget.evaluation_tab.limit_time_layout.start_time = default_start_time
        self.form_widget.evaluation_tab.limit_time_layout.end_time = default_end_time

    ## Util functions
    def SetUsedData(self, tab, selected_btn):
        tab.radio_label_layout.button_group.button(selected_btn).setChecked(True)
        tab.radio_label_layout.RadioButton()

class ZRollPitch_CalibrationTab(QWidget):
    def __init__(self, form_widget):
        super().__init__()
        self.form_widget = form_widget

        self.progress_status = CONST_IDLE
        self.result_labels = {}
        self.calib_result = {}

        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.z_m = 0.0

        self.idxSensor = 0

        self.initUi()

    def initUi(self):
        main_hbox = QHBoxLayout()

        self.config_widget = QWidget()
        self.config_widget.setLayout(self.Configuration_Layout())
        main_hbox.addWidget(self.config_widget, 25)

        self.result_widget = QWidget()
        self.result_widget.setLayout(self.Result_Layout())
        main_hbox.addWidget(self.result_widget, 75)
        self.setLayout(main_hbox)

    ## Layout
    def Configuration_Layout(self):
        self.configuration_vbox = QVBoxLayout()

        self.configuration_vbox.addWidget(self.Configuration_LimitTime_Groupbox(), 75)
        self.configuration_vbox.addWidget(self.Configuration_SetConfiguration_Groupbox())
        self.configuration_vbox.addWidget(self.Configuration_Calibration_Groupbox())

        return self.configuration_vbox

    def Result_Layout(self):
        vbox = QVBoxLayout()

        vbox.addWidget(self.Result_GroundPoints_Groupbox())
        vbox.addWidget(self.Result_Evaluation_Groupbox())

        return vbox

    ## Groupbox
    def Result_GroundPoints_Groupbox(self):
        groupbox = QGroupBox('Ground Points')

        hBox = QHBoxLayout()
        vbox = QVBoxLayout()

        label_2d = QLabel('[2D Ground Points]')
        vbox.addWidget(label_2d)

        self.result_2d_data_pose_fig = plt.figure()
        self.result_2d_data_pose_canvas = FigureCanvas(self.result_2d_data_pose_fig)
        self.result_2d_data_pose_ax = self.result_2d_data_pose_fig.add_subplot(1, 1, 1)
        self.result_2d_data_pose_ax.grid()
        self.result_2d_data_pose_canvas.draw()
        vbox.addWidget(self.result_2d_data_pose_canvas)

        btn1 = QPushButton('View')
        btn1.clicked.connect(self.ViewGroundPoints1)
        vbox.addWidget(btn1)

        hBox.addLayout(vbox)

        vbox2 = QVBoxLayout()

        label_3d = QLabel('[3D Ground Points]')
        vbox2.addWidget(label_3d)

        self.result_3d_data_pose_fig = plt.figure()
        self.result_3d_data_pose_canvas = FigureCanvas(self.result_3d_data_pose_fig)
        self.result_3d_data_pose_ax = self.result_3d_data_pose_fig.add_subplot(111, projection='3d')
        self.result_3d_data_pose_ax.grid()
        self.result_3d_data_pose_canvas.draw()
        vbox2.addWidget(self.result_3d_data_pose_canvas)

        btn2 = QPushButton('View')
        btn2.clicked.connect(self.ViewGroundPoints2)
        vbox2.addWidget(btn2)
        hBox.addLayout(vbox2)

        groupbox.setLayout(hBox)
        return groupbox

    def Result_Evaluation_Groupbox(self):
        groupbox = QGroupBox('Evaluation')

        hBox = QHBoxLayout()
        vbox = QVBoxLayout()

        label_mean_d = QLabel('[Mean Distance between Extracted Plane and Selected Ground Points]')
        vbox.addWidget(label_mean_d)

        self.result_graph1_fig = plt.figure()
        self.result_graph1_canvas = FigureCanvas(self.result_graph1_fig)
        self.result_graph1_ax = self.result_graph1_fig.add_subplot(1, 1, 1)
        self.result_graph1_ax.grid()
        self.result_graph1_canvas.draw()
        vbox.addWidget(self.result_graph1_canvas)

        btn1 = QPushButton('View')
        btn1.clicked.connect(self.ViewMeanDistance)
        # btn.clicked.connect(self.ViewPointCloud)
        vbox.addWidget(btn1)

        hBox.addLayout(vbox)

        vbox2 = QVBoxLayout()

        label_3d = QLabel('[Extracted Ground Slope using Z, Roll, Pitch Parameters]')
        vbox2.addWidget(label_3d)

        self.result_graph2_fig = plt.figure()
        self.result_graph2_canvas = FigureCanvas(self.result_graph2_fig)
        self.result_graph2_ax = self.result_graph2_fig.add_subplot(1, 1, 1)
        self.result_graph2_ax.grid()
        self.result_graph2_canvas.draw()
        vbox2.addWidget(self.result_graph2_canvas)

        btn2 = QPushButton('View')
        btn2.clicked.connect(self.ViewGroundSlope)
        vbox2.addWidget(btn2)
        hBox.addLayout(vbox2)

        groupbox.setLayout(hBox)

        return groupbox

    ## Callback func

    def ViewGroundPoints1(self):
        pass

    def ViewGroundPoints2(self):
        pass
    
    def ViewMeanDistance(self):
        pass
    
    def ViewGroundSlope(self):
        pass

    def Apply(self, display_groundpoint, start_time, end_time, PARM_ZRP, selected_sensor, end_callback):
        if self.form_widget.config_tab.is_lidar_num_changed == True:
            self.form_widget.ErrorPopUp('Please import after changing lidar number')
            return False
        if self.progress_status is not CONST_IDLE:
            return False
        if self.form_widget.importing.PointCloudSensorList.get(selected_sensor) is None:
            imported_pc = ''
            for pc in self.form_widget.importing.PointCloudSensorList:
                imported_pc += (str(pc) + ' ')

            self.form_widget.ErrorPopUp('Import pointcloud {}\nImported pointcloud: {}'.format(selected_sensor, imported_pc))
            return False

        self.progress_status = CONST_PLAY

        self.PlotClear(self.result_2d_data_pose_ax, self.result_2d_data_pose_canvas)
        self.PlotClear(self.result_3d_data_pose_ax, self.result_3d_data_pose_canvas)
        self.text_edit.clear()
        self.form_widget.rpz_thread._status = True
        self.form_widget.rpz_thread.SetFunc(display_groundpoint,
                               [start_time, end_time, PARM_ZRP, selected_sensor])
        try:
            self.form_widget.rpz_thread.end.disconnect()
        except:
            pass

        print('Start z, roll, pitch view ground')
        self.form_widget.rpz_thread.end.connect(end_callback)
        self.form_widget.rpz_thread.start()

    def PlaneExtractRadioButton(self):
        pass

    def OptimizeRadioButton(self):
        pass

    # Util functions

    def PlotClear(self, ax, canvas):
        # remove plots
        try:
            ax.clear()
            canvas.draw()
        except:
            pass

    def Clear(self):
        self.PlotClear(self.result_graph1_ax, self.result_graph1_canvas)
        self.PlotClear(self.result_graph2_ax, self.result_graph2_canvas)
        self.PlotClear(self.result_2d_data_pose_ax, self.result_2d_data_pose_canvas)
        self.PlotClear(self.result_3d_data_pose_ax, self.result_3d_data_pose_canvas)
        self.calib_result.clear()
        self.select_lidar_to_calib_layout.Clear()
        self.text_edit.clear()

    def ErrorCheck(self):
        if self.form_widget.config_tab.is_lidar_num_changed == True:
            self.form_widget.ErrorPopUp('Please import after changing lidar number')
            return False
        if self.progress_status is not CONST_IDLE:
            return False

        if self.form_widget.importing.PointCloudSensorList.get(self.idxSensor) is None:
            imported_pc = ''
            for pc in self.form_widget.importing.PointCloudSensorList:
                imported_pc += (str(pc) + ' ')

            self.form_widget.ErrorPopUp(
                'Import pointcloud {}\nImported pointcloud: {}'.format(self.idxSensor, imported_pc))
            return False

class ZRollPitchTab(ZRollPitch_CalibrationTab):
    def __init__(self, form_widget):
        super().__init__(form_widget)

        self.plane_method = self.button_group1.checkedId()
        self.est_method = self.button_group2.checkedId()

    ## Groupbox
    def Configuration_LimitTime_Groupbox(self):
        groupbox = QGroupBox('Limit Time Data')
        vbox = QVBoxLayout()

        self.limit_time_layout = element.SlideLabelLayouts(self.form_widget, '[ Limit Time ]')
        vbox.addLayout(self.limit_time_layout)

        groupbox.setLayout(vbox)
        return groupbox

    def Configuration_SetConfiguration_Groupbox(self):
        groupbox = QGroupBox('Configuration Info')
        vbox = QVBoxLayout()

        liDAR_configuration_label = QLabel('[ Select LiDAR ]', self)
        vbox.addWidget(liDAR_configuration_label)

        self.select_lidar_to_calib_layout = element.ComboBoxLabelLayout(CONST_ZRP_SELECT_LIDAR_TO_CALIB, 'Select Lidar To Calibration', self.form_widget)
        vbox.addLayout(self.select_lidar_to_calib_layout)

        roi_configuration_label = QLabel('[ ROI Configuration ]', self)
        vbox.addWidget(roi_configuration_label)

        self.maximum_x_layout = element.DoubleSpinBoxLabelLayout(CONST_ZRP_MAXIMUM_X_DISTANCE, 'Maximum X Distance [m]', self.form_widget)
        vbox.addLayout(self.maximum_x_layout)

        self.minimum_x_layout = element.DoubleSpinBoxLabelLayout(CONST_ZRP_MINIMUM_X_DISTANCE, 'Minimum X Distance [m]', self.form_widget)
        vbox.addLayout(self.minimum_x_layout)

        self.maximum_y_layout = element.DoubleSpinBoxLabelLayout(CONST_ZRP_MAXIMUM_Y_DISTANCE, 'Maximum Y Distance [m]', self.form_widget)
        vbox.addLayout(self.maximum_y_layout)

        self.minimum_y_layout = element.DoubleSpinBoxLabelLayout(CONST_ZRP_MINIMUM_Y_DISTANCE, 'Minimum Y Distance [m]', self.form_widget)
        vbox.addLayout(self.minimum_y_layout)

        self.maximum_z_layout = element.DoubleSpinBoxLabelLayout(CONST_ZRP_MAXIMUM_Z_DISTANCE, 'Maximum Z Distance [m]', self.form_widget)
        vbox.addLayout(self.maximum_z_layout)

        self.minimum_z_layout = element.DoubleSpinBoxLabelLayout(CONST_ZRP_MINIMUM_Z_DISTANCE, 'Minimum Z Distance [m]', self.form_widget)
        vbox.addLayout(self.minimum_z_layout)

        apply_btn = QPushButton('Apply')
        apply_btn.clicked.connect(lambda: self.Apply(self.form_widget.zrollpitch.DisplayGroundPoint,
                                                     self.form_widget.zrollpitch_tab.limit_time_layout.start_time,
                                                     self.form_widget.zrollpitch_tab.limit_time_layout.end_time,
                                                     self.form_widget.config.PARM_ZRP_DICT[self.idxSensor],
                                                     self.idxSensor,
                                                     self.EndApply))
        vbox.addWidget(apply_btn)

        groupbox.setLayout(vbox)

        return groupbox

    def Configuration_Calibration_Groupbox(self):
        groupbox = QGroupBox('Z, Roll, Pitch Calibration')
        vbox = QVBoxLayout(self)

        hbox = QHBoxLayout()
        label = QLabel('Plane Extract Method')
        hbox.addWidget(label)

        # button for SVD
        self.button_group1 = QButtonGroup()
        plane_rbn1 = QRadioButton('SVD')
        plane_rbn1.setChecked(True)
        plane_rbn1.clicked.connect(self.PlaneExtractRadioButton)
        hbox.addWidget(plane_rbn1)
        self.button_group1.addButton(plane_rbn1, 1)

        # button for Least Square
        plane_rbn2 = QRadioButton('LTSQ')
        plane_rbn2.clicked.connect(self.PlaneExtractRadioButton)
        hbox.addWidget(plane_rbn2)
        self.button_group1.addButton(plane_rbn2, 2)

        # button for RANSAC
        plane_rbn3 = QRadioButton('RANSAC')
        plane_rbn3.clicked.connect(self.PlaneExtractRadioButton)
        hbox.addWidget(plane_rbn3)
        self.button_group1.addButton(plane_rbn3, 3)
        vbox.addLayout(hbox)

        hbox = QHBoxLayout()
        label = QLabel('Optimization Method')
        hbox.addWidget(label)

        # button for Recursive Least Square
        self.button_group2 = QButtonGroup()
        opti_rbn1 = QRadioButton('Recursive Least Square')
        opti_rbn1.setChecked(True)
        opti_rbn1.clicked.connect(self.OptimizeRadioButton)
        hbox.addWidget(opti_rbn1)
        self.button_group2.addButton(opti_rbn1, 1)

        # button for Least Square
        opti_rbn2 = QRadioButton('Least Square')
        opti_rbn2.clicked.connect(self.OptimizeRadioButton)
        hbox.addWidget(opti_rbn2)
        self.button_group2.addButton(opti_rbn2, 2)
        vbox.addLayout(hbox)

        hbox = QHBoxLayout()
        btn = QPushButton('Start')
        btn.clicked.connect(self.Start)
        hbox.addWidget(btn)

        self.pause_btn = QPushButton('Pause')
        self.pause_btn.clicked.connect(self.Pause)
        hbox.addWidget(self.pause_btn)

        stop_btn = QPushButton('Stop')
        stop_btn.clicked.connect(self.Cancel)
        hbox.addWidget(stop_btn)
        vbox.addLayout(hbox)

        self.label = QLabel('[ Z, Roll, Pitch Calibration Progress ]')
        vbox.addWidget(self.label)

        self.calibration_pbar = QProgressBar(self)
        vbox.addWidget(self.calibration_pbar)

        self.text_edit = QTextEdit()
        vbox.addWidget(self.text_edit, 30)

        self.scroll_box = ScrollAreaV()
        vbox.addWidget(self.scroll_box, 70)

        groupbox.setLayout(vbox)
        return groupbox

    ## Callback func

    def Start(self):
        self.ErrorCheck()
        self.progress_status = CONST_PLAY

        self.PlotClear(self.result_graph1_ax, self.result_graph1_canvas)
        self.PlotClear(self.result_graph2_ax, self.result_graph2_canvas)
        self.text_edit.clear()
        self.calibration_pbar.reset()
        self.form_widget.rpz_thread._status = True
        self.form_widget.rpz_thread.SetFunc(self.form_widget.zrollpitch.Calibration,
                                            [self.form_widget.zrollpitch_tab.limit_time_layout.start_time,
                                             self.form_widget.zrollpitch_tab.limit_time_layout.end_time,
                                             self.form_widget.config.PARM_ZRP_DICT[self.idxSensor],
                                             self.plane_method,
                                             self.est_method,
                                             self.idxSensor])
        try:
            self.form_widget.rpz_thread.change_value.disconnect()
        except:
            pass
        try:
            self.form_widget.rpz_thread.iteration_percentage.disconnect()
        except:
            pass
        try:
            self.form_widget.rpz_thread.end.disconnect()
        except:
            pass
        try:
            self.form_widget.rpz_thread.emit_string.disconnect()
        except:
            pass

        self.form_widget.rpz_thread.emit_string.connect(self.text_edit.append)
        self.form_widget.rpz_thread.change_value.connect(self.calibration_pbar.setValue)

        self.form_widget.rpz_thread.end.connect(self.EndCalibration)
        self.form_widget.rpz_thread.start()

    def Pause(self):
        if self.progress_status is CONST_PLAY:
            self.progress_status = CONST_PAUSE

            self.form_widget.rpz_thread.pause = True
            self.pause_btn.setText("Resume")

        elif self.progress_status is CONST_PAUSE:
            self.progress_status = CONST_PLAY

            self.form_widget.rpz_thread.pause = False
            self.pause_btn.setText("Pause")

        else:
            return False

    def Cancel(self):
        self.progress_status = CONST_IDLE
        self.form_widget.rpz_thread.pause = False
        self.pause_btn.setText("Pause")
        self.form_widget.rpz_thread.toggle_status()

    def ViewGroundPoints1(self):
        self.form_widget.ViewGruondPoint2D(self.form_widget.zrollpitch.pointcloud,
                                           self.form_widget.zrollpitch.filtered_pointcloud)

    def ViewGroundPoints2(self):
        self.form_widget.ViewGroundPoint3D(self.form_widget.zrollpitch.pointcloud,
                                           self.form_widget.zrollpitch.filtered_pointcloud)

    def ViewMeanDistance(self):
        self.form_widget.ViewDistanceGraph(self.form_widget.zrollpitch.timestamp,
                                           self.form_widget.zrollpitch.mean_distance)

    def ViewGroundSlope(self):
        self.form_widget.ViewGroundSlopeGraph(self.form_widget.zrollpitch.timestamp,
                                              self.form_widget.zrollpitch.ground_slope)

    def PlaneExtractRadioButton(self):
        status = self.button_group1.checkedId()
        self.plane_method = status

    def OptimizeRadioButton(self):
        status = self.button_group2.checkedId()
        self.est_method = status

    def EndApply(self):
        self.progress_status = CONST_IDLE
        self.form_widget.zrollpitch.complete_calibration = True

        ## Plot 'Result Data'
        self.result_2d_data_pose_ax.clear()
        self.form_widget.ViewGruondPoint2D(self.form_widget.zrollpitch.pointcloud,
                                           self.form_widget.zrollpitch.filtered_pointcloud, self.result_2d_data_pose_ax,
                                           self.result_2d_data_pose_canvas)

        ## Plot 'Result Data'
        self.result_3d_data_pose_ax.clear()
        self.form_widget.ViewGroundPoint3D(self.form_widget.zrollpitch.pointcloud,
                                           self.form_widget.zrollpitch.filtered_pointcloud, self.result_3d_data_pose_ax,
                                           self.result_3d_data_pose_canvas)

        print("End Apply function")

    def EndCalibration(self):
        print("End z, roll, pitch calibration")
        self.progress_status = CONST_IDLE
        self.form_widget.zrollpitch.complete_calibration = True

        # ZRP tab
        self.result_graph1_ax.clear()
        self.form_widget.ViewDistanceGraph(self.form_widget.zrollpitch.timestamp,
                                           self.form_widget.zrollpitch.mean_distance,
                                           self.result_graph1_ax,
                                           self.result_graph1_canvas)

        ## Plot 'Result Graph'
        self.result_graph2_ax.clear()
        self.form_widget.ViewGroundSlopeGraph(self.form_widget.zrollpitch.timestamp,
                                              self.form_widget.zrollpitch.ground_slope,
                                              self.result_graph2_ax,
                                              self.result_graph2_canvas)

        self.calib_result[self.idxSensor] = copy.deepcopy(self.form_widget.zrollpitch.calib_result)

        ## Set 'Result Calibration Data'
        self.result_labels[self.idxSensor].label_edit1.setText(str(round(self.calib_result[self.idxSensor][0], 4)))
        self.result_labels[self.idxSensor].label_edit2.setText(str(round(self.calib_result[self.idxSensor][1], 4)))
        self.result_labels[self.idxSensor].label_edit3.setText(str(round(self.calib_result[self.idxSensor][2], 4)))

        # Data validation tab
        ## Set 'Result Calibration Data'
        self.form_widget.datavalidation_tab.zrp_result_labels[self.idxSensor].SetValue(self.calib_result[self.idxSensor])
        self.SetDefault(self.form_widget.datavalidation_tab.manual_zrp_calib_result, self.calib_result)

        # Handeye tab
        ## Set 'Result Calibration Data'
        self.form_widget.handeye_tab.zrp_result_labels[self.idxSensor].SetValue(self.calib_result[self.idxSensor])
        self.SetDefault(self.form_widget.handeye_tab.manual_zrp_calib_result, self.calib_result)

        # Unsupervised tab
        ## Set 'Result Calibration Data'
        self.form_widget.unsupervised_tab.zrp_result_labels[self.idxSensor].SetValue(self.calib_result[self.idxSensor])
        self.SetDefault(self.form_widget.unsupervised_tab.manual_zrp_calib_result, self.calib_result)

        # Evaluation tab
        ## Set 'Result Calibration Data'
        self.form_widget.evaluation_tab.zrp_result_labels[self.idxSensor].SetValue(self.calib_result[self.idxSensor])
        self.SetDefault(self.form_widget.evaluation_tab.manual_zrp_calib_result, self.calib_result)

    def CopyList(self, source, target):
        keys = list(source.keys())
        values = list(source.values())

        for i in range(len(keys)):
            target[keys[i]] = values[i].copy()

    ## Util funcs

    def SetDefault(self, manual_zrp_calib_result, calib_result):
        for idxSensor in self.form_widget.config.PARM_LIDAR["CheckedSensorList"]:
            if (manual_zrp_calib_result.get(idxSensor) is not None) and (calib_result.get(idxSensor) is not None):
                manual_zrp_calib_result[idxSensor] = calib_result[idxSensor]
            else:
                manual_zrp_calib_result[idxSensor] = [0.0, 0.0, 0.0]

class DataValidationTab(QWidget):
    def __init__(self, form_widget):
        super().__init__()
        self.form_widget = form_widget

        self.progress_status = CONST_IDLE
        self.zrp_result_labels = {}
        self.xyyaw_result_labels = {}
        self.manual_zrp_calib_result = {}        

        self.label_heading_threshold = self.form_widget.config.PARM_DV['FilterHeadingThreshold']
        self.label_distance_threshold = self.form_widget.config.PARM_DV['FilterDistanceThreshold']
        self.initUi()

    def initUi(self):
        main_vbox = QVBoxLayout()
        main_hbox = QHBoxLayout()

        self.config_widget = QWidget()
        self.config_widget.setLayout(self.Configuration_Layout())
        main_hbox.addWidget(self.config_widget, 30)

        self.result_widget = QWidget()
        self.result_widget.setLayout(self.Result_Layout())
        main_hbox.addWidget(self.result_widget, 70)

        main_vbox.addLayout(main_hbox, 50)

        skip_btn = QPushButton('Skip')
        skip_btn.clicked.connect(self.Skip)
        main_vbox.addWidget(skip_btn, 50)

        self.setLayout(main_vbox)

    ## Layout
    def Configuration_Layout(self):
        self.configuration_vbox = QVBoxLayout()

        self.configuration_vbox.addWidget(self.Configuration_DataInfo_Groupbox())

        self.configuration_vbox.addWidget(self.Configuration_Validation_Groupbox())

        return self.configuration_vbox

    def Result_Layout(self):
        vbox = QVBoxLayout()

        vbox.addWidget(self.Result_ResultData_Groupbox())
        vbox.addWidget(self.Result_ResultGraph_Groupbox())

        return vbox

    ## Groupbox
    def Configuration_DataInfo_Groupbox(self):
        groupbox = QGroupBox('Configuration Info')
        vbox = QVBoxLayout()

        height_roll_pitch_param_label = QLabel('[ Applying Height, Roll, Pitch Parameters ]', self)
        vbox.addWidget(height_roll_pitch_param_label)

        hbox = QHBoxLayout()
        label = QLabel()
        hbox.addWidget(label, 10)
        label = QLabel('Automatic')
        hbox.addWidget(label, 10)
        label = QLabel('Manual')
        hbox.addWidget(label, 10)
        label = QLabel('z [m]')
        hbox.addWidget(label, 10)
        label = QLabel('roll [deg]')
        hbox.addWidget(label, 10)
        label = QLabel('pitch [deg]')
        hbox.addWidget(label, 10)
        vbox.addLayout(hbox)

        self.zrp_scroll_box = ScrollAreaV()
        vbox.addWidget(self.zrp_scroll_box)

        ICP_configuration_label = QLabel('[ ICP Configuration ]', self)
        vbox.addWidget(ICP_configuration_label)

        self.maximum_interation_layout = element.SpinBoxLabelLayout('Maximum Iteration [Count]', self.form_widget)
        vbox.addLayout(self.maximum_interation_layout)

        self.tolerance_layout = element.DoubleSpinBoxLabelLayout(CONST_DATAVALIDATION_TOLERANCE, 'Tolerance', self.form_widget, 12)
        vbox.addLayout(self.tolerance_layout)

        self.outlier_distance_layout = element.DoubleSpinBoxLabelLayout(CONST_DATAVALIDATION_OUTLIER_DISTANCE, 'Outlier Distance [m]', self.form_widget)
        vbox.addLayout(self.outlier_distance_layout)

        Threshold_configuration_label = QLabel('[ Threshold ]', self)
        vbox.addWidget(Threshold_configuration_label)
        self.heading_threshold_layout = element.DoubleSpinBoxLabelLayout(CONST_DATAVALIDATION_HEADING_THRESHOLD, 'Heading Threshold (filter)', self.form_widget)
        vbox.addLayout(self.heading_threshold_layout)

        self.distance_threshold_layout = element.DoubleSpinBoxLabelLayout(CONST_DATAVALIDATION_DISTANCE_THRESHOLD, 'Distance Threshold (filter)',
                                                                          self.form_widget)
        vbox.addLayout(self.distance_threshold_layout)

        groupbox.setLayout(vbox)

        return groupbox

    def Configuration_Validation_Groupbox(self):
        groupbox = QGroupBox('Data Validation')
        vbox = QVBoxLayout(self)

        buttons = {0:'GNSS Data', 1:'Motion Data'}
        self.radio_label_layout = element.RadioLabelLayout(CONST_DATAVALIDATION_USED_DATA, 'Used Data', buttons, self.form_widget)
        vbox.addLayout(self.radio_label_layout)

        hbox = QHBoxLayout()
        btn = QPushButton('Start')
        btn.clicked.connect(lambda: self.StartValidation(self.form_widget.datavalidation.Validation,
                                                         self.form_widget.config.PARM_IM['VehicleSpeedThreshold'],
                                                         self.form_widget.importing_tab.limit_time_layout.start_time,
                                                         self.form_widget.importing_tab.limit_time_layout.end_time,
                                                         self.form_widget.config.PARM_LIDAR,
                                                         self.form_widget.GetZRPCalibDict(self.zrp_result_labels, self.manual_zrp_calib_result),
                                                         [self.text_edit.clear, self.calibration_pbar.reset],
                                                         [self.text_edit.append, self.calibration_pbar.setValue],
                                                         self.EndValibration))

        hbox.addWidget(btn)

        self.pause_btn = QPushButton('Pause')
        self.pause_btn.clicked.connect(self.Pause)
        hbox.addWidget(self.pause_btn)
        vbox.addLayout(hbox)

        stop_btn = QPushButton('Stop')
        stop_btn.clicked.connect(self.Cancel)
        hbox.addWidget(stop_btn)

        vbox.addLayout(hbox)

        self.label = QLabel('[ Data Validation Progress ]')
        vbox.addWidget(self.label)

        self.calibration_pbar = QProgressBar(self)
        vbox.addWidget(self.calibration_pbar)

        self.text_edit = QTextEdit()
        vbox.addWidget(self.text_edit, 30)

        self.scroll_box = ScrollAreaV()
        vbox.addWidget(self.scroll_box, 70)

        groupbox.setLayout(vbox)
        return groupbox

    def Result_ResultData_Groupbox(self):
        groupbox = QGroupBox('Translation RMSE')
        vbox = QVBoxLayout()

        self.result_data_pose_fig = plt.figure()
        self.result_data_pose_canvas = FigureCanvas(self.result_data_pose_fig)
        self.result_data_pose_ax = self.result_data_pose_fig.add_subplot(1, 1, 1)
        self.result_data_pose_ax.grid()
        self.result_data_pose_canvas.draw()
        vbox.addWidget(self.result_data_pose_canvas)

        btn = QPushButton('View')
        btn.clicked.connect(self.ViewTranslationError)
        vbox.addWidget(btn)

        groupbox.setLayout(vbox)
        return groupbox

    def Result_ResultGraph_Groupbox(self):
        groupbox = QGroupBox('Rotation RMSE')
        vbox = QVBoxLayout()

        self.result_graph_fig = plt.figure()
        self.result_graph_canvas = FigureCanvas(self.result_graph_fig)
        self.result_graph_ax = self.result_graph_fig.add_subplot(1, 1, 1)
        self.result_graph_ax.grid()
        self.result_graph_canvas.draw()
        vbox.addWidget(self.result_graph_canvas)

        btn = QPushButton('View')
        btn.clicked.connect(self.ViewRotationError)
        vbox.addWidget(btn)

        groupbox.setLayout(vbox)
        return groupbox

    ## Callback func

    def StartValidation(self, validation, vehicle_speed_threshold, start_time, end_time, sensor_list,
                        zrp_calib, targets_clear, progress_callbacks, end_callback):
        if self.form_widget.config_tab.is_lidar_num_changed == True:
            self.form_widget.ErrorPopUp('Please import after changing lidar number')
            return False
        if self.progress_status is not CONST_IDLE:
            return False
        self.progress_status = CONST_PLAY

        for idxSensor in sensor_list['CheckedSensorList']:
            if self.form_widget.importing.PointCloudSensorList.get(idxSensor) is None:
                self.form_widget.ErrorPopUp('Import pointcloud {}'.format(idxSensor))
                return False

        for target_clear in targets_clear:
            target_clear()

        try:
            self.result_data_pose_ax.clear()
            self.result_data_pose_canvas.draw()
            self.result_graph_ax.clear()
            self.result_graph_canvas.draw()
        except:
            pass
        self.form_widget.validation_thread._status = True
        self.form_widget.validation_thread.SetFunc(validation,
                                                   [start_time, end_time, sensor_list, self.radio_label_layout.using_gnss_motion,
                                                    vehicle_speed_threshold, zrp_calib])
        try:
            self.form_widget.validation_thread.change_value.disconnect()
        except:
            pass
        try:
            self.form_widget.validation_thread.iteration_percentage.disconnect()
        except:
            pass
        try:
            self.form_widget.validation_thread.end.disconnect()
        except:
            pass
        try:
            self.form_widget.validation_thread.emit_string.disconnect()
        except:
            pass

        self.form_widget.validation_thread.emit_string.connect(progress_callbacks[0])  # text_edit_callback
        self.form_widget.validation_thread.change_value.connect(progress_callbacks[1])  # progress_callback

        self.form_widget.validation_thread.end.connect(end_callback)
        self.form_widget.validation_thread.start()

    def Pause(self):
        if self.progress_status is CONST_PLAY:
            self.progress_status = CONST_PAUSE

            self.form_widget.validation_thread.pause = True
            self.pause_btn.setText("Resume")

        elif self.progress_status is CONST_PAUSE:
            self.progress_status = CONST_PLAY

            self.form_widget.validation_thread.pause = False
            self.pause_btn.setText("Pause")

        else:
            return False

    def Cancel(self):
        self.progress_status = CONST_IDLE
        self.form_widget.validation_thread.pause = False
        self.pause_btn.setText("Pause")
        self.form_widget.validation_thread.toggle_status()

    def EndValibration(self):
        self.progress_status = CONST_IDLE
        self.form_widget.datavalidation.complete_validation = True

        # validation tab
        ## Set 'Result Calibration Data'

        for idxSensor in self.form_widget.datavalidation.PARM_LIDAR['CheckedSensorList']:
            self.xyyaw_result_labels[idxSensor].label_edit_translation_error.setText(format(self.form_widget.datavalidation.RMSETranslationErrorDict[idxSensor], ".4f"))
            self.xyyaw_result_labels[idxSensor].label_edit_rotation_error.setText(format(self.form_widget.datavalidation.RMSERotationErrorDict[idxSensor], ".4f"))

        ## Plot 'Result Data'
        self.result_data_pose_ax.clear()
        self.ViewTranslationError()

        ## Plot 'Result Graph'
        self.result_graph_ax.clear()
        self.ViewRotationError()

    def ViewTranslationError(self):
        if self.progress_status is not CONST_IDLE:
            return False
        self.form_widget.ViewTranslationError(self.form_widget.datavalidation.TranslationError,
                                              self.form_widget.datavalidation.PARM_LIDAR, self.result_data_pose_ax,
                                              self.result_data_pose_canvas)

    def ViewRotationError(self):
        if self.progress_status is not CONST_IDLE:
            return False
        self.form_widget.ViewRotationError(self.form_widget.datavalidation.RotationError,
                                           self.form_widget.datavalidation.PARM_LIDAR, self.result_graph_ax,
                                           self.result_graph_canvas)

    def Skip(self):
        self.form_widget.tabs.setCurrentIndex(CONST_HANDEYE_TAB)

    def PlotClear(self, ax, canvas):
        # remove plots
        try:
            ax.clear()
            canvas.draw()
        except:
            pass

    def Clear(self):
        # Reset data validation values
        try:
            self.form_widget.datavalidation.RMSETranslationErrorDict.clear()
            self.form_widget.datavalidation.RMSERotationErrorDict.clear()
            self.form_widget.datavalidation.PARM_LIDAR.clear()
        except:
            pass

        # Clear scroll box result
        self.form_widget.ResetValidationLabel(CONST_UNEDITABLE_LABEL,
                                  self.form_widget.config.PARM_LIDAR,
                                  self.scroll_box.layout,
                                  self.xyyaw_result_labels,
                                  self.form_widget.datavalidation.RMSETranslationErrorDict,
                                  self.form_widget.datavalidation.RMSERotationErrorDict)

        # Clear HMI
        self.PlotClear(self.result_data_pose_ax, self.result_data_pose_canvas)
        self.PlotClear(self.result_graph_ax, self.result_graph_canvas)
        self.text_edit.clear()
        self.calibration_pbar.reset()

class XYYaw_CalibrationTab(QWidget):
    def __init__(self, form_widget):
        super().__init__()
        self.form_widget = form_widget

        self.progress_status = CONST_IDLE
        self.zrp_result_labels = {}
        self.calib_result = []
        self.xyyaw_result_labels = {}
        self.manual_zrp_calib_result = {}        

        self.calib_x = []
        self.calib_y = []
        self.calib_z = []
        self.calib_r = []
        self.calib_p = []
        self.calib_yaw = []

        self.PARM_LIDAR = []

        self.is_single = True
        self.initUi()

    def initUi(self):
        main_hbox = QHBoxLayout()

        self.config_widget = QWidget()
        self.config_widget.setLayout(self.Configuration_Layout())
        main_hbox.addWidget(self.config_widget, 30)

        self.result_widget = QWidget()
        self.result_widget.setLayout(self.Result_Layout())
        main_hbox.addWidget(self.result_widget, 70)
        self.setLayout(main_hbox)

    ## Layout
    def Configuration_Layout(self):
        self.configuration_vbox = QVBoxLayout()

        self.configuration_vbox.addWidget(self.Configuration_SetConfiguration_Groupbox())

        self.configuration_vbox.addWidget(self.Configuration_Calibration_Groupbox())

        return self.configuration_vbox

    def Result_Layout(self):
        vbox = QVBoxLayout()

        vbox.addWidget(self.Result_ResultData_Groupbox())
        vbox.addWidget(self.Result_ResultGraph_Groupbox())

        return vbox

    ## Groupbox

    def Result_ResultData_Groupbox(self):
        groupbox = QGroupBox('Result Data')
        vbox = QVBoxLayout()

        self.vtkWidget = QVTKRenderWindowInteractor()
        vbox.addWidget(self.vtkWidget)
        self.VTKInit(is_default=True)

        hbox = QHBoxLayout()

        btn = QPushButton('xy plane')
        btn.clicked.connect(lambda: self.form_widget.XY(self.ren, self.renWin))
        hbox.addWidget(btn)

        btn = QPushButton('yz plane')
        btn.clicked.connect(lambda: self.form_widget.YZ(self.ren, self.renWin))
        hbox.addWidget(btn)

        btn = QPushButton('zx plane')
        btn.clicked.connect(lambda: self.form_widget.ZX(self.ren, self.renWin))
        hbox.addWidget(btn)
        vbox.addLayout(hbox)

        btn = QPushButton('View')
        btn.clicked.connect(self.ViewLiDAR)
        vbox.addWidget(btn)

        groupbox.setLayout(vbox)
        return groupbox

    def Result_ResultGraph_Groupbox(self):
        groupbox = QGroupBox('Result Graph')
        vbox = QVBoxLayout()

        self.result_graph_fig = plt.figure()
        self.result_graph_canvas = FigureCanvas(self.result_graph_fig)
        self.result_graph_ax = self.result_graph_fig.add_subplot(1, 1, 1)
        self.result_graph_ax.grid()
        self.result_graph_canvas.draw()
        vbox.addWidget(self.result_graph_canvas)

        btn = QPushButton('View')
        btn.clicked.connect(self.ViewPointCloud)
        vbox.addWidget(btn)

        groupbox.setLayout(vbox)
        return groupbox

    ## Callback func

    def ViewLiDAR(self):
        if self.progress_status is not CONST_IDLE:
            return False

        self.dialog = element.NewWindow('ViewLiDAR', self.calib_result, self.form_widget)

    def ViewPointCloud(self):
        pass

    ## Util functions

    def PlotClear(self, ax, canvas):
        # remove plots
        try:
            ax.clear()
            canvas.draw()
        except:
            pass

    def ErrorCheck(self, sensor_list):
        if self.form_widget.config_tab.is_lidar_num_changed == True:
            self.form_widget.ErrorPopUp('Please import after changing lidar number')
            return False
        if self.progress_status is not CONST_IDLE:
            return False

        for idxSensor in sensor_list['CheckedSensorList']:
            if self.form_widget.importing.PointCloudSensorList.get(idxSensor) is None:
                self.form_widget.ErrorPopUp('Import pointcloud {}'.format(idxSensor))
                return False

    def VTKInit(self, is_default):
        vtk.vtkObject.GlobalWarningDisplayOff()
        colors = vtk.vtkNamedColors()

        self.ren = vtk.vtkRenderer()

        self.renWin = self.vtkWidget.GetRenderWindow()
        self.renWin.AddRenderer(self.ren)

        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        self.widget = vtk.vtkOrientationMarkerWidget()

        self.iren.SetRenderWindow(self.renWin)

        self.ren.SetBackground(colors.GetColor3d("white"))
        self.iren.Initialize()

        if is_default:
            vtk_lidar_calib.GetAxis(self.iren, self.widget)
            self.ren.AddActor(vtk_lidar_calib.GetGridActor())
            self.ren.AddActor(vtk_lidar_calib.GetVehicleActor())

            colors = vtk.vtkNamedColors()
            self.ren.SetBackground(colors.GetColor3d('white'))

            self.ren.ResetCamera()
            self.renWin.Render()

            self.iren.Start()

class HandEyeTab(XYYaw_CalibrationTab):
    def __init__(self, form_widget):
        super().__init__(form_widget)

    ## Groupbox
    def Configuration_SetConfiguration_Groupbox(self):
        groupbox = QGroupBox('Set Configuration')
        vbox = QVBoxLayout()

        height_roll_pitch_param_label = QLabel('[ Applying Height, Roll, Pitch Parameters ]', self)
        vbox.addWidget(height_roll_pitch_param_label)

        hbox = QHBoxLayout()
        label = QLabel()
        hbox.addWidget(label, 10)
        label = QLabel('Automatic')
        hbox.addWidget(label, 10)
        label = QLabel('Manual')
        hbox.addWidget(label, 10)
        label = QLabel('z [m]')
        hbox.addWidget(label, 10)
        label = QLabel('roll [deg]')
        hbox.addWidget(label, 10)
        label = QLabel('pitch [deg]')
        hbox.addWidget(label, 10)
        vbox.addLayout(hbox)

        self.zrp_scroll_box = ScrollAreaV()
        vbox.addWidget(self.zrp_scroll_box)

        liDAR_configuration_label = QLabel('[ HandEye Configuration ]', self)
        vbox.addWidget(liDAR_configuration_label)

        self.maximum_interation_layout = element.SpinBoxLabelLayout('Maximum Iteration [Count]', self.form_widget)
        vbox.addLayout(self.maximum_interation_layout)

        self.tolerance_layout = element.DoubleSpinBoxLabelLayout(CONST_HANDEYE_TOLERANCE, 'Tolerance', self.form_widget, 12)
        vbox.addLayout(self.tolerance_layout)

        self.outlier_distance_layout = element.DoubleSpinBoxLabelLayout(CONST_HANDEYE_OUTLIER_DISTANCE, 'Outlier Distance [m]', self.form_widget)
        vbox.addLayout(self.outlier_distance_layout)

        self.heading_threshold_layout = element.DoubleSpinBoxLabelLayout(CONST_HANDEYE_HEADING_THRESHOLD, 'Heading Threshold (filter)', self.form_widget)
        vbox.addLayout(self.heading_threshold_layout)

        self.distance_threshold_layout = element.DoubleSpinBoxLabelLayout(CONST_HANDEYE_DISTANCE_THRESHOLD, 'Distance Threshold (filter)', self.form_widget)
        vbox.addLayout(self.distance_threshold_layout)

        groupbox.setLayout(vbox)
        return groupbox

    def Configuration_Calibration_Groupbox(self):
        groupbox = QGroupBox('HandEye Calibration')
        vbox = QVBoxLayout(self)

        buttons = {0:'GNSS Data', 1:'Motion Data'}
        self.radio_label_layout = element.RadioLabelLayout(CONST_HANDEYE_USED_DATA, 'Used Data', buttons, self.form_widget)
        vbox.addLayout(self.radio_label_layout)

        hbox = QHBoxLayout()
        btn = QPushButton('Start')

        btn.clicked.connect(self.Start)
        hbox.addWidget(btn)

        self.pause_btn = QPushButton('Pause')
        self.pause_btn.clicked.connect(self.Pause)
        hbox.addWidget(self.pause_btn)

        stop_btn = QPushButton('Stop')
        stop_btn.clicked.connect(self.Cancel)
        hbox.addWidget(stop_btn)
        vbox.addLayout(hbox)

        self.label = QLabel('[ HandEye Calibration Progress ]')
        vbox.addWidget(self.label)

        self.calibration_pbar = QProgressBar(self)
        vbox.addWidget(self.calibration_pbar)

        self.text_edit = QTextEdit()
        vbox.addWidget(self.text_edit, 30)

        self.scroll_box = ScrollAreaV()
        vbox.addWidget(self.scroll_box, 70)

        groupbox.setLayout(vbox)
        return groupbox

    ## Callback func

    def Start(self):
        self.ErrorCheck(self.form_widget.config.PARM_LIDAR)
        self.progress_status = CONST_PLAY

        self.VTKInit(is_default=True)
        self.PlotClear(self.result_graph_ax, self.result_graph_canvas)
        self.text_edit.clear()
        self.calibration_pbar.reset()

        # disable access to unsupervised tab until complete handeye calibration
        self.form_widget.tabs.setTabEnabled(CONST_UNSUPERVISED_TAB, False)
        self.form_widget.tabs.setTabEnabled(CONST_EVALUATION_TAB, False)
        self.form_widget.handeye_thread._status = True
        self.form_widget.handeye_thread.SetFunc(self.form_widget.handeye.Calibration,
                               [self.form_widget.importing_tab.limit_time_layout.start_time, 
                                self.form_widget.importing_tab.limit_time_layout.end_time,
                                self.form_widget.config.PARM_LIDAR,
                                self.radio_label_layout.using_gnss_motion,
                                self.form_widget.config.PARM_IM['VehicleSpeedThreshold'],
                                self.form_widget.GetZRPCalibDict(self.zrp_result_labels, self.manual_zrp_calib_result)])
        try:
            self.form_widget.handeye_thread.change_value.disconnect()
        except:
            pass
        try:
            self.form_widget.handeye_thread.iteration_percentage.disconnect()
        except:
            pass
        try:
            self.form_widget.handeye_thread.end.disconnect()
        except:
            pass
        try:
            self.form_widget.handeye_thread.emit_string.disconnect()
        except:
            pass

        self.form_widget.handeye_thread.emit_string.connect(self.text_edit.append)
        self.form_widget.handeye_thread.change_value.connect(self.calibration_pbar.setValue)

        self.form_widget.handeye_thread.end.connect(self.EndCalibration)
        self.form_widget.handeye_thread.start()
    
    def Pause(self):
        if self.progress_status is CONST_PLAY:
            self.progress_status = CONST_PAUSE

            self.form_widget.handeye_thread.pause = True
            self.pause_btn.setText("Resume")

        elif self.progress_status is CONST_PAUSE:
            self.progress_status = CONST_PLAY

            self.form_widget.handeye_thread.pause = False
            self.pause_btn.setText("Pause")

        else:
            return False

    def Cancel(self):
        self.progress_status = CONST_IDLE
        self.form_widget.handeye_thread.pause = False
        self.pause_btn.setText("Pause")
        self.form_widget.handeye_thread.toggle_status()

    def ViewPointCloud(self):
        if self.progress_status is not CONST_IDLE:
            return False

        calib_result_dict = self.form_widget.GetZRPCalibDict(self.zrp_result_labels, self.manual_zrp_calib_result)
        for key in self.form_widget.handeye.CalibrationParam:
            self.form_widget.handeye.CalibrationParam[key][0] = calib_result_dict[key][1]
            self.form_widget.handeye.CalibrationParam[key][1] = calib_result_dict[key][2]
            self.form_widget.handeye.CalibrationParam[key][5] = calib_result_dict[key][0]

        df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.form_widget.importing,
                                                                                           self.radio_label_layout.using_gnss_motion,
                                                                                           self.form_widget.handeye.PARM_LIDAR,
                                                                                           self.form_widget.handeye.CalibrationParam,
                                                                                           self.form_widget.importing_tab.limit_time_layout.start_time,
                                                                                           self.form_widget.importing_tab.limit_time_layout.end_time)


        self.form_widget.ViewPointCloud(df_info, accum_pointcloud, PARM_LIDAR)

    def EndCalibration(self):
        zrp_result = self.form_widget.GetZRPCalibList(self.zrp_result_labels, self.manual_zrp_calib_result)
        self.calib_result = [self.form_widget.handeye.calib_x, self.form_widget.handeye.calib_y,
                             zrp_result[0], zrp_result[1], zrp_result[2],
                             self.form_widget.handeye.calib_yaw]
        self.PARM_LIDAR = self.form_widget.handeye.PARM_LIDAR

        calib_result_dict = self.form_widget.GetZRPCalibDict(self.zrp_result_labels, self.manual_zrp_calib_result)
        for key in self.form_widget.handeye.CalibrationParam:
            self.form_widget.handeye.CalibrationParam[key][0] = calib_result_dict[key][1]
            self.form_widget.handeye.CalibrationParam[key][1] = calib_result_dict[key][2]
            self.form_widget.handeye.CalibrationParam[key][5] = calib_result_dict[key][0]

        self.progress_status = CONST_IDLE
        self.form_widget.tabs.setTabEnabled(CONST_UNSUPERVISED_TAB, True)
        self.form_widget.tabs.setTabEnabled(CONST_EVALUATION_TAB, True)
        self.form_widget.handeye.complete_calibration = True

        self.form_widget.ResetResultsLabels(self.PARM_LIDAR)
        self.form_widget.evaluation_tab.eval_lidar['CheckedSensorList'] = copy.deepcopy(self.PARM_LIDAR['CheckedSensorList'])

        df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.form_widget.importing,
                                                                                           self.radio_label_layout.using_gnss_motion,
                                                                                           self.PARM_LIDAR,
                                                                                           self.form_widget.handeye.CalibrationParam,
                                                                                           self.form_widget.importing_tab.limit_time_layout.start_time,
                                                                                           self.form_widget.importing_tab.limit_time_layout.end_time)
        # Handeye tab
        ## Set 'Result Calibration Data'
        for idxSensor in PARM_LIDAR['CheckedSensorList']:
            self.xyyaw_result_labels[idxSensor].label_edit1.setText(format(self.form_widget.handeye.CalibrationParam[idxSensor][3], ".4f"))
            self.xyyaw_result_labels[idxSensor].label_edit2.setText(format(self.form_widget.handeye.CalibrationParam[idxSensor][4], ".4f"))
            self.xyyaw_result_labels[idxSensor].label_edit3.setText(format(self.form_widget.handeye.CalibrationParam[idxSensor][2] * 180 / math.pi, ".4f"))

        ## Plot 'Result Data'
        self.VTKInit(is_default=False)
        self.form_widget.DisplayLiDAR(self.calib_result, PARM_LIDAR, self.ren, self.iren, self.renWin, self.widget)

        ## Plot 'Result Graph'
        self.result_graph_ax.clear()
        self.form_widget.ViewPointCloud(df_info, accum_pointcloud, PARM_LIDAR, self.result_graph_ax, self.result_graph_canvas)

        ## Transfer
        self.CopyList(self.form_widget.handeye.CalibrationParam, self.form_widget.unsupervised_tab.edit_handeye_calibration_parm)


        # Unsupervised tab
        ## Set 'Unsupervised Initial Value'
        for idxSensor in PARM_LIDAR['CheckedSensorList']:
            self.form_widget.unsupervised_tab.handeye_result_labels[idxSensor].double_spin_box_x.setValue(self.form_widget.handeye.CalibrationParam[idxSensor][3])
            self.form_widget.unsupervised_tab.handeye_result_labels[idxSensor].double_spin_box_y.setValue(self.form_widget.handeye.CalibrationParam[idxSensor][4])
            self.form_widget.unsupervised_tab.handeye_result_labels[idxSensor].double_spin_box_yaw.setValue(self.form_widget.handeye.CalibrationParam[idxSensor][2] * 180 / math.pi)

        ## Transfer
        self.form_widget.unsupervised.CalibrationParam = copy.deepcopy(self.form_widget.handeye.CalibrationParam)


        # Evaluation tab
        ## Set 'Select The Method'
        for idxSensor in PARM_LIDAR['CheckedSensorList']:
            self.form_widget.evaluation_tab.xyyaw_result_labels[idxSensor].button_group.button(CONST_HANDEYE).setChecked(True)
            self.form_widget.evaluation_tab.xyyaw_result_labels[idxSensor].prev_checkID = CONST_HANDEYE
            self.form_widget.evaluation_tab.xyyaw_result_labels[idxSensor].spinbox1.setValue(self.form_widget.handeye.CalibrationParam[idxSensor][3])
            self.form_widget.evaluation_tab.xyyaw_result_labels[idxSensor].spinbox2.setValue(self.form_widget.handeye.CalibrationParam[idxSensor][4])
            self.form_widget.evaluation_tab.xyyaw_result_labels[idxSensor].spinbox3.setValue(self.form_widget.handeye.CalibrationParam[idxSensor][2] * 180 / math.pi)
            self.form_widget.evaluation_tab.custom_calibration_param[idxSensor] = copy.deepcopy(self.form_widget.handeye.CalibrationParam[idxSensor])

    def CopyList(self, source, target):
        keys = list(source.keys())
        values = list(source.values())

        for i in range(len(keys)):
            target[keys[i]] = values[i].copy()

    def Clear(self):
        # Reset handeye values
        try:
            self.form_widget.handeye.calib_x.clear()
            self.form_widget.handeye.calib_y.clear()
            self.form_widget.handeye.calib_yaw.clear()
            self.form_widget.handeye.PARM_LIDAR.clear()
            self.form_widget.handeye.CalibrationParam.clear()
        except:
            pass
        self.calib_x = []
        self.calib_y = []
        self.calib_z = []
        self.calib_r = []
        self.calib_p = []
        self.calib_yaw = []
        self.PARM_LIDAR = []

        # Clear calib result
        self.calib_result.clear()
        self.manual_zrp_calib_result.clear()
        self.form_widget.ResetResultsLabel(CONST_UNEDITABLE_LABEL,
                                           self.form_widget.config.PARM_LIDAR,
                                           self.scroll_box.layout,
                                           self.xyyaw_result_labels,
                                           self.form_widget.handeye.CalibrationParam)

        # Clear HMI
        self.VTKInit(is_default=True)
        self.PlotClear(self.result_graph_ax, self.result_graph_canvas)
        self.text_edit.clear()
        self.calibration_pbar.reset()

        self.progress_status = CONST_IDLE

        self.radio_label_layout.button_group.button(1).setChecked(True)
        self.radio_label_layout.RadioButton()

class UnsupervisedTab(XYYaw_CalibrationTab):
    def __init__(self, form_widget):
        super().__init__(form_widget)
        self.edit_handeye_calibration_parm = {}
        self.user_define_initial_labels = {}
        self.handeye_result_labels = {}

    ## Groupbox
    def Configuration_SetConfiguration_Groupbox(self):
        self.optimization_configuration_groupbox = QGroupBox('Set Configuration')
        vbox = QVBoxLayout()

        height_roll_pitch_param_label = QLabel('[ Applying Height, Roll, Pitch Parameters ]', self)
        vbox.addWidget(height_roll_pitch_param_label)

        hbox = QHBoxLayout()
        label = QLabel()
        hbox.addWidget(label, 10)
        label = QLabel('Automatic')
        hbox.addWidget(label, 10)
        label = QLabel('Manual')
        hbox.addWidget(label, 10)
        label = QLabel('z [m]')
        hbox.addWidget(label, 10)
        label = QLabel('roll [deg]')
        hbox.addWidget(label, 10)
        label = QLabel('pitch [deg]')
        hbox.addWidget(label, 10)
        vbox.addLayout(hbox)

        self.zrp_scroll_box = ScrollAreaV()
        vbox.addWidget(self.zrp_scroll_box)

        liDAR_configuration_label = QLabel('[ LiDAR Configuration ]', self)
        vbox.addWidget(liDAR_configuration_label)

        buttons = {0:'Single LiDAR', 1:'Multi LiDAR'}
        self.select_lidar_num_layout = element.RadioLabelLayout(CONST_UNSUPERVISED_SELECT_LIDAR, 'Select Lidar Type', buttons, self.form_widget)
        vbox.addLayout(self.select_lidar_num_layout)

        self.select_lidar_combobox_layout = element.ComboBoxLabelLayout(CONST_UNSUPERVISED_SELECT_LIDAR_TO_CALIB, 'Select Single Lidar To Calibration', self.form_widget)
        vbox.addLayout(self.select_lidar_combobox_layout)

        liDAR_configuration_label = QLabel('[ Unsupervised Configuration ]', self)
        vbox.addWidget(liDAR_configuration_label)

        self.point_sampling_ratio_layout = element.DoubleSpinBoxLabelLayout(CONST_OPTI_POINT_SAMPLING_RATIO, 'Point Sampling Ratio', self.form_widget)
        vbox.addLayout(self.point_sampling_ratio_layout)

        self.num_points_plane_modeling_layout = element.SpinBoxLabelLayout('Num Points Plane Modeling', self.form_widget)
        vbox.addLayout(self.num_points_plane_modeling_layout)

        self.outlier_distance_layout = element.DoubleSpinBoxLabelLayout(CONST_OPTI_OUTLIER_DISTANCE, 'Outlier Distance [m]', self.form_widget)
        vbox.addLayout(self.outlier_distance_layout)

        optimization_initial_value_label = QLabel('[ Unsupervised Initial Value ]', self)
        vbox.addWidget(optimization_initial_value_label)

        self.optimization_initial_value_tab = element.ResultTab('Handeye', 'User Define', self.form_widget)
        vbox.addLayout(self.optimization_initial_value_tab)

        self.optimization_configuration_groupbox.setLayout(vbox)
        return self.optimization_configuration_groupbox

    def Configuration_Calibration_Groupbox(self):
        groupbox = QGroupBox('Unsupervised Calibration')
        vbox = QVBoxLayout(self)

        buttons = {0:'GNSS Data', 1:'Motion Data'}
        self.radio_label_layout = element.RadioLabelLayout(CONST_UNSUPERVISED_USED_DATA, 'Used Data', buttons, self.form_widget)
        vbox.addLayout(self.radio_label_layout)

        hbox = QHBoxLayout(self)
        btn = QPushButton('Start')
        btn.clicked.connect(self.Start)
        hbox.addWidget(btn)

        self.stop_btn = QPushButton('Stop')
        self.stop_btn.clicked.connect(self.Stop)
        hbox.addWidget(self.stop_btn)
        vbox.addLayout(hbox)

        label = QLabel('[ Unsupervised Progress ]')
        # label.setFont(QFont('MSGOTHIC',10))
        vbox.addWidget(label)

        self.text_edit = QTextEdit()
        self.text_edit.setFont(QFont('MSGOTHIC',10))
        vbox.addWidget(self.text_edit, 30)

        self.scroll_box = ScrollAreaV()
        vbox.addWidget(self.scroll_box, 70)

        groupbox.setLayout(vbox)
        return groupbox

    ## Callback func
    def Start(self):
        self.ErrorCheck(self.form_widget.config.PARM_LIDAR)
        self.progress_status = CONST_PLAY

        self.PlotClear(self.result_graph_ax, self.result_graph_canvas)
        self.text_edit.clear()

        self.form_widget.opti_thread._status = True
        self.form_widget.opti_thread.SetFunc(self.form_widget.unsupervised.Calibration, 
                                            [self.form_widget.importing_tab.limit_time_layout.start_time,
                                            self.form_widget.importing_tab.limit_time_layout.end_time,
                                            self.form_widget.config.PARM_LIDAR,
                                            self.radio_label_layout.using_gnss_motion,
                                            self.form_widget.config.PARM_IM['VehicleSpeedThreshold'],
                                            self.form_widget.GetZRPCalibDict(self.zrp_result_labels, self.manual_zrp_calib_result),
                                            self.is_single])
        try:
            self.form_widget.opti_thread.change_value.disconnect()
        except:
            pass
        try:
            self.form_widget.opti_thread.iteration_percentage.disconnect()
        except:
            pass
        try:
            self.form_widget.opti_thread.end.disconnect()
        except:
            pass
        try:
            self.form_widget.opti_thread.emit_string.disconnect()
        except:
            pass


        self.form_widget.opti_thread.emit_string.connect(self.text_edit.append)
        self.form_widget.opti_thread.end.connect(self.EndCalibration)
        self.form_widget.opti_thread.start()

    def Stop(self):
        self.progress_status = CONST_IDLE

        self.form_widget.opti_thread.toggle_status()

        self.form_widget.ErrorPopUp('Please wait for stop the calibration')

    def ViewPointCloud(self):
        if self.progress_status is not CONST_IDLE:
            return False

        calib_result_dict = self.form_widget.GetZRPCalibDict(self.zrp_result_labels, self.manual_zrp_calib_result)
        for key in self.form_widget.unsupervised.CalibrationParam:
            self.form_widget.unsupervised.CalibrationParam[key][0] = calib_result_dict[key][1]
            self.form_widget.unsupervised.CalibrationParam[key][1] = calib_result_dict[key][2]
            self.form_widget.unsupervised.CalibrationParam[key][5] = calib_result_dict[key][0]

        df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.form_widget.importing,
                                                                                           self.radio_label_layout.using_gnss_motion,
                                                                                           self.form_widget.unsupervised.PARM_LIDAR,
                                                                                           self.form_widget.unsupervised.CalibrationParam,
                                                                                           self.form_widget.importing_tab.limit_time_layout.start_time,
                                                                                           self.form_widget.importing_tab.limit_time_layout.end_time)

        self.form_widget.ViewPointCloud(df_info, accum_pointcloud, PARM_LIDAR)

    def EndCalibration(self):
        zrp_result = self.form_widget.GetZRPCalibList(self.zrp_result_labels, self.manual_zrp_calib_result)
        self.calib_result = [self.form_widget.unsupervised.calib_x, self.form_widget.unsupervised.calib_y,
                             zrp_result[0], zrp_result[1], zrp_result[2],
                             self.form_widget.unsupervised.calib_yaw]
        self.PARM_LIDAR = self.form_widget.unsupervised.PARM_LIDAR

        calib_result_dict = self.form_widget.GetZRPCalibDict(self.zrp_result_labels, self.manual_zrp_calib_result)
        for key in self.form_widget.unsupervised.CalibrationParam:
            self.form_widget.unsupervised.CalibrationParam[key][0] = calib_result_dict[key][1]
            self.form_widget.unsupervised.CalibrationParam[key][1] = calib_result_dict[key][2]
            self.form_widget.unsupervised.CalibrationParam[key][5] = calib_result_dict[key][0]

        self.progress_status = CONST_IDLE
        self.form_widget.unsupervised.complete_calibration = True

        df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.form_widget.importing,
                                                                                           self.radio_label_layout.using_gnss_motion,
                                                                                           self.form_widget.unsupervised.PARM_LIDAR,
                                                                                           self.form_widget.unsupervised.CalibrationParam,
                                                                                           self.form_widget.importing_tab.limit_time_layout.start_time,
                                                                                           self.form_widget.importing_tab.limit_time_layout.end_time)

        # Unsupervised tab

        ## Set 'Result Calibration Data'
        for idxSensor in PARM_LIDAR['CheckedSensorList']:
            self.xyyaw_result_labels[idxSensor].label_edit1.setText(format(self.form_widget.unsupervised.CalibrationParam[idxSensor][3], ".4f"))
            self.xyyaw_result_labels[idxSensor].label_edit2.setText(format(self.form_widget.unsupervised.CalibrationParam[idxSensor][4], ".4f"))
            self.xyyaw_result_labels[idxSensor].label_edit3.setText(format(self.form_widget.unsupervised.CalibrationParam[idxSensor][2] * 180 / math.pi, ".4f"))

        ## Plot 'Result Data'
        self.VTKInit(is_default=False)
        self.form_widget.DisplayLiDAR(self.calib_result, PARM_LIDAR, self.ren, self.iren, self.renWin, self.widget, self.is_single)

        ## Plot 'Result Graph''
        self.result_graph_ax.clear()
        self.form_widget.ViewPointCloud(df_info, accum_pointcloud, PARM_LIDAR, self.result_graph_ax, self.result_graph_canvas)

        # Evaluation tab

        for idxSensor in self.PARM_LIDAR['CheckedSensorList']:
            if (self.is_single == False) or ((self.is_single == True) and idxSensor == self.form_widget.config.PARM_LIDAR['SingleSensor']) :
                self.form_widget.evaluation_tab.xyyaw_result_labels[idxSensor].button_group.button(CONST_UNSUPERVISED).setChecked(True)
                self.form_widget.evaluation_tab.xyyaw_result_labels[idxSensor].prev_checkID = CONST_UNSUPERVISED
                self.form_widget.evaluation_tab.xyyaw_result_labels[idxSensor].spinbox1.setValue(self.form_widget.unsupervised.CalibrationParam[idxSensor][3])
                self.form_widget.evaluation_tab.xyyaw_result_labels[idxSensor].spinbox2.setValue(self.form_widget.unsupervised.CalibrationParam[idxSensor][4])
                self.form_widget.evaluation_tab.xyyaw_result_labels[idxSensor].spinbox3.setValue(self.form_widget.unsupervised.CalibrationParam[idxSensor][2] * 180 / math.pi)
                self.form_widget.evaluation_tab.custom_calibration_param[idxSensor] = copy.deepcopy(self.form_widget.unsupervised.CalibrationParam[idxSensor])

    def CopyList(self, source, target):
        keys = list(source.keys())
        values = list(source.values())

        for i in range(len(keys)):
            target[keys[i]] = values[i].copy()

    def Clear(self):
        # Reset unsupervised values
        try:
            self.form_widget.unsupervised.PARM_LIDAR.clear()
        except:
            pass
        self.form_widget.unsupervised.calib_x.clear()
        self.form_widget.unsupervised.calib_y.clear()
        self.form_widget.unsupervised.calib_yaw.clear()
        self.form_widget.unsupervised.CalibrationParam.clear()

        self.calib_x = []
        self.calib_y = []
        self.calib_z = []
        self.calib_r = []
        self.calib_p = []
        self.calib_yaw = []
        self.PARM_LIDAR = []

        # Clear calib result
        self.calib_result.clear()
        self.manual_zrp_calib_result.clear()

        # reset unsupervised calibration result
        self.form_widget.ResetResultsLabel(CONST_UNEDITABLE_LABEL,
                                           self.form_widget.config.PARM_LIDAR,
                                           self.scroll_box.layout,
                                           self.xyyaw_result_labels,
                                           self.form_widget.unsupervised.CalibrationParam)

        # reset unsupervised initial value of handeye
        self.edit_handeye_calibration_parm.clear()
        self.form_widget.ResetResultsLabel(CONST_EDITABLE_LABEL2,
                                           self.form_widget.config.PARM_LIDAR,
                                           self.optimization_initial_value_tab.scroll_box1.layout,
                                           self.handeye_result_labels,
                                           self.edit_handeye_calibration_parm)

        # reset unsupervised initial value of custom
        self.form_widget.config.CalibrationParam.clear()
        self.form_widget.ResetResultsLabel(CONST_EDITABLE_LABEL2,
                                           self.form_widget.config.PARM_LIDAR,
                                           self.optimization_initial_value_tab.scroll_box2.layout,
                                           self.user_define_initial_labels,
                                           self.form_widget.config.CalibrationParam)


        # Clear HMI
        self.VTKInit(is_default=True)
        self.PlotClear(self.result_graph_ax, self.result_graph_canvas)
        self.text_edit.clear()

        self.progress_status = CONST_IDLE

        self.radio_label_layout.button_group.button(1).setChecked(True)
        self.radio_label_layout.RadioButton()

        self.select_lidar_num_layout.button_group.button(1).setChecked(True)
        self.select_lidar_num_layout.RadioButton()

class EvaluationTab(QWidget):
    def __init__(self, form_widget):
        super().__init__()
        self.form_widget = form_widget
        self.evaluation_status = CONST_IDLE
        self.custom_calibration_param = {}

        self.xyyaw_result_labels = {}
        self.eval_lidar = {}
        self.eval_calibration_param = {}
        self.eval_calib_x = []
        self.eval_calib_y = []
        self.eval_calib_yaw = []
        self.calib_result = []

        self.zrp_result_labels = {}
        self.manual_zrp_calib_result = {}

        self.initUi()

    def initUi(self):
        main_vbox = QVBoxLayout()

        main_vbox.addLayout(self.UserInterface())
        main_vbox.addLayout(self.EvaluationResult())

        self.setLayout(main_vbox)

    ## Layout
    def UserInterface(self):
        hbox = QHBoxLayout()

        hbox.addWidget(self.UserInterface_SelectMethod_Groupbox())

        vbox = QVBoxLayout()
        sub_hbox = QHBoxLayout()
        sub_hbox.addWidget(self.UserInterface_SetConfiguration_Groupbox())
        sub_hbox.addWidget(self.UserInterface_LimitTime_Groupbox())
        vbox.addLayout(sub_hbox)

        label = QLabel('[ Evaluation Progress ]')
        vbox.addWidget(label)

        self.pbar = QProgressBar(self)
        vbox.addWidget(self.pbar)

        self.text_edit = QTextEdit()
        vbox.addWidget(self.text_edit)

        hbox.addLayout(vbox)

        return hbox

    def EvaluationResult(self):
        hbox = QHBoxLayout()

        hbox.addWidget(self.EvaluationResult_ResultGraph_Groupbox())
        hbox.addWidget(self.EvaluationResult_LidarPosition_Groupbox())
        hbox.addWidget(self.EvaluationResult_RMSE_Groupbox())

        return hbox

    ## Groupbox

    def UserInterface_SelectMethod_Groupbox(self):
        groupbox = QGroupBox('Select Evaluation Target')
        vbox = QVBoxLayout()

        height_roll_pitch_param_label = QLabel('[ Select Z, Roll, Pitch ]', self)
        vbox.addWidget(height_roll_pitch_param_label)

        hbox = QHBoxLayout()
        label = QLabel()
        hbox.addWidget(label, 10)
        label = QLabel('Automatic')
        hbox.addWidget(label, 10)
        label = QLabel('Manual')
        hbox.addWidget(label, 10)
        label = QLabel('z [m]')
        hbox.addWidget(label, 10)
        label = QLabel('roll [deg]')
        hbox.addWidget(label, 10)
        label = QLabel('pitch [deg]')
        hbox.addWidget(label, 10)
        vbox.addLayout(hbox)

        self.zrp_scroll_box = ScrollAreaV()
        vbox.addWidget(self.zrp_scroll_box)

        height_roll_pitch_param_label = QLabel('[ Select X, Y, Yaw ]', self)
        vbox.addWidget(height_roll_pitch_param_label)

        hbox = QHBoxLayout()
        label = QLabel()
        hbox.addWidget(label, 10)
        label = QLabel('Custom')
        hbox.addWidget(label, 10)
        label = QLabel('HandEye')
        hbox.addWidget(label, 10)
        label = QLabel('Unsupervised')
        hbox.addWidget(label, 10)
        label = QLabel('x [m]')
        hbox.addWidget(label, 10)
        label = QLabel('y [m]')
        hbox.addWidget(label, 10)
        label = QLabel('yaw [deg]')
        hbox.addWidget(label, 10)
        vbox.addLayout(hbox)

        self.scroll_box = ScrollAreaV()
        vbox.addWidget(self.scroll_box)

        groupbox.setLayout(vbox)
        return groupbox

    def UserInterface_SetConfiguration_Groupbox(self):
        groupbox = QGroupBox('Set Configuration')
        vbox = QVBoxLayout()

        buttons = {0:'GNSS Data', 1:'Motion Data'}
        self.radio_label_layout = element.RadioLabelLayout(CONST_EVAL_SELECT_LIDAR, 'Used Data', buttons, self.form_widget)
        vbox.addLayout(self.radio_label_layout)

        self.sampling_interval_layout = element.SpinBoxLabelLayout('Eval Sampling Interval [Count]', self.form_widget)
        vbox.addLayout(self.sampling_interval_layout)

        self.distance_interval_layout = element.DoubleSpinBoxLabelLayout(CONST_EVAL_DISTANCE_INTERVAL, 'Eval Distance Interval [m]', self.form_widget)
        vbox.addLayout(self.distance_interval_layout)

        self.time_speed_threshold_layout = element.DoubleSpinBoxLabelLayout(CONST_EVAL_VEHICLE_MINIMUM_SPEED, 'Eval Vehicle Minimum Speed [km/h]', self.form_widget)
        vbox.addLayout(self.time_speed_threshold_layout)

        groupbox.setLayout(vbox)
        return groupbox

    def UserInterface_LimitTime_Groupbox(self):
        groupbox = QGroupBox('Limit Time')
        vbox = QVBoxLayout()

        self.limit_time_layout = element.SlideLabelLayouts(self.form_widget)
        vbox.addLayout(self.limit_time_layout)

        hbox = QHBoxLayout()
        btn = QPushButton('Start')
        btn.clicked.connect(self.StartBtn)
        hbox.addWidget(btn)

        btn = QPushButton('Stop')
        btn.clicked.connect(self.StopBtn)
        hbox.addWidget(btn)
        vbox.addLayout(hbox)

        groupbox.setLayout(vbox)
        return groupbox

    def EvaluationResult_ResultGraph_Groupbox(self):
        groupbox = QGroupBox('HD Map Result of Calibration')
        vbox = QVBoxLayout()

        self.eval_graph_fig = plt.figure()
        self.eval_graph_canvas = FigureCanvas(self.eval_graph_fig)
        self.eval_graph_ax = self.eval_graph_fig.add_subplot(1, 1, 1)
        self.eval_graph_ax.grid()
        self.eval_graph_canvas.draw()
        vbox.addWidget(self.eval_graph_canvas)

        btn = QPushButton('View')
        btn.clicked.connect(self.ViewPointCloud)
        vbox.addWidget(btn)

        groupbox.setLayout(vbox)
        return groupbox

    def EvaluationResult_LidarPosition_Groupbox(self):
        groupbox = QGroupBox('Lidar Position Result of Calibration')
        vbox = QVBoxLayout()

        self.vtkWidget = QVTKRenderWindowInteractor()
        vbox.addWidget(self.vtkWidget)
        self.VTKInit(is_default=True)

        btn = QPushButton('View')
        btn.clicked.connect(self.ViewLiDAR)
        vbox.addWidget(btn)

        groupbox.setLayout(vbox)
        return groupbox

    def EvaluationResult_RMSE_Groupbox(self):
        groupbox = QGroupBox('RMSE')
        vbox = QVBoxLayout()

        self.eval_rmse_fig = plt.figure()
        self.eval_rmse_canvas = FigureCanvas(self.eval_rmse_fig)

        self.eval_rmse_xy_ax = self.eval_rmse_fig.add_subplot(2, 1, 1)
        self.eval_rmse_xy_ax.set_xlabel('LiDAR List')
        self.eval_rmse_xy_ax.set_ylabel('RMSE [m]')
        self.eval_rmse_xy_ax.set_title('RMSE x, y')

        self.eval_rmse_yaw_ax = self.eval_rmse_fig.add_subplot(2, 1, 2)
        self.eval_rmse_yaw_ax.set_xlabel('LiDAR List')
        self.eval_rmse_yaw_ax.set_ylabel('RMSE [deg]')
        self.eval_rmse_yaw_ax.set_title('RMSE yaw')

        self.eval_rmse_fig.tight_layout()
        self.eval_rmse_canvas.draw()
        vbox.addWidget(self.eval_rmse_canvas)

        groupbox.setLayout(vbox)
        return groupbox

    # Callback
    def ViewLiDAR(self):
        self.dialog = element.NewWindow('ViewLiDAR', self.calib_result, self.form_widget)

    def ViewPointCloud(self):
        eval_df_info = copy.deepcopy(self.form_widget.evaluation.df_info)
        eval_Map = copy.deepcopy(self.form_widget.evaluation.Map)

        self.form_widget.ViewPointCloud(eval_df_info, eval_Map, self.eval_lidar)

    def StartBtn(self):
        if self.form_widget.config_tab.is_lidar_num_changed == True:
            self.form_widget.ErrorPopUp('Please import after changing lidar number')
            return False

        is_handeye_selected = False
        is_optimization_selected = False
        for idxSensor in self.eval_lidar['CheckedSensorList']:
            method = self.xyyaw_result_labels[idxSensor].button_group.checkedId()
            if method == CONST_HANDEYE:
                is_handeye_selected = True
            elif method == CONST_UNSUPERVISED:
                is_optimization_selected = True
        if not self.form_widget.handeye.complete_calibration:
            if is_handeye_selected:
                self.form_widget.ErrorPopUp('Please complete the HandEye calibration')
                return False
        elif not self.form_widget.unsupervised.complete_calibration:
            if is_optimization_selected:
                self.form_widget.ErrorPopUp('Please complete the Opimization calibration')
                return False

        for idxSensor in self.eval_lidar['CheckedSensorList']:
            if self.form_widget.importing.PointCloudSensorList.get(idxSensor) is None:
                self.form_widget.ErrorPopUp('Import pointcloud {}'.format(idxSensor))
                return False
        if self.evaluation_status is not CONST_IDLE:
            return False
        self.evaluation_status = CONST_PLAY


        self.VTKInit(is_default=True)
        self.PlotClear(self.eval_graph_ax, self.eval_graph_canvas)
        self.PlotClear(self.eval_rmse_yaw_ax, self.eval_rmse_canvas)
        self.PlotClear(self.eval_rmse_xy_ax, self.eval_rmse_canvas)

        self.eval_calibration_param.clear()
        self.eval_calib_x.clear()
        self.eval_calib_y.clear()
        self.eval_calib_yaw.clear()
        for idxSensor in self.eval_lidar['CheckedSensorList']:
            result_param = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            method = self.xyyaw_result_labels[idxSensor].button_group.checkedId()
            if method == CONST_HANDEYE:
                result_param[2] = self.form_widget.handeye.CalibrationParam[idxSensor][2]
                result_param[3] = self.form_widget.handeye.CalibrationParam[idxSensor][3]
                result_param[4] = self.form_widget.handeye.CalibrationParam[idxSensor][4]
                self.eval_calib_x.append(self.form_widget.handeye.CalibrationParam[idxSensor][3])
                self.eval_calib_y.append(self.form_widget.handeye.CalibrationParam[idxSensor][4])
                self.eval_calib_yaw.append(self.form_widget.handeye.CalibrationParam[idxSensor][2] * 180/math.pi)
            elif method == CONST_UNSUPERVISED:
                result_param[2] = self.form_widget.unsupervised.CalibrationParam[idxSensor][2]
                result_param[3] = self.form_widget.unsupervised.CalibrationParam[idxSensor][3]
                result_param[4] = self.form_widget.unsupervised.CalibrationParam[idxSensor][4]
                self.eval_calib_x.append(self.form_widget.unsupervised.CalibrationParam[idxSensor][3])
                self.eval_calib_y.append(self.form_widget.unsupervised.CalibrationParam[idxSensor][4])
                self.eval_calib_yaw.append(self.form_widget.unsupervised.CalibrationParam[idxSensor][2] * 180 / math.pi)
            elif method == CONST_CUSTOM:
                result_param[2] = self.custom_calibration_param[idxSensor][2]
                result_param[3] = self.custom_calibration_param[idxSensor][3]
                result_param[4] = self.custom_calibration_param[idxSensor][4]
                self.eval_calib_x.append(self.custom_calibration_param[idxSensor][3])
                self.eval_calib_y.append(self.custom_calibration_param[idxSensor][4])
                self.eval_calib_yaw.append(self.custom_calibration_param[idxSensor][2] * 180 / math.pi)

            method = self.zrp_result_labels[idxSensor].button_group.checkedId()
            if method == CONST_AUTOMATIC:
                if self.form_widget.zrollpitch_tab.calib_result.get(idxSensor) is None:
                    self.form_widget.zrollpitch_tab.calib_result[idxSensor] = [0.0, 0.0, 0.0]
                calib_result = self.form_widget.zrollpitch_tab.calib_result[idxSensor]

                result_param[0] = calib_result[1] * math.pi / 180
                result_param[1] = calib_result[2] * math.pi / 180
                result_param[5] = calib_result[0]
            elif method == CONST_MANUAL:
                result_param[0] = self.manual_zrp_calib_result[idxSensor][1] * math.pi / 180
                result_param[1] = self.manual_zrp_calib_result[idxSensor][2] * math.pi / 180
                result_param[5] = self.manual_zrp_calib_result[idxSensor][0]

            self.eval_calibration_param[idxSensor] = copy.deepcopy(result_param)

        self.StartEvaluation(self.limit_time_layout.start_time,
                             self.limit_time_layout.end_time,
                             self.eval_lidar,
                             self.eval_calibration_param,
                             self.form_widget.GetZRPCalibDict(self.zrp_result_labels, self.manual_zrp_calib_result),
                             self.form_widget.config.PARM_EV['DistanceInterval'],
                             [self.text_edit.clear, self.pbar.reset],
                             self.text_edit.append,
                             self.pbar.setValue,
                             self.EndEvaluation)

    def StopBtn(self):
        self.form_widget.thread.toggle_status()
        self.evaluation_status = CONST_IDLE

    def StartEvaluation(self, start_time, end_time, sensor_list, calibration_param, zrp_calib, dist_interval, targets_clear, text_edit_callback, progress_callback, end_callback):
        self.form_widget.thread._status = True

        self.form_widget.thread.SetFunc(self.form_widget.evaluation.Evaluation,
                                        [start_time, end_time, sensor_list, calibration_param, self.radio_label_layout.using_gnss_motion, zrp_calib, dist_interval])
        try:
            self.form_widget.thread.change_value.disconnect()
        except:
            pass
        try:
            self.form_widget.thread.iteration_percentage.disconnect()
        except:
            pass
        try:
            self.form_widget.thread.end.disconnect()
        except:
            pass
        try:
            self.form_widget.thread.emit_string.disconnect()
        except:
            pass

        for target_clear in targets_clear:
            target_clear()

        self.form_widget.thread.emit_string.connect(text_edit_callback)
        self.form_widget.thread.change_value.connect(progress_callback)
        self.form_widget.thread.end.connect(end_callback)
        self.form_widget.thread.start()

    def EndEvaluation(self):
        self.evaluation_status = CONST_IDLE

        ## Plot 'Result Graph'
        self.eval_graph_ax.clear()
        self.form_widget.ViewPointCloud(self.form_widget.evaluation.df_info,
                               self.form_widget.evaluation.Map,
                               self.form_widget.evaluation.PARM_LIDAR,
                               self.eval_graph_ax,
                               self.eval_graph_canvas)

        ## Plot 'Result Data'
        zrp_result = self.form_widget.GetZRPCalibList(self.zrp_result_labels, self.manual_zrp_calib_result)
        self.calib_result = [self.eval_calib_x, self.eval_calib_y,
                             zrp_result[0], zrp_result[1], zrp_result[2],
                             self.eval_calib_yaw]

        ## Plot 'Result Data'
        self.VTKInit(is_default=False)
        self.form_widget.DisplayLiDAR(self.calib_result, self.form_widget.config.PARM_LIDAR, self.ren, self.iren, self.renWin, self.widget)


        ## Plot 'Result RMSE'
        self.eval_rmse_xy_ax.clear()
        self.eval_rmse_yaw_ax.clear()
        self.form_widget.ViewRMSE(self.form_widget.evaluation.rmse_x,
                         self.form_widget.evaluation.rmse_y,
                         self.form_widget.evaluation.rmse_yaw,
                         self.form_widget.evaluation.lidarlist,
                         self.form_widget.evaluation.PARM_LIDAR,
                         self.eval_rmse_xy_ax,
                         self.eval_rmse_yaw_ax,
                         self.eval_rmse_canvas)

        fail_str = ''
        for fail in self.form_widget.evaluation.fail_list:
            fail_str += ('XYZRGB_' + str(fail) + ' ')

        if len(self.form_widget.evaluation.fail_list) != 0:
            self.form_widget.ErrorPopUp('Failed Evaluation: ' + fail_str)
        
        print('End evaluation')

    def VTKInit(self, is_default):
        vtk.vtkObject.GlobalWarningDisplayOff()
        colors = vtk.vtkNamedColors()

        self.ren = vtk.vtkRenderer()

        self.renWin = self.vtkWidget.GetRenderWindow()
        self.renWin.AddRenderer(self.ren)

        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        self.widget = vtk.vtkOrientationMarkerWidget()

        self.iren.SetRenderWindow(self.renWin)

        self.ren.SetBackground(colors.GetColor3d("white"))
        self.iren.Initialize()

        if is_default:
            vtk_lidar_calib.GetAxis(self.iren, self.widget)
            self.ren.AddActor(vtk_lidar_calib.GetGridActor())
            self.ren.AddActor(vtk_lidar_calib.GetVehicleActor())

            colors = vtk.vtkNamedColors()
            self.ren.SetBackground(colors.GetColor3d('white'))

            self.ren.ResetCamera()
            self.renWin.Render()

            self.iren.Start()

    def PlotClear(self, ax, canvas):
        # remove plots
        try:
            ax.clear()
            canvas.draw()
        except:
            pass

    def Clear(self):
        # Reset evaluation values
        try:
            self.form_widget.evaluation.df_info.clear()
            self.Map.clear()
            self.form_widget.evaluation.rmse_x.clear()
            self.form_widget.evaluation.rmse_y.clear()
            self.form_widget.evaluation.rmse_yaw.clear()
            self.form_widget.evaluation.lidarlist.clear()
            self.form_widget.evaluation.PARM_LIDAR.clear()
        except:
            pass

        # Clear calib result
        self.custom_calibration_param.clear()
        self.eval_calibration_param.clear()
        self.eval_calib_x.clear()
        self.eval_calib_y.clear()
        self.eval_calib_yaw.clear()
        self.calib_result.clear()
        self.manual_zrp_calib_result.clear()

        # reset evaluation select method
        self.form_widget.ResetResultsLabel(CONST_EVAULATION_LABEL,
                               self.form_widget.config.PARM_LIDAR,
                               self.scroll_box.layout,
                               self.xyyaw_result_labels,
                               self.form_widget.handeye.CalibrationParam)

        # Clear HMI
        self.VTKInit(is_default=True)
        self.PlotClear(self.eval_graph_ax, self.eval_graph_canvas)
        self.PlotClear(self.eval_rmse_yaw_ax, self.eval_rmse_canvas)
        self.PlotClear(self.eval_rmse_xy_ax, self.eval_rmse_canvas)
        self.text_edit.clear()
        self.pbar.reset()

        self.evaluation_status = CONST_IDLE

        self.radio_label_layout.button_group.button(1).setChecked(True)
        self.radio_label_layout.RadioButton()

class MyApp(QMainWindow):
    def __init__(self, parent=None):
        super(MyApp, self).__init__(parent)
        self.form_widget = FormWidget(self)
        self.setWindowIcon(QIcon(self.form_widget.config.PATH['Image'] + 'exe_icon.ico'))

        self.InitUi()

    def InitUi(self):
        ### Define menuBar File tap and that's contents

        self.setCentralWidget(self.form_widget)

        menubar = self.menuBar()
        menubar.setNativeMenuBar(False)
        filemenu = menubar.addMenu('&File')

        new_action = QAction('New', self)
        new_action.setShortcut('Ctrl+N')
        new_action.setStatusTip('Open new ini file')
        new_action.triggered.connect(self.OpenNewFile)
        filemenu.addAction(new_action)

        open_action = QAction('Open', self)
        open_action.setShortcut('Ctrl+O')
        open_action.setStatusTip('Open ini file')
        open_action.triggered.connect(self.OpenExistFile)
        filemenu.addAction(open_action)

        save_action = QAction('Save', self)
        save_action.setShortcut('Ctrl+S')
        save_action.setStatusTip('Save ini file')
        save_action.triggered.connect(self.SaveIniFile)
        filemenu.addAction(save_action)

        save_as_action = QAction('Save as', self)
        save_as_action.setShortcut('Ctrl+Shift+S')
        save_as_action.setStatusTip('Save as different ini file')
        save_as_action.triggered.connect(self.SaveAsInitFile)
        filemenu.addAction(save_as_action)

        self.statusBar()

        self.setWindowTitle('Lidar Calibration Tools')
        self.showMaximized()

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def OpenExistFile(self): # Ctrl+O
        if self.form_widget.value_changed:
            reply = QMessageBox.question(self, 'Open Exist File', 'Do you want to save?',
                                         QMessageBox.Save | QMessageBox.No | QMessageBox.Cancel, QMessageBox.Save)
            if reply == QMessageBox.Save:
                fname = self.SaveDialog()
                if fname[0]:
                    self.form_widget.config.configuration_file = fname[0]
                    self.form_widget.config.WriteDefaultFileBase(fname[0])
                    self.SaveIniFile()
                    print('Save '+ str(fname[0]))
                else:
                    print('Cancel Open ini File')
                    return False
            elif reply == QMessageBox.Cancel:
                print('Cancel Open ini File')
                return False

        widget = QWidget()
        fname = QFileDialog.getOpenFileName(widget, 'Open file', self.form_widget.config.PATH['Configuration'], "Configuration file (*.ini)") # exist ini file path

        if fname[0]:
            self.form_widget.config.configuration_file = fname[0]
            self.form_widget.config.InitConfiguration()
            self.form_widget.SetConfiguration()
            print('Open '+ str(fname[0]))

    def OpenNewFile(self): # Ctrl+N
        ### Open new file when some parameter change
        if self.form_widget.value_changed:
            # Generate message box to save current setting. The option is save, no and cancel. Default is save
            reply = QMessageBox.question(self, 'Open New File', 'Do you want to save?',
                                         QMessageBox.Save | QMessageBox.No | QMessageBox.Cancel, QMessageBox.Save)
            if reply == QMessageBox.Save:
                fname = self.SaveDialog() # path of saving directory
                if fname[0]:
                    self.form_widget.config.configuration_file = fname[0]
                    self.form_widget.config.WriteDefaultFileBase(fname[0])
                    self.SaveIniFile() # save parameters
                    print('Save Current ini File')
                else:
                    print('Cancel Save ini File')
                    return False
            elif reply == QMessageBox.Cancel:
                print('Cancel Save ini File')
                return False

        ### clear parameters
        # self.ClearForWidget()
        self.form_widget.config.WriteDefaultFile()
        self.form_widget.config.InitConfiguration()
        self.form_widget.SetConfiguration()
        print('Open New ini File')

    def SaveIniFile(self):
        checked_sensor_list = ''
        for sensor_index in self.form_widget.config.PARM_LIDAR['CheckedSensorList']:
            if checked_sensor_list == '':
                checked_sensor_list = str(sensor_index)
            else:
                checked_sensor_list = checked_sensor_list + ' ' + str(sensor_index)

        sensor_list = ''
        for sensor_index in self.form_widget.config.PARM_LIDAR['SensorList']:
            if sensor_list == '':
                sensor_list = str(sensor_index)
            else:
                sensor_list = sensor_list + ' ' + str(sensor_index)

        is_pointcloud = False
        is_zrollpitch = False
        is_validation = False
        is_handeye = False
        is_single_optimization = False
        is_multi_optimization = False
        is_import = False
        is_evaluation = False
        is_path = False

        for line in fileinput.input(self.form_widget.config.configuration_file, inplace=True):
            if 'SingleSensor' in line:
                line = line.replace(line, 'SingleSensor = ' + str(self.form_widget.config.PARM_LIDAR['SingleSensor']) + '\n')
            if 'PrincipalSensor' in line:
                line = line.replace(line, 'PrincipalSensor = ' + str(self.form_widget.config.PARM_LIDAR['PrincipalSensor']) + '\n')
            elif 'CheckedSensorList' in line:
                line = line.replace(line, 'CheckedSensorList = ' + str(checked_sensor_list) + '\n')
            elif 'SensorList' in line:
                line = line.replace(line, 'SensorList = ' + str(sensor_list) + '\n')
            elif '[PointCloud]' in line:
                is_pointcloud = True
            elif 'MinThresholdDist_m' in line:
                line = line.replace(line, 'MinThresholdDist_m = ' + str(self.form_widget.config.PARM_PC['MinThresholdDist_m']) + '\n')
            elif 'MaxThresholdDist_m' in line:
                line = line.replace(line, 'MaxThresholdDist_m = ' + str(self.form_widget.config.PARM_PC['MaxThresholdDist_m']) + '\n')
            elif ('MinThresholdX_m') in line and is_pointcloud:
                line = line.replace(line, 'MinThresholdX_m = ' + str(self.form_widget.config.PARM_PC['MinThresholdX_m']) + '\n')
            elif ('MaxThresholdX_m') in line and is_pointcloud:
                line = line.replace(line, 'MaxThresholdX_m = ' + str(self.form_widget.config.PARM_PC['MaxThresholdX_m']) + '\n')
            elif ('MinThresholdY_m') in line and is_pointcloud:
                line = line.replace(line, 'MinThresholdY_m = ' + str(self.form_widget.config.PARM_PC['MinThresholdY_m']) + '\n')
            elif ('MaxThresholdY_m') in line and is_pointcloud:
                line = line.replace(line, 'MaxThresholdY_m = ' + str(self.form_widget.config.PARM_PC['MaxThresholdY_m']) + '\n')
            elif 'MinThresholdZ_m' in line:
                line = line.replace(line, 'MinThresholdZ_m = ' + str(self.form_widget.config.PARM_PC['MinThresholdZ_m']) + '\n')
            elif 'MaxThresholdZ_m' in line:
                line = line.replace(line, 'MaxThresholdZ_m = ' + str(self.form_widget.config.PARM_PC['MaxThresholdZ_m']) + '\n')
            elif ('SamplingInterval' in line) and (is_multi_optimization == False):
                line = line.replace(line, 'SamplingInterval = ' + str(self.form_widget.config.PARM_IM['SamplingInterval']) + '\n')
            elif ('VehicleSpeedThreshold' in line) and (is_multi_optimization == False):
                line = line.replace(line, 'VehicleSpeedThreshold = ' + str(self.form_widget.config.PARM_IM['VehicleSpeedThreshold']) + '\n')
            elif '[Import]' in line:
                is_import = True
            elif ('SamplingInterval' in line) and is_import:
                line = line.replace(line, 'SamplingInterval = ' + str(self.form_widget.config.PARM_IM['SamplingInterval']) + '\n')
            elif ('VehicleSpeedThreshold' in line) and is_import:
                line = line.replace(line, 'VehicleSpeedThreshold = ' + str(self.form_widget.config.PARM_IM['VehicleSpeedThreshold']) + '\n')
            elif '[ZRPCalibration]' in line:
                is_zrollpitch = True
            elif ('MaxDistanceX_m') in line and is_zrollpitch:
                line = line.replace(line, 'MaxDistanceX_m = ' + str(self.form_widget.config.PARM_ZRP['MaxDistanceX_m']) + '\n')
            elif ('MinDistanceX_m') in line and is_zrollpitch:
                line = line.replace(line, 'MinDistanceX_m = ' + str(self.form_widget.config.PARM_ZRP['MinDistanceX_m']) + '\n')
            elif ('MaxDistanceY_m') in line and is_zrollpitch:
                line = line.replace(line, 'MaxDistanceY_m = ' + str(self.form_widget.config.PARM_ZRP['MaxDistanceY_m']) + '\n')
            elif 'MinDistanceY_m' in line and is_zrollpitch:
                line = line.replace(line, 'MinDistanceY_m = ' + str(self.form_widget.config.PARM_ZRP['MinDistanceY_m']) + '\n')
            elif 'MaxThresholdZ_m' in line and is_zrollpitch:
                line = line.replace(line, 'MaxThresholdZ_m = ' + str(self.form_widget.config.PARM_ZRP['MaxThresholdZ_m']) + '\n')
            elif 'MinDistanceZ_m' in line and is_zrollpitch:
                line = line.replace(line, 'MinDistanceZ_m = ' + str(self.form_widget.config.PARM_ZRP['MinDistanceZ_m']) + '\n')
            elif '[Validation]' in line:
                is_validation = True
            elif ('MaximumIteration' in line) and is_validation:
                line = line.replace(line, 'MaximumIteration = ' + str(self.form_widget.config.PARM_DV['MaximumIteration']) + '\n')
            elif ('Tolerance' in line) and is_validation:
                line = line.replace(line, 'Tolerance = ' + str(self.form_widget.config.PARM_DV['Tolerance']) + '\n')
            elif ('OutlierDistance_m' in line) and is_validation:
                line = line.replace(line, 'OutlierDistance_m = ' + str(self.form_widget.config.PARM_DV['OutlierDistance_m']) + '\n')
            elif ('FilterHeadingThreshold' in line) and is_validation:
                line = line.replace(line, 'FilterHeadingThreshold = ' + str(self.form_widget.config.PARM_DV['FilterHeadingThreshold']) + '\n')
            elif ('FilterDistanceThreshold' in line) and is_validation:
                line = line.replace(line, 'FilterDistanceThreshold = ' + str(self.form_widget.config.PARM_DV['FilterDistanceThreshold']) + '\n')
            elif '[Handeye]' in line:
                is_handeye = True
            elif ('MaximumIteration' in line) and is_handeye:
                line = line.replace(line, 'MaximumIteration = ' + str(self.form_widget.config.PARM_HE['MaximumIteration']) + '\n')
            elif ('Tolerance' in line) and is_handeye:
                line = line.replace(line, 'Tolerance = ' + str(self.form_widget.config.PARM_HE['Tolerance']) + '\n')
            elif ('OutlierDistance_m' in line) and is_handeye:
                line = line.replace(line, 'OutlierDistance_m = ' + str(self.form_widget.config.PARM_HE['OutlierDistance_m']) + '\n')
            elif ('FilterHeadingThreshold' in line) and is_handeye:
                line = line.replace(line, 'FilterHeadingThreshold = ' + str(self.form_widget.config.PARM_HE['FilterHeadingThreshold']) + '\n')
            elif ('FilterDistanceThreshold' in line) and is_handeye:
                line = line.replace(line, 'FilterDistanceThreshold = ' + str(self.form_widget.config.PARM_HE['FilterDistanceThreshold']) + '\n')
            elif '[SingleOptimization]' in line:
                is_single_optimization = True
                is_multi_optimization = False
            elif ('PointSamplingRatio' in line) and is_single_optimization:
                line = line.replace(line, 'PointSamplingRatio = ' + str(self.form_widget.config.PARM_SO['PointSamplingRatio']) + '\n')
            elif ('NumPointsPlaneModeling') in line and is_single_optimization:
                line = line.replace(line, 'NumPointsPlaneModeling = ' + str(self.form_widget.config.PARM_SO['NumPointsPlaneModeling']) + '\n')
            elif ('OutlierDistance_m' in line) and is_single_optimization:
                line = line.replace(line, 'OutlierDistance_m = ' + str(self.form_widget.config.PARM_SO['OutlierDistance_m']) + '\n')
            elif '[MultiOptimization]' in line:
                is_multi_optimization = True
                is_single_optimization = False
            elif ('PointSamplingRatio' in line) and is_multi_optimization:
                line = line.replace(line, 'PointSamplingRatio = ' + str(self.form_widget.config.PARM_MO['PointSamplingRatio']) + '\n')
            elif ('NumPointsPlaneModeling') in line and is_multi_optimization:
                line = line.replace(line, 'NumPointsPlaneModeling = ' + str(self.form_widget.config.PARM_MO['NumPointsPlaneModeling']) + '\n')
            elif ('OutlierDistance_m' in line) and is_multi_optimization:
                line = line.replace(line, 'OutlierDistance_m = ' + str(self.form_widget.config.PARM_MO['OutlierDistance_m']) + '\n')
            elif '[Evaluation]' in line:
                is_evaluation = True
            elif ('SamplingInterval' in line) and is_evaluation:
                line = line.replace(line, 'SamplingInterval = ' + str(self.form_widget.config.PARM_EV['SamplingInterval']) + '\n')
            elif ('DistanceInterval' in line) and is_evaluation:
                line = line.replace(line, 'DistanceInterval = ' + str(self.form_widget.config.PARM_EV['DistanceInterval']) + '\n')
            elif ('VehicleSpeedThreshold' in line) and is_evaluation:
                line = line.replace(line, 'VehicleSpeedThreshold = ' + str(self.form_widget.config.PARM_EV['VehicleSpeedThreshold']) + '\n')
            elif '[Path]' in line:
                is_path = True
            elif ('Configuration' in line) and is_path:
                line = line.replace(line, 'Configuration = ' + str(self.form_widget.config.PATH['Configuration'] + '\n'))
            elif ('LoggingFile' in line) and is_path:
                line = line.replace(line, 'LoggingFile = ' + str(self.form_widget.importing_tab.logging_file_path_layout.path_file_str + '\n'))
            elif ('Image' in line) and is_path:
                line = line.replace(line, 'Image = ' + str(self.form_widget.config.PATH['Image'] + '\n'))
            elif ('VehicleMesh' in line) and is_path:
                line = line.replace(line, 'VehicleMesh = ' + str(self.form_widget.config.PATH['VehicleMesh'] + '\n'))
            sys.stdout.write(line)
        self.form_widget.value_changed = False

        print('Save ' + str(self.form_widget.config.configuration_file))

    def SaveAsInitFile(self):
        fname = self.SaveDialog()
        if fname[0]:
            self.form_widget.config.configuration_file = fname[0]
            self.form_widget.config.WriteDefaultFileBase(fname[0])
            self.SaveIniFile()

        self.form_widget.value_changed = False

    def closeEvent(self, e):
        if self.form_widget.value_changed:
            reply = QMessageBox.question(self, 'Window Close', 'Do you want to save your changes',
                                         QMessageBox.Save | QMessageBox.No | QMessageBox.Cancel, QMessageBox.Save)
            if reply == QMessageBox.No:
                print('Window closed')
                e.accept()
            elif reply == QMessageBox.Save:
                fname = self.SaveDialog()

                if fname[0]:
                    self.form_widget.config.configuration_file = fname[0]
                    self.form_widget.config.WriteDefaultFileBase(fname[0])
                    self.SaveIniFile()
                    e.accept()
                else:
                    e.ignore()
            elif reply == QMessageBox.Cancel:
                e.ignore()

    def CheckExtension(self, file_path_string, check_string):
        words = file_path_string.split('.')
        if not words[-1] == check_string:
            error_message = 'Please input \'' + check_string + '\' extension file'
            widget = QWidget()

            qr = widget.frameGeometry()
            cp = QDesktopWidget().availableGeometry().center()
            qr.moveCenter(cp)
            widget.move(qr.topLeft())

            QMessageBox.information(widget, 'Information', error_message)
            return False
        return True

    def GetPath(self, file):
        words = file.split('/')

        del words[-1]
        path = ''
        for word in words:
            if path == '':
                path = word
            else:
                path = path + '/' + word

        return path

    def SaveDialog(self):
        words = self.form_widget.config.configuration_file.split('/') # first thing is the current application directory. Second is common, configuration, default.ini
        file = words[-1] # default.ini
        directory = self.form_widget.config.PATH['Configuration'] # ~/common/configuration/
        default_file = directory + '/' + file
        widget = QWidget()
        fname = QFileDialog().getSaveFileName(widget, caption='Save File', directory=default_file, filter="Configuration file (*.ini)")
        return fname # return the path of saving directory

    def ClearForWidget(self):
        self.form_widget.deleteLater()
        self.form_widget = FormWidget(self)
        self.setCentralWidget(self.form_widget)

class FormWidget(QWidget):
    def __init__(self, parent):
        self.fontDB = QFontDatabase()
        self.fontDB.addApplicationFont('./MSGOTHIC.TTF')

        super(FormWidget, self).__init__(parent)
        self.resize_count = 0
        self.thread = QThread.Thread()
        self.rpz_thread = QThread.Thread()
        self.validation_thread = QThread.Thread()
        self.handeye_thread = QThread.Thread()
        self.opti_thread = QThread.Thread()

        ### setting configuration file structure each taps default path of each parameters defining in config.WriteDefaultFile() and initialize in InitConfiguration()
        self.config =           step1_1_configuration.Configuration()
        self.importing =        step1_2_import_data.Import(self.config)
        self.zrollpitch =       step2_ZRP_calibration.ZRollPitch(self.config, self.importing)
        self.datavalidation =   step3_1_XYYaw_data_validation.DataValidation(self.config, self.importing)
        self.handeye =          step3_2_XYYaw_handeye.HandEye(self.config, self.importing)
        self.unsupervised =     step3_3_XYYaw_unsupervised.Unsupervised(self.config, self.importing)
        self.evaluation =       step4_evaluation.Evaluation(self.config, self.importing)
        self.config.WriteDefaultFile()
        self.config.WriteVehicleInfoFile()
        self.config.InitConfiguration()
        self.InitMesh()

        ### Initialise configuration
        self.InitUi()

        ### Set the configuration data
        self.SetConfiguration()
        self.value_changed = False

        self.color_list = ['r', 'b', 'c', 'm', 'g', 'y']

    def InitUi(self):
        self.hbox = QHBoxLayout(self)
        self.tabs = QTabWidget(self)

        ### Initialization each taps configuring data
        self.config_tab = ConfigurationTab(self)
        self.importing_tab = ImportDataTab(self)
        self.zrollpitch_tab = ZRollPitchTab(self)
        self.datavalidation_tab = DataValidationTab(self)
        self.handeye_tab = HandEyeTab(self)
        self.unsupervised_tab = UnsupervisedTab(self)
        self.evaluation_tab = EvaluationTab(self)

        self.tabs.addTab(self.config_tab, '1.1 Configuration')
        self.tabs.setTabEnabled(CONST_CONFIG_TAB, True)

        self.tabs.addTab(self.importing_tab, '1.2. Import Data')
        self.tabs.setTabEnabled(CONST_IMPORTDATA_TAB, False)

        self.tabs.addTab(self.zrollpitch_tab, '2. Z, Roll, Pitch: Calibration')
        self.tabs.setTabEnabled(CONST_ZROLLPITCH_TAB, False)

        self.tabs.addTab(self.datavalidation_tab, '3.1. X, Y, Yaw: Data Validation')
        self.tabs.setTabEnabled(CONST_VALIDATION_TAB, False)

        self.tabs.addTab(self.handeye_tab, '3.2. X, Y, Yaw: Hand-Eye Calibration')
        self.tabs.setTabEnabled(CONST_HANDEYE_TAB, False)

        self.tabs.addTab(self.unsupervised_tab, '3.3. X, Y, Yaw: Unsupervised Calibration')
        self.tabs.setTabEnabled(CONST_UNSUPERVISED_TAB, False)

        self.tabs.addTab(self.evaluation_tab, '4. Evaluation')
        self.tabs.setTabEnabled(CONST_EVALUATION_TAB, False)

        ### Set Basic Window
        self.hbox.addWidget(self.tabs)
        self.setLayout(self.hbox)
        self.setGeometry(300, 100, 1000, 600)

    def InitMesh(self):
        ### Setting VTK path
        stl_path = self.config.PATH['LidarMesh'] + 'lidar.stl'
        stl_path = stl_path.replace('/', '\\')  # path is gui_tool.exe directory
        vtk_lidar_calib.SetLidarStlPath(stl_path)

        stl_path = self.config.PATH['GridMesh'] + 'grid.stl'
        stl_path = stl_path.replace('/', '\\')  # path is gui_tool.exe directory
        vtk_lidar_calib.SetGridStlPath(stl_path)

        print('Set mesh pathes')

    def SetConfiguration(self):
        ### Setting configuration tab
        self.config_tab.lidar_num_label_layout.spin_box.setValue(len(self.config.PARM_LIDAR['SensorList']))
        self.config_tab.select_using_sensor_list_layout.AddWidgetItem(self.config.PARM_LIDAR['SensorList'], self.config.PARM_LIDAR['CheckedSensorList'])

        text = self.config_tab.cb.currentText()
        words = text.split('.')
        self.config_tab.vehicle_info_layout.Clear(self.config.VEHICLE_INFO[words[0]])
        self.config_tab.vehicle_info_layout.BtnDisable()

        PARM_PC = self.config.PARM_PC
        self.config_tab.minimum_threshold_distance_layout.double_spin_box.setValue(PARM_PC['MinThresholdDist_m'])
        self.config_tab.maximum_threshold_istance_layout.double_spin_box.setValue(PARM_PC['MaxThresholdDist_m'])
        self.config_tab.minimum_threshold_layout_x.double_spin_box.setValue(PARM_PC['MinThresholdX_m'])
        self.config_tab.maximum_threshold_layout_x.double_spin_box.setValue(PARM_PC['MaxThresholdX_m'])
        self.config_tab.minimum_threshold_layout_y.double_spin_box.setValue(PARM_PC['MinThresholdY_m'])
        self.config_tab.maximum_threshold_layout_y.double_spin_box.setValue(PARM_PC['MaxThresholdY_m'])
        self.config_tab.minimum_threshold_layout_z.double_spin_box.setValue(PARM_PC['MinThresholdZ_m'])
        self.config_tab.maximum_threshold_layout_z.double_spin_box.setValue(PARM_PC['MaxThresholdZ_m'])

        ### Setting import tab
        PARM_IM = self.config.PARM_IM
        self.importing_tab.logging_file_path_layout.label_edit.setText(self.config.PATH['LoggingFile'])
        self.importing_tab.logging_file_path_layout.path_file_str = self.config.PATH['LoggingFile']
        self.importing_tab.sampling_interval_layout.spin_box.setValue(PARM_IM['SamplingInterval'])
        self.importing_tab.time_speed_threshold_layout.double_spin_box.setValue(PARM_IM['VehicleSpeedThreshold'])

        ### Setting ZRP tab
        PARM_ZRP = self.config.PARM_ZRP
        for idxSensor in self.config.PARM_LIDAR['SensorList']:
            if self.config.PARM_ZRP_DICT.get(idxSensor):
                continue
            self.config.PARM_ZRP_DICT[idxSensor] = {}
            self.config.PARM_ZRP_DICT[idxSensor]['MaxDistanceX_m'] = PARM_ZRP['MaxDistanceX_m']
            self.config.PARM_ZRP_DICT[idxSensor]['MinDistanceX_m'] = PARM_ZRP['MinDistanceX_m']
            self.config.PARM_ZRP_DICT[idxSensor]['MaxDistanceY_m'] = PARM_ZRP['MaxDistanceY_m']
            self.config.PARM_ZRP_DICT[idxSensor]['MinDistanceY_m'] = PARM_ZRP['MinDistanceY_m']
            self.config.PARM_ZRP_DICT[idxSensor]['MaxDistanceZ_m'] = PARM_ZRP['MaxDistanceZ_m']
            self.config.PARM_ZRP_DICT[idxSensor]['MinDistanceZ_m'] = PARM_ZRP['MinDistanceZ_m']

        self.zrollpitch_tab.maximum_x_layout.double_spin_box.setValue(PARM_ZRP['MaxDistanceX_m'])
        self.zrollpitch_tab.minimum_x_layout.double_spin_box.setValue(PARM_ZRP['MinDistanceX_m'])
        self.zrollpitch_tab.maximum_y_layout.double_spin_box.setValue(PARM_ZRP['MaxDistanceY_m'])
        self.zrollpitch_tab.minimum_y_layout.double_spin_box.setValue(PARM_ZRP['MinDistanceY_m'])
        self.zrollpitch_tab.maximum_z_layout.double_spin_box.setValue(PARM_ZRP['MaxDistanceZ_m'])
        self.zrollpitch_tab.minimum_z_layout.double_spin_box.setValue(PARM_ZRP['MinDistanceZ_m'])

        ### Setting data validation tab
        PARM_DV = self.config.PARM_DV
        self.datavalidation_tab.maximum_interation_layout.spin_box.setValue(PARM_DV['MaximumIteration'])
        self.datavalidation_tab.tolerance_layout.double_spin_box.setValue(PARM_DV['Tolerance'])
        self.datavalidation_tab.outlier_distance_layout.double_spin_box.setValue(PARM_DV['OutlierDistance_m'])
        self.datavalidation_tab.heading_threshold_layout.double_spin_box.setValue(PARM_DV['FilterHeadingThreshold'])
        self.datavalidation_tab.distance_threshold_layout.double_spin_box.setValue(PARM_DV['FilterDistanceThreshold'])

        ### Setting handeye tab
        PARM_HE = self.config.PARM_HE
        self.handeye_tab.maximum_interation_layout.spin_box.setValue(PARM_HE['MaximumIteration'])
        self.handeye_tab.tolerance_layout.double_spin_box.setValue(PARM_HE['Tolerance'])
        self.handeye_tab.outlier_distance_layout.double_spin_box.setValue(PARM_HE['OutlierDistance_m'])
        self.handeye_tab.heading_threshold_layout.double_spin_box.setValue(PARM_HE['FilterHeadingThreshold'])
        self.handeye_tab.distance_threshold_layout.double_spin_box.setValue(PARM_HE['FilterDistanceThreshold'])

        ### Setting unsupervised tab
        PARM_MO = self.config.PARM_MO
        self.unsupervised_tab.point_sampling_ratio_layout.double_spin_box.setValue(PARM_MO['PointSamplingRatio'])
        self.unsupervised_tab.num_points_plane_modeling_layout.spin_box.setValue(PARM_MO['NumPointsPlaneModeling'])
        self.unsupervised_tab.outlier_distance_layout.double_spin_box.setValue(PARM_MO['OutlierDistance_m'])

        ### Setting evaluation tab
        PARM_EV = self.config.PARM_EV
        self.evaluation_tab.eval_lidar['CheckedSensorList'] = copy.deepcopy(self.config.PARM_LIDAR['CheckedSensorList'])
        self.evaluation_tab.sampling_interval_layout.spin_box.setValue(PARM_EV['SamplingInterval'])
        self.evaluation_tab.distance_interval_layout.double_spin_box.setValue(PARM_EV['DistanceInterval'])
        self.evaluation_tab.time_speed_threshold_layout.double_spin_box.setValue(PARM_EV['VehicleSpeedThreshold'])

        print('Set all tab\'s configuration')

    def ResetResultsLabels(self, PARM_LIDAR, zrp_calib_result=None):
        # reset data validation result
        self.ResetValidationLabel(CONST_UNEDITABLE_LABEL, PARM_LIDAR, self.datavalidation_tab.scroll_box.layout, self.datavalidation_tab.xyyaw_result_labels,
                               self.datavalidation.RMSETranslationErrorDict, self.datavalidation.RMSERotationErrorDict)

        # reset handeye calibration result
        self.ResetResultsLabel(CONST_UNEDITABLE_LABEL, PARM_LIDAR, self.handeye_tab.scroll_box.layout, self.handeye_tab.xyyaw_result_labels,
                               self.handeye.CalibrationParam)
        # reset unsupervised calibration result
        self.ResetResultsLabel(CONST_UNEDITABLE_LABEL, PARM_LIDAR, self.unsupervised_tab.scroll_box.layout, self.unsupervised_tab.xyyaw_result_labels,
                               self.unsupervised.CalibrationParam)

        # reset evaluation select method
        self.ResetResultsLabel(CONST_EVAULATION_LABEL, PARM_LIDAR, self.evaluation_tab.scroll_box.layout,
                               self.evaluation_tab.xyyaw_result_labels,
                               self.handeye.CalibrationParam)

        # reset unsupervised initial value of handeye
        self.ResetResultsLabel(CONST_EDITABLE_LABEL2, PARM_LIDAR, self.unsupervised_tab.optimization_initial_value_tab.scroll_box1.layout,
                               self.unsupervised_tab.handeye_result_labels,
                               self.unsupervised_tab.edit_handeye_calibration_parm)

        # reset unsupervised initial value of custom
        self.ResetResultsLabel(CONST_EDITABLE_LABEL2, PARM_LIDAR, self.unsupervised_tab.optimization_initial_value_tab.scroll_box2.layout,
                               self.unsupervised_tab.user_define_initial_labels,
                               self.config.CalibrationParam)

        if zrp_calib_result is not None:
            # reset zrp tab zrp calibration result
            self.ResetResultsLabel(CONST_UNEDITABLE_LABEL2, PARM_LIDAR,
                                               self.zrollpitch_tab.scroll_box.layout,
                                               self.zrollpitch_tab.result_labels,
                                               zrp_calib_result)

            # reset data validation tab zrp calibration result
            self.ResetResultsLabel(CONST_ZRP_LABEL, PARM_LIDAR,
                                               self.datavalidation_tab.zrp_scroll_box.layout,
                                               self.datavalidation_tab.zrp_result_labels,
                                               zrp_calib_result)

            # reset handeye tab zrp calibration result
            self.ResetResultsLabel(CONST_ZRP_LABEL, PARM_LIDAR,
                                               self.handeye_tab.zrp_scroll_box.layout,
                                               self.handeye_tab.zrp_result_labels,
                                               zrp_calib_result)

            # reset unsupervised tab zrp calibration result
            self.ResetResultsLabel(CONST_ZRP_LABEL, PARM_LIDAR,
                                               self.unsupervised_tab.zrp_scroll_box.layout,
                                               self.unsupervised_tab.zrp_result_labels,
                                               zrp_calib_result)

    def ResetResultsLabel(self, label_type, PARM_LIDAR, layout, labels, calibration_param):
        self.RemoveLayout(layout)
        labels.clear()
        for idxSensor in PARM_LIDAR['CheckedSensorList']:
            if label_type is CONST_UNEDITABLE_LABEL:
                result_label = element.CalibrationResultLabel(idxSensor, 'x [m]', 'y [m]', 'yaw [deg]')
                if calibration_param.get(idxSensor) is not None:
                    result_label.label_edit1.setText(str(round(calibration_param[idxSensor][3], 4)))
                    result_label.label_edit2.setText(str(round(calibration_param[idxSensor][4], 4)))
                    result_label.label_edit3.setText(str(round(calibration_param[idxSensor][2] * 180.0 / math.pi, 4)))

            elif label_type is CONST_UNEDITABLE_LABEL2:
                result_label = element.CalibrationResultLabel(idxSensor, 'z [m]', 'roll [deg]', 'pitch [deg]')
                if calibration_param.get(idxSensor) is not None:
                    result_label.label_edit1.setText(str(round(calibration_param[idxSensor][0], 4)))
                    result_label.label_edit2.setText(str(round(calibration_param[idxSensor][1], 4)))
                    result_label.label_edit3.setText(str(round(calibration_param[idxSensor][2], 4)))
                    
            elif label_type is CONST_EDITABLE_LABEL2:
                result_label = element.CalibrationResultEditLabel2(idxSensor, calibration_param, self)
                if calibration_param.get(idxSensor) is not None:
                    result_label.double_spin_box_x.setValue(calibration_param[idxSensor][3])
                    result_label.double_spin_box_y.setValue(calibration_param[idxSensor][4])
                    result_label.double_spin_box_yaw.setValue(calibration_param[idxSensor][2] * 180.0 / math.pi)

            elif label_type is CONST_EVAULATION_LABEL:
                buttons = {0:True, 1:False, 2:False}
                result_label = element.RadioWithEditLabel(CONST_EVALUATION_RESULT_LABEL, idxSensor, buttons, self, editable=True, using_checkbox=True)

            elif label_type is CONST_ZRP_LABEL:
                buttons = {0:True, 1:False}
                result_label = element.RadioWithEditLabel(CONST_EVALUATION_ZRP_RESULT_LABEL, idxSensor, buttons, self, editable=True, using_checkbox=False)
                if calibration_param.get(idxSensor) is not None:
                    result_label.spinbox1.setValue(round(calibration_param[idxSensor][0], 4))
                    result_label.spinbox2.setValue(round(calibration_param[idxSensor][1], 4))
                    result_label.spinbox3.setValue(round(calibration_param[idxSensor][2], 4))

            labels[idxSensor] = result_label
            layout.addLayout(result_label)
        layout.addStretch(1)
        
    def ResetValidationLabel(self, label_type, PARM_LIDAR, layout, labels, RMSETranslationError, RMSERotationError):
        self.RemoveLayout(layout)
        labels.clear()

        for idxSensor in PARM_LIDAR['CheckedSensorList']:
            if label_type is CONST_UNEDITABLE_LABEL:
                result_label = element.ValidationResultLabel(idxSensor)
                if RMSETranslationError.get(idxSensor) is not None:
                    result_label.label_edit_translation_error.setText(str(round(RMSETranslationError[idxSensor], 4)))
                if RMSERotationError.get(idxSensor) is not None:
                    result_label.label_edit_rotation_error.setText(str(round(RMSERotationError[idxSensor], 4)))
           
            labels[idxSensor] = result_label
            layout.addLayout(result_label)
        layout.addStretch(1)
        
    def RemoveLayout(self, target):
        while target.count():
            item = target.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
            else:
                if 'QSpacerItem' in str(type(item)):
                    target.removeItem(item)
                else:
                    self.RemoveLayout(item)

        layout = target.itemAt(0)
        target.removeItem(layout)

    def DisplayLiDAR(self, calib_result, PARM_LIDAR, ren, iren, renWin, widget, is_single=False):

        lidar_info_dict = {}

        if is_single:
            lidar_info_dict[self.config.PARM_LIDAR['SingleSensor']] = [calib_result[0][0], calib_result[1][0], calib_result[2][0],
                                                                       calib_result[3][0], calib_result[4][0], calib_result[5][0]]
        else:
            for i in range(len(PARM_LIDAR['CheckedSensorList'])):
                idxSensors = list(PARM_LIDAR['CheckedSensorList'])

                lidar_info = [calib_result[0][i], calib_result[1][i], calib_result[2][i],
                           calib_result[3][i], calib_result[4][i], calib_result[5][i]]
                lidar_info_dict[idxSensors[i]] = lidar_info

        actors = vtk_lidar_calib.GetActors(lidar_info_dict)
        vtk_lidar_calib.GetAxis(iren, widget)

        # Assign actor to the renderer
        for actor in actors:
            ren.AddActor(actor)

        colors = vtk.vtkNamedColors()
        ren.SetBackground(colors.GetColor3d('white'))

        ren.ResetCamera()
        renWin.Render()

        iren.Start()

    def ViewPointCloud(self, df_info, pointcloud, PARM_LIDAR, ax=None, canvas=None):
        lidar_num = len(PARM_LIDAR['CheckedSensorList'])
        column = '2'
        row = str(math.ceil(lidar_num / 2))

        fig = plt.figure(figsize=(16, 12), dpi=70)

        if ax is not None:
            ax.plot(df_info['east_m'].values, df_info['north_m'].values, '.', label='trajectory', color='gray')
        for i in range(len(PARM_LIDAR['CheckedSensorList'])):
            idxSensor = list(PARM_LIDAR['CheckedSensorList'])
            if canvas is None:
                plot_num_str = column + row + str(i + 1)
                ax = fig.add_subplot(int(plot_num_str))
                ax.plot(df_info['east_m'].values, df_info['north_m'].values, '.', label='trajectory', color='gray')

            strColIndex = 'XYZRGB_' + str(idxSensor[i])
            # ax.scatter(pointcloud[idxSensor[i]][:, 0], pointcloud[idxSensor[i]][:, 1], pointcloud[idxSensor[i]][:, 2])
            ax.plot(pointcloud[idxSensor[i]][:, 0], pointcloud[idxSensor[i]][:, 1], '.',
                    color=self.color_list[(idxSensor[i]) % len(self.color_list)], label=strColIndex, ms = 2)

            if canvas is None:
                ax.axis('equal')
                ax.grid()
                ax.legend(markerscale=2)
                ax.set_title('Result of calibration - LiDAR' + str(idxSensor[i]))

        if canvas is not None:
            ax.axis('equal')
            ax.grid()
            ax.legend(markerscale=2)
            ax.set_title('Result of calibration')
            canvas.draw()
        else:
            root = Tk.Tk()
            canvas = FigureCanvasTkAgg(fig, master=root)
            nav = NavigationToolbar2Tk(canvas, root)
            canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            canvas._tkcanvas.pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            root.mainloop()

    def ViewGruondPoint2D(self, pointcloud, filtered_pointcloud,  ax=None, canvas=None):
        fig = plt.figure(figsize=(16, 12), dpi=70)
                
        if ax is not None:
            ax.plot(pointcloud[:,0], pointcloud[:,1], ',', label='PointCloud')
            ax.plot(filtered_pointcloud[:,0], filtered_pointcloud[:,1], ',', label='Selected Ground Points')

        if canvas is None:
            ax = fig.add_subplot(1, 1, 1)
            ax.plot(pointcloud[:,0], pointcloud[:,1], ',', label='PointCloud')
            ax.plot(filtered_pointcloud[:,0], filtered_pointcloud[:,1], ',', label='Selected Ground Points')
            ax.axis('equal')
            ax.grid()
            ax.legend(markerscale=2)
            ax.set_title('Selected Ground Points')
            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')
            

        if canvas is not None:
            ax.axis('equal')
            ax.grid()
            ax.legend(markerscale=2)
            ax.set_title('Selected Ground Points')
            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')
            canvas.draw()
        else:
            root = Tk.Tk()
            canvas = FigureCanvasTkAgg(fig, master=root)
            nav = NavigationToolbar2Tk(canvas, root)
            canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            canvas._tkcanvas.pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            root.mainloop()

    def ViewGroundPoint3D(self, pointcloud, filtered_pointcloud,  ax=None, canvas=None):
        fig = plt.figure(figsize=(16, 12), dpi=70)
                
        if ax is not None:
            ax.scatter(pointcloud[:,0], pointcloud[:,1], pointcloud[:,2], zdir='z', s=0.2, c=None, depthshade=True, label = 'PointCloud')
            ax.scatter(filtered_pointcloud[:,0], filtered_pointcloud[:,1], filtered_pointcloud[:,2], zdir='z', s=0.2, c=None, depthshade=True, label = 'Selected Ground Points')
       

        if canvas is None:
            ax = fig.add_subplot(111, projection='3d')
            ax.scatter(pointcloud[:,0], pointcloud[:,1], pointcloud[:,2], zdir='z', s=0.2, c=None, depthshade=True, label = 'PointCloud')
            ax.scatter(filtered_pointcloud[:,0], filtered_pointcloud[:,1], filtered_pointcloud[:,2], zdir='z', s=0.2, c=None, depthshade=True, label = 'Selected Ground Points')
            ax.grid()
            ax.legend(markerscale=2)
            ax.set_title('Selected Ground Points')
            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')
            ax.set_zlabel('Z [m]')
            

        if canvas is not None:
            ax.grid()
            ax.legend(markerscale=2)
            ax.set_title('Selected Ground Points')
            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')
            ax.set_ylabel('Z [m]')
            canvas.draw()
        else:
            root = Tk.Tk()
            canvas = FigureCanvasTkAgg(fig, master=root)
            nav = NavigationToolbar2Tk(canvas, root)
            canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            canvas._tkcanvas.pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            root.mainloop()
            
    def ViewDistanceGraph(self, timestamp, measured_distance,  ax=None, canvas=None):
        fig = plt.figure(figsize=(16, 12), dpi=70)
                
        if ax is not None:
            ax.stem(timestamp, measured_distance, label='Mean distance between point and plane [m]')
            
        if canvas is None:
            ax = fig.add_subplot(1, 1, 1)
            ax.stem(timestamp, measured_distance, label='Mean distance between point and plane [m]')
            val = np.max(measured_distance) * 1.4
            ax.set_ylim([0 -val/2, np.max(measured_distance) + val/2])
            ax.grid()
            ax.legend(markerscale=2)
            ax.set_title('Distance Error')
            ax.set_xlabel('Time [s]')
            ax.set_ylabel('Error [m]')
            
        if canvas is not None:
            val = np.max(measured_distance) * 1.4
            ax.set_ylim([0 -val/2, np.max(measured_distance) + val/2])
            ax.grid()
            ax.legend(markerscale=2)
            ax.set_title('Distance Error')
            ax.set_xlabel('Time [s]')
            ax.set_ylabel('Error [m]')
            canvas.draw()
        else:
            root = Tk.Tk()
            canvas = FigureCanvasTkAgg(fig, master=root)
            nav = NavigationToolbar2Tk(canvas, root)
            canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            canvas._tkcanvas.pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            root.mainloop()
        
    def ViewGroundSlopeGraph(self, timestamp, ground_slope,  ax=None, canvas=None):
        fig = plt.figure(figsize=(16, 12), dpi=70)
                
        if ax is not None:
            ax.stem(timestamp, ground_slope, label='Transformed Ground Slope [deg]')
            val = np.max(ground_slope) * 1.4
            ax.set_xlim([timestamp[0] - 3, timestamp[len(timestamp) - 1] + 3])
            ax.set_ylim([0 -val/2, np.max(ground_slope) + val/2])
            ax.axhline(3,color = 'gray',label='Threshold')
            if np.max(ground_slope) > 3:
                ax.fill_betweenx([3, np.max(ground_slope) + val/2], [timestamp[len(timestamp)-1]+3, timestamp[len(timestamp)-1]+3], color = 'gray', alpha = 0.2)
                ax.text(timestamp[0]+5, np.max(ground_slope) + 20,'Fault',fontsize=13, color = 'gray')
            
        if canvas is None:
            ax = fig.add_subplot(1, 1, 1)
            ax.stem(timestamp, ground_slope, label='Transformed Ground Slope [deg]')
            ax.axhline(3,color = 'gray',label='Threshold')
            ax.text(timestamp[0] + 5,max(ground_slope) + 20,'Fault',fontsize=13, color = 'gray')
            val = np.max(ground_slope) * 1.4
            ax.set_xlim([timestamp[0] - 3, timestamp[len(timestamp) - 1] + 3])
            ax.set_ylim([0 -val/2, np.max(ground_slope) + val/2])            
            if np.max(ground_slope) > 3:
                ax.fill_betweenx([3, np.max(ground_slope) + val/2], [timestamp[len(timestamp)-1]+3, timestamp[len(timestamp)-1]+3], color = 'gray', alpha = 0.2)
                ax.text(timestamp[0] + 5,max(ground_slope) + 20,'Fault',fontsize=13, color = 'gray')
            ax.grid()
            ax.legend(markerscale=2)
            #ax.text(0, ground_slope*1.02, '%.3f' % ground_slope, rotation=90, ha='center', va='bottom')
            ax.set_title('Ground Slope')
            ax.set_xlabel('Time [s]')
            ax.set_ylabel('Slope [deg]')
            

        if canvas is not None:
            ax.grid()
            ax.legend(markerscale=2)
            ax.set_title('Ground Slope')
            ax.set_xlabel('Time [s]')
            ax.set_ylabel('Slope [deg]')
            canvas.draw()
        else:
            root = Tk.Tk()
            canvas = FigureCanvasTkAgg(fig, master=root)
            nav = NavigationToolbar2Tk(canvas, root)
            canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            canvas._tkcanvas.pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            root.mainloop()    

    def ViewTranslationError(self, error, PARM_LIDAR,  ax=None, canvas=None):
        lidar_num = len(PARM_LIDAR['CheckedSensorList'])
        column = '2'
        row = str(math.ceil(lidar_num / 2))
        fig = plt.figure(figsize=(16, 12), dpi=70)
        
        #if ax is not None:
        
        for i in range(len(PARM_LIDAR['CheckedSensorList'])):
            idxSensor = list(PARM_LIDAR['CheckedSensorList'])
            
            if canvas is None:
                plot_num_str = column + row + str(i + 1)
                ax = fig.add_subplot(int(plot_num_str))

            strColIndex = 'XYZRGB_' + str(i)
            ax.plot(error[idxSensor[i]], color=self.color_list[(idxSensor[i]) % len(self.color_list)], label='LiDAR' + str(idxSensor[i]))

            if canvas is None:
                ax.grid()
                ax.legend(markerscale=2)
                ax.set_title('Translation RMSE between Vehicle and LiDAR' + str(idxSensor[i]))
                ax.set_xlabel('Step [cnt]')
                ax.set_ylabel('RMSE [m]')
                ax.set_xlim([-10,len(error[idxSensor[i]])+10])               
                val = np.max(error[idxSensor[i]]) * 1.4
                ax.set_ylim([0 -val/2, np.max(error[idxSensor[i]]) + val/2])
                #plt.axis('scaled')


        if canvas is not None:
            max_err = []
            for i in range(len(PARM_LIDAR['CheckedSensorList'])):
                idxSensor = list(PARM_LIDAR['CheckedSensorList'])
                max_err.append(np.max(error[idxSensor[i]]))
            
            ax.grid()
            ax.legend(markerscale=2)
            ax.set_title('Translation RMSE')
            ax.set_xlabel('Step [cnt]')
            ax.set_ylabel('RMSE [m]')           
            val = np.max(max_err) * 1.4
            ax.set_ylim([0 -val/2, np.max(max_err) + val/2]) 
            canvas.draw()
        else:
            root = Tk.Tk()
            canvas = FigureCanvasTkAgg(fig, master=root)
            nav = NavigationToolbar2Tk(canvas, root)
            canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            canvas._tkcanvas.pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            root.mainloop()

    def ViewRotationError(self, error, PARM_LIDAR,  ax=None, canvas=None):
        lidar_num = len(PARM_LIDAR['CheckedSensorList'])
        column = '2'
        row = str(math.ceil(lidar_num / 2))
        fig = plt.figure(figsize=(16, 12), dpi=70)
        #fig = plt.figure(figsize=(16, 12))
        
        #if ax is not None:

        for i in range(len(PARM_LIDAR['CheckedSensorList'])):
            idxSensor = list(PARM_LIDAR['CheckedSensorList'])
             
            if canvas is None:
                plot_num_str = column + row + str(i + 1)
                ax = fig.add_subplot(int(plot_num_str))

            strColIndex = 'XYZRGB_' + str(idxSensor[i])
            ax.plot(error[idxSensor[i]], color=self.color_list[(idxSensor[i]) % len(self.color_list)], label='LiDAR' + str(idxSensor[i]))

            if canvas is None:
                ax.grid()
                ax.legend(markerscale=2)
                ax.set_title('Rotation RMSE between Vehicle and LiDAR' + str(idxSensor[i]))
                ax.set_xlabel('Step [cnt]')
                ax.set_ylabel('RMSE [deg]')
                ax.set_xlim([-10,len(error[idxSensor[i]])+10])            
                val = np.max(error[idxSensor[i]]) * 1.4
                ax.set_ylim([0 -val/2, np.max(error[idxSensor[i]]) + val/2])

        if canvas is not None:
            max_err = []
            for i in range(len(PARM_LIDAR['CheckedSensorList'])):
                idxSensor = list(PARM_LIDAR['CheckedSensorList'])
                max_err.append(np.max(error[idxSensor[i]]))
                
            ax.grid()
            ax.legend(markerscale=2)
            ax.set_title('Rotation RMSE')
            ax.set_xlabel('Step [cnt]')
            ax.set_ylabel('RMSE [deg]')
            val = np.max(max_err) * 1.4
            ax.set_ylim([0 -val/2, np.max(max_err) + val/2])
            canvas.draw()
        else:
            root = Tk.Tk()
            canvas = FigureCanvasTkAgg(fig, master=root)
            nav = NavigationToolbar2Tk(canvas, root)
            canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            canvas._tkcanvas.pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            root.mainloop()

    def ViewRMSE(self, rmse_x, rmse_y, rmse_yaw, lidarlist, PARM_LIDAR, xy_ax, yaw_ax, canvas):
        index = np.arange(len(PARM_LIDAR['CheckedSensorList']))
        sensor_num = len(PARM_LIDAR['CheckedSensorList'])
        bar_width = 0.35
        opacity = 0.4

        rect_x = xy_ax.bar(index[:sensor_num] - bar_width / 2, rmse_x[:sensor_num], bar_width/2., alpha=opacity, color="b", label='RMSE x')
        rect_y = xy_ax.bar(index[:sensor_num] + bar_width / 2, rmse_y[:sensor_num], bar_width/2., alpha=opacity, color="r", label='RMSE y')
        self.evaluation.AutoLabel(rect_x, xy_ax)
        self.evaluation.AutoLabel(rect_y, xy_ax)
        xy_ax.set_xlim(-1, sensor_num)
        rect_x_height = self.evaluation.GetMaxBarHeight(rect_x)
        rect_y_height = self.evaluation.GetMaxBarHeight(rect_y)
        height = max(rect_x_height, rect_y_height) * 1.2
        xy_ax.set_ylim(0, height)
        xy_ax.set_xticks(list(range(len(lidarlist))))
        xy_ax.set_xticklabels(lidarlist)
        xy_ax.set_xlabel('LiDAR List')
        xy_ax.set_ylabel('RMSE [m]')
        # xy_ax.set_title('RMSE x, y')
        xy_ax.legend()

        rect_yaw = yaw_ax.bar(index[:sensor_num], rmse_yaw[:sensor_num], bar_width / 2., alpha=opacity, color="g", label='RMSE yaw')
        self.evaluation.AutoLabel(rect_yaw, yaw_ax)
        yaw_ax.set_xlim(-1, sensor_num)
        height = self.evaluation.GetMaxBarHeight(rect_yaw) * 1.2
        yaw_ax.set_ylim(0, height)
        yaw_ax.set_xticks(list(range(len(lidarlist))))
        yaw_ax.set_xticklabels(lidarlist)
        yaw_ax.set_xlabel('LiDAR List')
        yaw_ax.set_ylabel('RMSE [deg]')
        # yaw_ax.set_title('RMSE yaw')
        yaw_ax.legend()
        canvas.draw()

    def ErrorPopUp(self, error_message):
        widget = QWidget()

        qr = widget.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        widget.move(qr.topLeft())

        QMessageBox.information(widget, 'Information', error_message)

    def resizeEvent(self, e):
        self.resize_count += 1

        if self.resize_count >= 2:
            width = self.config_tab.next_btn.geometry().width()
            self.config_tab.lidar_config_groupbox.setFixedWidth(round(width/2))
            self.config_tab.next_btn.setFixedHeight(CONST_NEXT_BTN_HEIGHT)
            self.importing_tab.next_btn.setFixedHeight(CONST_NEXT_BTN_HEIGHT)
            self.importing_tab.lidar_scroll_box.setFixedHeight(CONST_SCROLL_BOX_HEIGHT)
            self.importing_tab.gnss_scroll_box.setFixedHeight(CONST_SCROLL_BOX_HEIGHT)

            self.unsupervised_tab.text_edit.setFixedHeight(CONST_SCROLL_BOX_HEIGHT)
            self.unsupervised_tab.optimization_initial_value_tab.tabs.setFixedHeight(CONST_SCROLL_BOX_HEIGHT+50)

    def XY(self, ren, renWin):
        ren.GetActiveCamera().SetPosition(0, 0, 45)
        ren.GetActiveCamera().SetFocalPoint(1, 0, 0)
        ren.GetActiveCamera().SetViewUp(0, 0, 1)
        ren.ResetCamera()
        renWin.Render()

    def YZ(self, ren, renWin):
        ren.GetActiveCamera().SetPosition(45, 0, 0)
        ren.GetActiveCamera().SetFocalPoint(1, 0, 0)
        ren.GetActiveCamera().SetViewUp(0, 0, 1)
        ren.ResetCamera()
        renWin.Render()

    def ZX(self, ren, renWin):
        ren.GetActiveCamera().SetPosition(0, 45, 0)
        ren.GetActiveCamera().SetFocalPoint(1, 0, 0)
        ren.GetActiveCamera().SetViewUp(0, 1, 0)
        ren.ResetCamera()
        renWin.Render()

    # Utils

    def GetZRPCalibDict(self, zrp_result_labels, manual_zrp_calib_result):
        calib = {}

        for idxSensor in self.config.PARM_LIDAR["CheckedSensorList"]:
            status = zrp_result_labels[idxSensor].button_group.checkedId()

            if status == CONST_AUTOMATIC:
                if self.zrollpitch_tab.calib_result.get(idxSensor) is None:
                    calib_result = [0.0, 0.0, 0.0]
                else:
                    calib_result = self.zrollpitch_tab.calib_result[idxSensor]

                calib[idxSensor] = calib_result
            elif status == CONST_MANUAL:
                calib[idxSensor] = manual_zrp_calib_result[idxSensor]

        return calib

    def GetZRPCalibList(self, zrp_result_labels, manual_zrp_calib_result):
        calib_z = []
        calib_r = []
        calib_p = []

        for idxSensor in self.config.PARM_LIDAR["CheckedSensorList"]:
            status = zrp_result_labels[idxSensor].button_group.checkedId()

            if status == CONST_AUTOMATIC:
                if self.zrollpitch_tab.calib_result.get(idxSensor) is None:
                    calib_result = [0.0, 0.0, 0.0]
                else:
                    calib_result = self.zrollpitch_tab.calib_result[idxSensor]

                calib_z.append(calib_result[0])
                calib_r.append(calib_result[1])
                calib_p.append(calib_result[2])
            elif status == CONST_MANUAL:
                calib_z.append(manual_zrp_calib_result[idxSensor][0])
                calib_r.append(manual_zrp_calib_result[idxSensor][1])
                calib_p.append(manual_zrp_calib_result[idxSensor][2])

        calib_result = [calib_z, calib_r, calib_p]
        return calib_result

## Version
def version():
    version = 0
    f = open("LidarCalibrationTools.py",'r',encoding = 'utf-8').read().split()
    for i, word in enumerate(f):
        if (word == '@version'):
            version = f[i+1]
    return version

if __name__ == '__main__':
    argc = len(sys.argv)
    if argc > 1:
        if (sys.argv[1] == "-v" or sys.argv[1] == "--version"):
            print(version())
        else:
            print('unknown argument')
    else:
        app = QApplication(sys.argv)
        app.setStyle("fusion")
        w = MyApp()
        w.show()
        sys.exit(app.exec_())

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

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

import element
from process import get_result
from process import v1_step1_configuration
from process import v1_step2_import_data
from process import v1_step2_2_data_validation
from process import v1_step3_handeye
from process import v1_step4_unsupervised
from process import v1_step5_evaluation
from widget.QButton import *
from widget import QThread
from widget.QScrollarea import *
from PyQt5.QtGui import *

import pylab as pl
import numpy as np
from PIL import Image
import tkinter as Tk
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

## button size
CONST_NEXT_BTN_HEIGHT = 80
CONST_SCROLL_BOX_HEIGHT = 120

## tab
CONST_CONFIG_TAB = 0
CONST_IMPORTDATA_TAB = 1
CONST_VALIDATION_TAB = 2
CONST_HANDEYE_TAB = 3
CONST_UNSUPERVISED_TAB = 4
CONST_EVALUATION_TAB = 5

## label type
CONST_UNEDITABLE_LABEL = 0
CONST_EDITABLE_LABEL = 1
CONST_EDITABLE_LABEL2 = 2
CONST_EVAULATION_LABEL = 3

## calibration status
CONST_STOP = 0
CONST_PLAY = 1
CONST_PAUSE = 2

CONST_CUSTOM = 1
CONST_HANDEYE = 2
CONST_UNSUPERVISED = 3

class ConfigurationTab(QWidget):
    def __init__(self, ui):
        super().__init__() # what's doing this? --> Operate Qwidget's init() function
        self.is_lidar_num_changed = False
        self.ui = ui # What is ui? -- ui is FormWidget function class

        self.initUi()

    def initUi(self):
        main_vbox = QVBoxLayout()

        config_widget = QWidget()
        config_widget.setLayout(self.SetConfiguration_Layout())
        main_vbox.addWidget(config_widget, 40)

        interface_widget = QWidget()
        interface_widget.setLayout(self.UserInterface_Layout())
        main_vbox.addWidget(interface_widget, 60)

        self.setLayout(main_vbox)

    ## Layout
    def SetConfiguration_Layout(self):
        hbox = QHBoxLayout()
        hbox.addWidget(self.SetConfiguration_LidarConfiguration_Groupbox())
        hbox.addWidget(self.SetConfiguration_PointCloudConfiguration_Groupbox())

        return hbox

    def UserInterface_Layout(self):
        vbox = QVBoxLayout()

        self.image_display_widget = element.ImageDisplay(self.ui.config.PATH['Image_path'])
        vbox.addWidget(self.image_display_widget)

        self.next_btn = QPushButton('Next step')
        self.next_btn.clicked.connect(self.NextBtn)
        vbox.addWidget(self.next_btn)

        return vbox

    ## Groupbox

    def SetConfiguration_LidarConfiguration_Groupbox(self):
        self.lidar_config_groupbox = QGroupBox('LiDAR Configuration')
        vbox = QVBoxLayout()

        self.lidar_num_label_layout = element.SpinBoxLabelLayout('LiDAR Num', self.ui)
        vbox.addLayout(self.lidar_num_label_layout)

        self.select_using_sensor_list_layout = element.CheckBoxListLayout(self.ui, 'Select Using Sensor List')
        vbox.addLayout(self.select_using_sensor_list_layout)

        self.lidar_config_groupbox.setLayout(vbox)
        return self.lidar_config_groupbox

    def SetConfiguration_PointCloudConfiguration_Groupbox(self):
        groupbox = QGroupBox('[ PointCloud Configuration ]')
        vbox = QVBoxLayout()

        self.minimum_threshold_distance_layout = element.DoubleSpinBoxLabelLayout("Minimum Threshold Distance [m]", self.ui)
        vbox.addLayout(self.minimum_threshold_distance_layout)

        self.maximum_threshold_istance_layout = element.DoubleSpinBoxLabelLayout("Maximum Threshold Distance [m]", self.ui)
        vbox.addLayout(self.maximum_threshold_istance_layout)

        self.minimum_threshold_layout_x = element.DoubleSpinBoxLabelLayout("Minimum Threshold X [m]", self.ui)
        vbox.addLayout(self.minimum_threshold_layout_x)

        self.maximum_threshold_layout_x = element.DoubleSpinBoxLabelLayout("Maximum Threshold X [m]", self.ui)
        vbox.addLayout(self.maximum_threshold_layout_x)

        self.minimum_threshold_layout_y = element.DoubleSpinBoxLabelLayout("Minimum Threshold Y [m]", self.ui)
        vbox.addLayout(self.minimum_threshold_layout_y)

        self.maximum_threshold_layout_y = element.DoubleSpinBoxLabelLayout("Maximum Threshold Y [m]", self.ui)
        vbox.addLayout(self.maximum_threshold_layout_y)

        self.minimum_threshold_layout_z = element.DoubleSpinBoxLabelLayout("Minimum Threshold Z [m]", self.ui)
        vbox.addLayout(self.minimum_threshold_layout_z)

        self.maximum_threshold_layout_z = element.DoubleSpinBoxLabelLayout("Maximum Threshold Z [m]", self.ui)
        vbox.addLayout(self.maximum_threshold_layout_z)

        groupbox.setLayout(vbox)
        return groupbox

    ## Callback Function

    def NextBtn(self):
        self.ui.tabs.setTabEnabled(CONST_IMPORTDATA_TAB, True)
        self.ui.tabs.setCurrentIndex(CONST_IMPORTDATA_TAB)

class ImportDataTab(QWidget):
    def __init__(self, ui):
        super().__init__()
        self.ui = ui

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

        self.logging_file_path_layout = element.FileInputWithCheckBtnLayout('Select Logging File Path', self.ui)
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

        self.sampling_interval_layout = element.SpinBoxLabelLayout('Sampling Interval [Count]', self.ui)
        vbox.addLayout(self.sampling_interval_layout)

        self.time_speed_threshold_layout = element.DoubleSpinBoxLabelLayout('Vehicle Minimum Speed [km/h]', self.ui)
        vbox.addLayout(self.time_speed_threshold_layout)

        groupbox.setLayout(vbox)
        return groupbox

    def Main_LimitTime_Groupbox(self):
        groupbox = QGroupBox('Limit Time Data')
        vbox = QVBoxLayout()

        self.limit_time_layout = element.SlideLabelLayouts(self.ui, '[ Limit Time ]')
        vbox.addLayout(self.limit_time_layout)

        groupbox.setLayout(vbox)
        return groupbox

    ## Callback func

    def NextBtn(self):
        if self.ui.importing.is_complete == False:
            self.ui.ErrorPopUp('Please import logging file path')
        else:
            self.ui.tabs.setTabEnabled(CONST_HANDEYE_TAB, True)
            self.ui.tabs.setTabEnabled(CONST_VALIDATION_TAB, True)
            self.ui.tabs.setTabEnabled(CONST_UNSUPERVISED_TAB, True)
            self.ui.tabs.setCurrentIndex(CONST_VALIDATION_TAB)

    def IterationPercentage(self, percentage_dict):
        idxSensor = list(percentage_dict.keys())[0]
        percentage = list(percentage_dict.values())[0]
        text = 'XYZRGB {}'.format(idxSensor) + ' ' + str(int(percentage)) + '%'

        self.logging_file_path_layout.lidar_buttons[idxSensor].setText(text)

    def EndImport(self):
        parsed_pandas_dataframe = self.ui.importing.text_pointcloud
        self.logging_file_path_layout.parsed_bin = parsed_pandas_dataframe.to_string()

        default_start_time = self.ui.importing.DefaultStartTime
        default_end_time = self.ui.importing.DefaultEndTime

        # Import tab
        ## Set Configuration
        self.ui.RemoveLayout(self.scroll_box.layout)

        if self.ui.importing.has_gnss_file:
            self.init_gnss_value_layout = element.GnssInitEditLabel('Gnss Initial Pose in ENU Coordinate      ', self.ui)
            self.init_gnss_value_layout.double_spin_box_east.setValue(self.ui.importing.df_gnss['east_m'].values[0])
            self.init_gnss_value_layout.double_spin_box_north.setValue(self.ui.importing.df_gnss['north_m'].values[0])
            self.init_gnss_value_layout.double_spin_box_heading.setValue(self.ui.importing.df_gnss['heading'].values[0])
            self.scroll_box.layout.addLayout(self.init_gnss_value_layout)
        elif not self.ui.importing.has_gnss_file:
            ## Set button status in handeye_tab
            self.ui.datavalidation_tab.using_gnss_motion = True
            
            self.ui.handeye_tab.using_gnss_motion = True
            self.ui.handeye_tab.button_group.button(2).setChecked(True)
            self.ui.handeye_tab.prev_checkID = self.ui.handeye_tab.button_group.checkedId()

            ## Set button status in unsupervised_tab
            self.ui.unsupervised_tab.using_gnss_motion = True
            self.ui.unsupervised_tab.button_group.button(2).setChecked(True)
            self.ui.unsupervised_tab.prev_checkID = self.ui.handeye_tab.button_group.checkedId()

        if self.ui.importing.has_motion_file:
            ## Set intial value
            self.init_motion_value_layout = element.GnssInitEditLabel('Motion Initial Pose in ENU coordinate    ', self.ui)
            self.init_motion_value_layout.double_spin_box_east.setValue(self.ui.importing.init[0])
            self.init_motion_value_layout.double_spin_box_north.setValue(self.ui.importing.init[1])
            self.init_motion_value_layout.double_spin_box_heading.setValue(self.ui.importing.init[2])
            self.scroll_box.layout.addLayout(self.init_motion_value_layout)

            ## Change time_speed_threshold_layout status
            self.time_speed_threshold_layout.double_spin_box.setReadOnly(False)
            self.time_speed_threshold_layout.double_spin_box.setStyleSheet("background-color:white")
            self.time_speed_threshold_layout.double_spin_box.setButtonSymbols(QAbstractSpinBox.UpDownArrows)
        elif not self.ui.importing.has_motion_file:
            ## Set button status in handeye_tab
            self.ui.datavalidation_tab.using_gnss_motion = False
            
            self.ui.handeye_tab.using_gnss_motion = False
            self.ui.handeye_tab.button_group.button(1).setChecked(True)
            self.ui.handeye_tab.prev_checkID = self.ui.handeye_tab.button_group.checkedId()

            ## Setbutton status in unsupervised_tab
            self.ui.unsupervised_tab.using_gnss_motion = False
            self.ui.unsupervised_tab.button_group.button(1).setChecked(True)
            self.ui.unsupervised_tab.prev_checkID = self.ui.handeye_tab.button_group.checkedId()

            ## Change time_speed_threshold_layout status
            self.time_speed_threshold_layout.double_spin_box.setReadOnly(True)
            self.time_speed_threshold_layout.double_spin_box.setStyleSheet("background-color: #F0F0F0;")
            self.time_speed_threshold_layout.double_spin_box.setButtonSymbols(QAbstractSpinBox.NoButtons)

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

        # Evaluation tab
        ## set slider default time
        self.ui.evaluation_tab.limit_time_layout.start_time_layout.slider.setMaximum(default_end_time)
        self.ui.evaluation_tab.limit_time_layout.start_time_layout.slider.setMinimum(default_start_time)
        self.ui.evaluation_tab.limit_time_layout.end_time_layout.slider.setMaximum(default_end_time)
        self.ui.evaluation_tab.limit_time_layout.end_time_layout.slider.setMinimum(default_start_time)

        ## set slider and double_spin_box value
        self.ui.evaluation_tab.limit_time_layout.start_time_layout.slider.setValue(self.ui.evaluation_tab.limit_time_layout.end_time_layout.slider.minimum())
        self.ui.evaluation_tab.limit_time_layout.start_time_layout.double_spin_box.setValue(default_start_time)
        self.ui.evaluation_tab.limit_time_layout.end_time_layout.slider.setValue(self.ui.evaluation_tab.limit_time_layout.end_time_layout.slider.maximum())
        self.ui.evaluation_tab.limit_time_layout.end_time_layout.double_spin_box.setValue(default_end_time)

        ## set slider and double_spin_box value start, end time
        self.ui.evaluation_tab.limit_time_layout.start_time = default_start_time
        self.ui.evaluation_tab.limit_time_layout.end_time = default_end_time

class CalibrationTab(QWidget):
    def __init__(self, ui):
        super().__init__()
        self.using_gnss_motion = False
        self.ui = ui

        self.progress_status = CONST_STOP
        self.result_labels = {}
        self.prev_checkID = 1

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

        self.configuration_vbox.addWidget(self.Configuration_SetConfiguration_Groupbox())

        hbox = QHBoxLayout()
        label = QLabel('Used Data')
        hbox.addWidget(label)

        # button for Handeye calibration
        self.button_group = QButtonGroup()
        rbn1 = QRadioButton('GNSS Data')
        rbn1.setChecked(True)
        rbn1.clicked.connect(self.RadioButton)
        hbox.addWidget(rbn1)
        self.button_group.addButton(rbn1, 1)

        # button for unsupervised calibration
        rbn2 = QRadioButton('Motion Data')
        rbn2.clicked.connect(self.RadioButton)
        hbox.addWidget(rbn2)
        self.button_group.addButton(rbn2, 2)
        self.configuration_vbox.addLayout(hbox)

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

        self.result_data_pose_fig = plt.figure()
        self.result_data_pose_canvas = FigureCanvas(self.result_data_pose_fig)
        self.result_data_pose_ax = self.result_data_pose_fig.add_subplot(1, 1, 1)
        self.result_data_pose_ax.grid()
        self.result_data_pose_canvas.draw()
        vbox.addWidget(self.result_data_pose_canvas)

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

    def StartCalibration(self, calibration_id, calibration, vehicle_speed_threshold, start_time, end_time, sensor_list, targets_clear, progress_callbacks, end_callback):
        if self.ui.config_tab.is_lidar_num_changed == True:
            self.ui.ErrorPopUp('Please import after changing lidar number')
            return False
        if self.progress_status is not CONST_STOP:
            return False
        self.progress_status = CONST_PLAY

        for idxSensor in sensor_list['CheckedSensorList']:
            if self.ui.importing.PointCloudSensorList.get(idxSensor) is None:
                self.ui.ErrorPopUp('Import pointcloud {}'.format(idxSensor))
                return False

        for target_clear in targets_clear:
            target_clear()

        if calibration_id == CONST_HANDEYE:
            # disable access to unsupervised tab until complite handeye calibration
            self.ui.tabs.setTabEnabled(CONST_UNSUPERVISED_TAB, False)
            self.ui.tabs.setTabEnabled(CONST_EVALUATION_TAB, False)
            # if unsupervised calibration's result is being, reset the result
            try:
                self.result_data_pose_ax.clear()
                self.result_data_pose_canvas.draw()
                self.result_graph_ax.clear()
                self.result_graph_canvas.draw()
                self.ui.unsupervised_tab.result_data_pose_ax.clear()
                self.ui.unsupervised_tab.result_data_pose_canvas.draw()
                self.ui.unsupervised_tab.result_graph_ax.clear()
                self.ui.unsupervised_tab.result_graph_canvas.draw()
                # print("clear results in unsupervised")
            except:
                pass
            self.ui.handeye_thread._status = True
            self.ui.handeye_thread.SetFunc(calibration,
                                   [start_time, end_time, sensor_list, self.using_gnss_motion, vehicle_speed_threshold])
            try:
                self.ui.handeye_thread.change_value.disconnect()
            except:
                pass
            try:
                self.ui.handeye_thread.iteration_percentage.disconnect()
            except:
                pass
            try:
                self.ui.handeye_thread.end.disconnect()
            except:
                pass
            try:
                self.ui.handeye_thread.emit_string.disconnect()
            except:
                pass

            self.ui.handeye_thread.emit_string.connect(progress_callbacks[0]) # text_edit_callback
            self.ui.handeye_thread.change_value.connect(progress_callbacks[1]) # progress_callback

            self.ui.handeye_thread.end.connect(end_callback)
            self.ui.handeye_thread.start()
        elif calibration_id == CONST_UNSUPERVISED:
            # remove plots
            try:
                self.result_data_pose_ax.clear()
                self.result_data_pose_canvas.draw()
                self.result_graph_ax.clear()
                self.result_graph_canvas.draw()
            except:
                pass
            self.ui.opti_thread._status = True
            self.ui.opti_thread.SetFunc(calibration,
                                   [start_time, end_time, sensor_list, self.using_gnss_motion, vehicle_speed_threshold])
            try:
                self.ui.opti_thread.change_value.disconnect()
            except:
                pass
            try:
                self.ui.opti_thread.iteration_percentage.disconnect()
            except:
                pass
            try:
                self.ui.opti_thread.end.disconnect()
            except:
                pass
            try:
                self.ui.opti_thread.emit_string.disconnect()
            except:
                pass


            self.ui.opti_thread.emit_string.connect(progress_callbacks[0]) # text_edit_callback
            self.ui.opti_thread.end.connect(end_callback)
            self.ui.opti_thread.start()

    def ViewLiDAR(self):
        pass

    def ViewPointCloud(self):
        pass

    def RadioButton(self):
        '''
        only one button can selected
        '''
        status = self.button_group.checkedId()
        if status == 1:  # GNSS Data
            if self.ui.importing.has_gnss_file == False:
                self.button_group.button(self.prev_checkID).setChecked(True)
                self.ui.ErrorPopUp('Please import Gnss.csv')
                return False
            self.using_gnss_motion = False
        elif status == 2:  # Motion Data
            if self.ui.importing.has_motion_file == False:
                self.button_group.button(self.prev_checkID).setChecked(True)
                self.ui.ErrorPopUp('Please import Motion.csv')
                return False
            self.using_gnss_motion = True
        self.prev_checkID = self.button_group.checkedId()


class DataValidation(QWidget):
    def __init__(self, ui):
        super().__init__()
        self.using_gnss_motion = False
        self.ui = ui
        
        self.progress_status = CONST_STOP
        self.result_labels = {}
        self.label_heading_threshold = self.ui.config.PARM_DV['filter_HeadingThreshold']
        self.label_distance_threshold = self.ui.config.PARM_DV['filter_DistanceThreshold']
        self.initUi()
        

    def initUi(self):
        main_vbox = QVBoxLayout()
        
        main_hbox = QHBoxLayout()

        self.config_widget = QWidget()
        self.config_widget.setLayout(self.Configuration_Layout())
        main_hbox.addWidget(self.config_widget, 25)

        self.result_widget = QWidget()
        self.result_widget.setLayout(self.Result_Layout())
        main_hbox.addWidget(self.result_widget, 75)
        #self.setLayout(main_hbox)
        
        main_vbox.addLayout(main_hbox, 50)
        self.next_btn = QPushButton('Skip')
        self.next_btn.clicked.connect(self.NextBtn)
        main_vbox.addWidget(self.next_btn, 50)

        self.setLayout(main_vbox)
        
        
        
        
    def Configuration_Layout(self):
        self.configuration_vbox = QVBoxLayout()


        self.configuration_vbox.addWidget(self.Configuration_DataInfo_Groupbox())

        hbox = QHBoxLayout()
        label = QLabel('Used Data')
        hbox.addWidget(label)

        # button for Handeye calibration
        self.button_group = QButtonGroup()
        rbn1 = QRadioButton('GNSS Data')
        rbn1.setChecked(True)
        rbn1.clicked.connect(self.RadioButton)
        hbox.addWidget(rbn1)
        self.button_group.addButton(rbn1, 1)

        # button for unsupervised calibration
        rbn2 = QRadioButton('Motion Data')
        rbn2.clicked.connect(self.RadioButton)
        hbox.addWidget(rbn2)
        self.button_group.addButton(rbn2, 2)
        self.configuration_vbox.addLayout(hbox)


        self.configuration_vbox.addWidget(self.Configuration_Validation_Groupbox())

        return self.configuration_vbox

        
        
    def Result_Layout(self):
        vbox = QVBoxLayout()

        vbox.addWidget(self.Result_ResultData_Groupbox())
        vbox.addWidget(self.Result_ResultGraph_Groupbox())

        return vbox
        
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
        
    def Configuration_DataInfo_Groupbox(self):
        groupbox = QGroupBox('Configuration Info')
        vbox = QVBoxLayout()
        
        ICP_configuration_label = QLabel('[ ICP Configuration ]', self)
        vbox.addWidget(ICP_configuration_label)
        
        self.maximum_interation_layout = element.SpinBoxLabelLayout('Maximum Iteration [Count]', self.ui)
        vbox.addLayout(self.maximum_interation_layout)

        self.tolerance_layout = element.DoubleSpinBoxLabelLayout('Tolerance', self.ui)
        vbox.addLayout(self.tolerance_layout)

        self.outlier_distance_layout = element.DoubleSpinBoxLabelLayout('Outlier Distance [m]', self.ui)
        vbox.addLayout(self.outlier_distance_layout)

        Threshold_configuration_label = QLabel('[ Threshold ]', self)
        vbox.addWidget(Threshold_configuration_label)
        self.heading_threshold_layout = element.DoubleSpinBoxLabelLayout('Heading Threshold (filter)', self.ui)
        vbox.addLayout(self.heading_threshold_layout)

        self.distance_threshold_layout = element.DoubleSpinBoxLabelLayout('Distance Threshold (filter)', self.ui)
        vbox.addLayout(self.distance_threshold_layout)
        
        
        groupbox.setLayout(vbox)
        
        print(self.maximum_interation_layout)

        return groupbox



    def Configuration_Validation_Groupbox(self):
        groupbox = QGroupBox('Data Validation')
        vbox = QVBoxLayout(self)

        hbox = QHBoxLayout()
        btn = QPushButton('Start')
        btn.clicked.connect(lambda: self.StartValidation(CONST_HANDEYE,
                                                            self.ui.datavalidation.Validation,
                                                            self.ui.config.PARM_IM['VehicleSpeedThreshold'],
                                                            self.ui.importing_tab.limit_time_layout.start_time,
                                                            self.ui.importing_tab.limit_time_layout.end_time,
                                                            self.ui.config.PARM_LIDAR,
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
        vbox.addWidget(self.text_edit)

        self.scroll_box = ScrollAreaV()
        vbox.addWidget(self.scroll_box)

        groupbox.setLayout(vbox)
        return groupbox
    
    def NextBtn(self):
        self.ui.tabs.setCurrentIndex(CONST_HANDEYE_TAB)
    
    def StartValidation(self, validation_exist, validation, vehicle_speed_threshold, start_time, end_time, sensor_list, targets_clear, progress_callbacks, end_callback):
        if self.ui.config_tab.is_lidar_num_changed == True:
            self.ui.ErrorPopUp('Please import after changing lidar number')
            return False
        if self.progress_status is not CONST_STOP:
            return False
        self.progress_status = CONST_PLAY

        for idxSensor in sensor_list['CheckedSensorList']:
            if self.ui.importing.PointCloudSensorList.get(idxSensor) is None:
                self.ui.ErrorPopUp('Import pointcloud {}'.format(idxSensor))
                return False

        for target_clear in targets_clear:
            target_clear()

        try:
            self.result_data_pose_ax.clear()
            self.result_data_pose_canvas.draw()
            self.result_graph_ax.clear()
            self.result_graph_canvas.draw()
            # print("clear results in unsupervised")
        except:
            pass
        self.ui.validation_thread._status = True
        self.ui.validation_thread.SetFunc(validation,
                               [start_time, end_time, sensor_list, self.using_gnss_motion, vehicle_speed_threshold])
        try:
            self.ui.validation_thread.change_value.disconnect()
        except:
            pass
        try:
            self.ui.validation_thread.iteration_percentage.disconnect()
        except:
            pass
        try:
            self.ui.validation_thread.end.disconnect()
        except:
            pass
        try:
            self.ui.validation_thread.emit_string.disconnect()
        except:
            pass

        self.ui.validation_thread.emit_string.connect(progress_callbacks[0]) # text_edit_callback
        self.ui.validation_thread.change_value.connect(progress_callbacks[1]) # progress_callback

        self.ui.validation_thread.end.connect(end_callback)
        self.ui.validation_thread.start()
    
     
        
    def Pause(self):
        if self.progress_status is CONST_PLAY:
            self.progress_status = CONST_PAUSE

            self.ui.validation_thread.pause = True
            self.pause_btn.setText("Resume")

        elif self.progress_status is CONST_PAUSE:
            self.progress_status = CONST_PLAY

            self.ui.validation_thread.pause = False
            self.pause_btn.setText("Pause")

        else:
            return False

    def Cancel(self):
        self.progress_status = CONST_STOP
        self.ui.validation_thread.pause = False
        self.pause_btn.setText("Pause")
        self.ui.validation_thread.toggle_status()

    def EndValibration(self):
        self.progress_status = CONST_STOP
        self.ui.datavalidation.complete_validation = True

        #print(self.ui.config.PARM_DV['filter_HeadingThreshold'])
    
    
        #element.ValidationConfigLabel.label_edit_heading_threshold.setText(format(self.ui.config.PARM_DV['filter_HeadingThreshold'],".4f"))
        #element.ValidationConfigLabel.label_edit_heading_threshold.setText(format(self.ui.config.PARM_DV['filter_DistanceThreshold'],".4f"))

        print(self.ui.config.PARM_DV['filter_HeadingThreshold'])
    
        #self.label_heading_threshold = QLabel('Heading Threshold [deg]: {}'.format(self.ui.config.PARM_DV['filter_HeadingThreshold']))

        #self.label_distance_threshold = QLabel('Distance Threshold [m]: {}'.format(self.ui.config.PARM_DV['filter_DistanceThreshold']))
        
        #self.label_heading_threshold.label_edit_heading_threshold.setText(format(self.label_heading_threshold, ".4f"))
        #self.label_distance_threshold.label_edit_distance_threshold.setText(format(self.label_distance_threshold, ".4f"))
        #self.result_labels[0].label_edit_heading_threshold.setText(format(self.label_heading_threshold, ".4f"))
        #self.result_labels[0].label_edit_distance_threshold.setText(format(self.label_distance_threshold, ".4f"))
    
        # validation tab
        ## Set 'Result Calibration Data'
        
        for idxSensor in self.ui.datavalidation.PARM_LIDAR['CheckedSensorList']:
            self.result_labels[idxSensor].label_edit_translation_error.setText(format(self.ui.datavalidation.RMSETranslationErrorDict[idxSensor], ".4f"))
            self.result_labels[idxSensor].label_edit_rotation_error.setText(format(self.ui.datavalidation.RMSERotationErrorDict[idxSensor], ".4f"))
            
        
        ## Plot 'Result Data'
        self.result_data_pose_ax.clear()
        self.ui.ViewTranslationError(self.ui.datavalidation.TranslationError, self.ui.datavalidation.PARM_LIDAR, self.result_data_pose_ax, self.result_data_pose_canvas)
        
        ## Plot 'Result Graph'
        self.result_graph_ax.clear()
        self.ui.ViewRotationError(self.ui.datavalidation.RotationError, self.ui.datavalidation.PARM_LIDAR, self.result_graph_ax, self.result_graph_canvas)
        




    def ViewTranslationError(self):
        if self.progress_status is not CONST_STOP:
            return False
        self.ui.ViewTranslationError(self.ui.datavalidation.TranslationError, self.ui.config.PARM_LIDAR)

    def ViewRotationError(self):
        if self.progress_status is not CONST_STOP:
            return False
        self.ui.ViewRotationError(self.ui.datavalidation.RotationError, self.ui.config.PARM_LIDAR)
    
    def RadioButton(self):
        '''
        only one button can selected
        '''
        status = self.button_group.checkedId()
        if status == 1:  # GNSS Data
            if self.ui.importing.has_gnss_file == False:
                self.button_group.button(self.prev_checkID).setChecked(True)
                self.ui.ErrorPopUp('Please import Gnss.csv')
                return False
            self.using_gnss_motion = False
        elif status == 2:  # Motion Data
            if self.ui.importing.has_motion_file == False:
                self.button_group.button(self.prev_checkID).setChecked(True)
                self.ui.ErrorPopUp('Please import Motion.csv')
                return False
            self.using_gnss_motion = True
        self.prev_checkID = self.button_group.checkedId()
        
        

class HandEyeTab(CalibrationTab):
    def __init__(self, ui):
        super().__init__(ui)

    ## Groupbox
    def Configuration_SetConfiguration_Groupbox(self):
        groupbox = QGroupBox('Set Configuration')
        vbox = QVBoxLayout()

        liDAR_configuration_label = QLabel('[ HandEye Configuration ]', self)
        vbox.addWidget(liDAR_configuration_label)

        self.maximum_interation_layout = element.SpinBoxLabelLayout('Maximum Iteration [Count]', self.ui)
        vbox.addLayout(self.maximum_interation_layout)

        self.tolerance_layout = element.DoubleSpinBoxLabelLayout('Tolerance', self.ui)
        vbox.addLayout(self.tolerance_layout)

        self.outlier_distance_layout = element.DoubleSpinBoxLabelLayout('Outlier Distance [m]', self.ui)
        vbox.addLayout(self.outlier_distance_layout)

        self.heading_threshold_layout = element.DoubleSpinBoxLabelLayout('Heading Threshold (filter)', self.ui)
        vbox.addLayout(self.heading_threshold_layout)

        self.distance_threshold_layout = element.DoubleSpinBoxLabelLayout('Distance Threshold (filter)', self.ui)
        vbox.addLayout(self.distance_threshold_layout)

        groupbox.setLayout(vbox)
        return groupbox

    def Configuration_Calibration_Groupbox(self):
        groupbox = QGroupBox('HandEye Calibration')
        vbox = QVBoxLayout(self)

        hbox = QHBoxLayout()
        btn = QPushButton('Start')
        btn.clicked.connect(lambda: self.StartCalibration(CONST_HANDEYE,
                                                          self.ui.handeye.Calibration,
                                                          self.ui.config.PARM_IM['VehicleSpeedThreshold'],
                                                          self.ui.importing_tab.limit_time_layout.start_time,
                                                          self.ui.importing_tab.limit_time_layout.end_time,
                                                          self.ui.config.PARM_LIDAR,
                                                          [self.text_edit.clear, self.calibration_pbar.reset],
                                                          [self.text_edit.append, self.calibration_pbar.setValue],
                                                          self.EndCalibration))
        hbox.addWidget(btn)

        self.pause_btn = QPushButton('Pause')
        self.pause_btn.clicked.connect(self.Pause)
        hbox.addWidget(self.pause_btn)
        vbox.addLayout(hbox)

        stop_btn = QPushButton('Stop')
        stop_btn.clicked.connect(self.Cancel)
        hbox.addWidget(stop_btn)
        vbox.addLayout(hbox)

        self.label = QLabel('[ HandEye Calibration Progress ]')
        vbox.addWidget(self.label)

        self.calibration_pbar = QProgressBar(self)
        vbox.addWidget(self.calibration_pbar)

        self.text_edit = QTextEdit()
        vbox.addWidget(self.text_edit)

        self.scroll_box = ScrollAreaV()
        vbox.addWidget(self.scroll_box)

        groupbox.setLayout(vbox)
        return groupbox

    ## Callback func

    def Pause(self):
        if self.progress_status is CONST_PLAY:
            self.progress_status = CONST_PAUSE

            self.ui.handeye_thread.pause = True
            self.pause_btn.setText("Resume")

        elif self.progress_status is CONST_PAUSE:
            self.progress_status = CONST_PLAY

            self.ui.handeye_thread.pause = False
            self.pause_btn.setText("Pause")

        else:
            return False

    def Cancel(self):
        self.progress_status = CONST_STOP
        self.ui.handeye_thread.pause = False
        self.pause_btn.setText("Pause")
        self.ui.handeye_thread.toggle_status()

    def ViewLiDAR(self):
        if self.progress_status is not CONST_STOP:
            return False
        self.ui.ViewLiDAR(self.ui.handeye.calib_x, self.ui.handeye.calib_y, self.ui.handeye.calib_yaw, self.ui.config.PARM_LIDAR)

    def ViewPointCloud(self):
        if self.progress_status is not CONST_STOP:
            return False
        df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.ui.importing,
                                                                                           self.ui.handeye_tab.using_gnss_motion,
                                                                                           self.ui.handeye.PARM_LIDAR,
                                                                                           self.ui.handeye.CalibrationParam,
                                                                                           self.ui.importing_tab.limit_time_layout.start_time,
                                                                                           self.ui.importing_tab.limit_time_layout.end_time)


        self.ui.ViewPointCloud(df_info, accum_pointcloud, PARM_LIDAR)

    def EndCalibration(self):
        self.progress_status = CONST_STOP
        self.ui.tabs.setTabEnabled(CONST_UNSUPERVISED_TAB, True)
        self.ui.tabs.setTabEnabled(CONST_EVALUATION_TAB, True)
        self.ui.handeye.complete_calibration = True

        self.ui.unsupervised_tab.select_principle_sensor_list_layout.AddWidgetItem(self.ui.config.PARM_LIDAR['SensorList'], self.ui.handeye.PARM_LIDAR['CheckedSensorList'])
        self.ui.ResetResultsLabels(self.ui.handeye.PARM_LIDAR)
        self.ui.evaluation_tab.eval_lidar['CheckedSensorList'] = copy.deepcopy(self.ui.handeye.PARM_LIDAR['CheckedSensorList'])

        df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.ui.importing,
                                                                                           self.ui.handeye_tab.using_gnss_motion,
                                                                                           self.ui.handeye.PARM_LIDAR,
                                                                                           self.ui.handeye.CalibrationParam,
                                                                                           self.ui.importing_tab.limit_time_layout.start_time,
                                                                                           self.ui.importing_tab.limit_time_layout.end_time)
        # Handeye tab
        ## Set 'Result Calibration Data'
        for idxSensor in self.ui.handeye.PARM_LIDAR['CheckedSensorList']:
            self.result_labels[idxSensor].label_edit_x.setText(format(self.ui.handeye.CalibrationParam[idxSensor][3], ".4f"))
            self.result_labels[idxSensor].label_edit_y.setText(format(self.ui.handeye.CalibrationParam[idxSensor][4], ".4f"))
            self.result_labels[idxSensor].label_edit_yaw.setText(format(self.ui.handeye.CalibrationParam[idxSensor][2] * 180 / math.pi, ".4f"))

        ## Plot 'Result Data'
        self.result_data_pose_ax.clear()
        self.ui.ViewLiDAR(self.ui.handeye.calib_x, self.ui.handeye.calib_y, self.ui.handeye.calib_yaw, self.ui.handeye.PARM_LIDAR, self.result_data_pose_ax, self.result_data_pose_canvas)

        ## Plot 'Result Graph'
        self.result_graph_ax.clear()
        self.ui.ViewPointCloud(df_info, accum_pointcloud, PARM_LIDAR, self.result_graph_ax, self.result_graph_canvas)

        ## Transfer
        self.CopyList(self.ui.handeye.CalibrationParam, self.ui.unsupervised_tab.edit_handeye_calibration_parm)


        # Unsupervised tab
        ## Set 'Unsupervised Initial Value'
        for idxSensor in self.ui.handeye.PARM_LIDAR['CheckedSensorList']:
            self.ui.unsupervised_tab.handeye_result_labels[idxSensor].double_spin_box_x.setValue(self.ui.handeye.CalibrationParam[idxSensor][3])
            self.ui.unsupervised_tab.handeye_result_labels[idxSensor].double_spin_box_y.setValue(self.ui.handeye.CalibrationParam[idxSensor][4])
            self.ui.unsupervised_tab.handeye_result_labels[idxSensor].double_spin_box_yaw.setValue(self.ui.handeye.CalibrationParam[idxSensor][2] * 180 / math.pi)

        ## Transfer
        self.ui.unsupervised.CalibrationParam = copy.deepcopy(self.ui.handeye.CalibrationParam)


        # Evaluation tab
        ## Set 'Select The Method'
        for idxSensor in self.ui.handeye.PARM_LIDAR['CheckedSensorList']:
            self.ui.evaluation_tab.userinterface_labels[idxSensor].button_group.button(CONST_HANDEYE).setChecked(True)
            self.ui.evaluation_tab.userinterface_labels[idxSensor].prev_checkID = CONST_HANDEYE
            self.ui.evaluation_tab.userinterface_labels[idxSensor].spinbox1.setValue(self.ui.handeye.CalibrationParam[idxSensor][3])
            self.ui.evaluation_tab.userinterface_labels[idxSensor].spinbox2.setValue(self.ui.handeye.CalibrationParam[idxSensor][4])
            self.ui.evaluation_tab.userinterface_labels[idxSensor].spinbox3.setValue(self.ui.handeye.CalibrationParam[idxSensor][2] * 180 / math.pi)
            self.ui.evaluation_tab.custom_calibration_param[idxSensor] = copy.deepcopy(self.ui.handeye.CalibrationParam[idxSensor])

    def CopyList(self, source, target):
        keys = list(source.keys())
        values = list(source.values())

        for i in range(len(keys)):
            target[keys[i]] = values[i].copy()

class UnsupervisedTab(CalibrationTab):
    def __init__(self, ui):
        super().__init__(ui)
        self.edit_handeye_calibration_parm = {}
        self.user_define_initial_labels = {}
        self.handeye_result_labels = {}

    ## Groupbox
    def Configuration_SetConfiguration_Groupbox(self):
        self.optimization_configuration_groupbox = QGroupBox('Set Configuration')
        vbox = QVBoxLayout()

        liDAR_configuration_label = QLabel('[ LiDAR Configuration ]', self)
        vbox.addWidget(liDAR_configuration_label)

        self.select_principle_sensor_list_layout = element.CheckBoxListLayout(self.ui, 'Select Principle Sensor List')
        vbox.addLayout(self.select_principle_sensor_list_layout)

        liDAR_configuration_label = QLabel('[ Unsupervised Configuration ]', self)
        vbox.addWidget(liDAR_configuration_label)

        self.point_sampling_ratio_layout = element.DoubleSpinBoxLabelLayout('Point Sampling Ratio', self.ui)
        vbox.addLayout(self.point_sampling_ratio_layout)

        self.num_points_plane_modeling_layout = element.SpinBoxLabelLayout('Num Points Plane Modeling', self.ui)
        vbox.addLayout(self.num_points_plane_modeling_layout)

        self.outlier_distance_layout = element.DoubleSpinBoxLabelLayout('Outlier Distance [m]', self.ui)
        vbox.addLayout(self.outlier_distance_layout)

        optimization_initial_value_label = QLabel('[ Unsupervised Initial Value ]', self)
        vbox.addWidget(optimization_initial_value_label)

        self.optimization_initial_value_tab = element.ResultTab(self.ui)
        vbox.addLayout(self.optimization_initial_value_tab)

        self.optimization_configuration_groupbox.setLayout(vbox)
        return self.optimization_configuration_groupbox

    def Configuration_Calibration_Groupbox(self):
        groupbox = QGroupBox('Unsupervised Calibration')
        vbox = QVBoxLayout(self)

        hbox = QHBoxLayout(self)
        btn = QPushButton('Start')
        btn.clicked.connect(lambda: self.StartCalibration(CONST_UNSUPERVISED,
                                                          self.ui.unsupervised.Calibration,
                                                          self.ui.config.PARM_IM['VehicleSpeedThreshold'],
                                                          self.ui.importing_tab.limit_time_layout.start_time,
                                                          self.ui.importing_tab.limit_time_layout.end_time,
                                                          self.ui.config.PARM_LIDAR,
                                                          [self.text_edit.clear],
                                                          [self.text_edit.append],
                                                          self.EndCalibration))
        hbox.addWidget(btn)

        self.stop_btn = QPushButton('Stop')
        self.stop_btn.clicked.connect(self.StopCalibartion)
        hbox.addWidget(self.stop_btn)
        vbox.addLayout(hbox)

        label = QLabel('[ Unsupervised Progress ]')
        # label.setFont(QFont('MSGOTHIC',10))
        vbox.addWidget(label)

        self.text_edit = QTextEdit()
        self.text_edit.setFont(QFont('MSGOTHIC',10))
        vbox.addWidget(self.text_edit)

        self.scroll_box = ScrollAreaV()
        vbox.addWidget(self.scroll_box)

        groupbox.setLayout(vbox)
        return groupbox

    ## Callback func

    def StopCalibartion(self):
        self.progress_status = CONST_STOP

        self.ui.opti_thread.toggle_status()

        self.ui.ErrorPopUp('Please wait for stop the calibration')

    def ViewLiDAR(self):
        if self.progress_status is not CONST_STOP:
            return False
        self.ui.ViewLiDAR(self.ui.unsupervised.calib_x, self.ui.unsupervised.calib_y, self.ui.unsupervised.calib_yaw, self.ui.config.PARM_LIDAR)

    def ViewPointCloud(self):
        if self.progress_status is not CONST_STOP:
            return False
        df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.ui.importing,
                                                                                           self.ui.unsupervised_tab.using_gnss_motion,
                                                                                           self.ui.unsupervised.PARM_LIDAR,
                                                                                           self.ui.unsupervised.CalibrationParam,
                                                                                           self.ui.importing_tab.limit_time_layout.start_time,
                                                                                           self.ui.importing_tab.limit_time_layout.end_time)

        self.ui.ViewPointCloud(df_info, accum_pointcloud, PARM_LIDAR)

    def EndCalibration(self):
        self.progress_status = CONST_STOP
        self.ui.unsupervised.complete_calibration = True

        df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.ui.importing,
                                                                                           self.ui.unsupervised_tab.using_gnss_motion,
                                                                                           self.ui.unsupervised.PARM_LIDAR,
                                                                                           self.ui.unsupervised.CalibrationParam,
                                                                                           self.ui.importing_tab.limit_time_layout.start_time,
                                                                                           self.ui.importing_tab.limit_time_layout.end_time)

        # Unsupervised tab

        ## Set 'Result Calibration Data'
        for idxSensor in self.ui.unsupervised.PARM_LIDAR['CheckedSensorList']:
            self.result_labels[idxSensor].label_edit_x.setText(format(self.ui.unsupervised.CalibrationParam[idxSensor][3], ".4f"))
            self.result_labels[idxSensor].label_edit_y.setText(format(self.ui.unsupervised.CalibrationParam[idxSensor][4], ".4f"))
            self.result_labels[idxSensor].label_edit_yaw.setText(format(self.ui.unsupervised.CalibrationParam[idxSensor][2] * 180 / math.pi, ".4f"))

        ## Plot 'Result Data'
        self.result_data_pose_ax.clear()
        self.ui.ViewLiDAR(self.ui.unsupervised.calib_x, self.ui.unsupervised.calib_y, self.ui.unsupervised.calib_yaw, self.ui.unsupervised.PARM_LIDAR, self.result_data_pose_ax, self.result_data_pose_canvas)

        ## Plot 'Result Graph''
        self.result_graph_ax.clear()
        self.ui.ViewPointCloud(df_info, accum_pointcloud, PARM_LIDAR, self.result_graph_ax, self.result_graph_canvas)

        # Evaluation tab

        for idxSensor in self.ui.unsupervised.PARM_LIDAR['CheckedSensorList']:
            self.ui.evaluation_tab.userinterface_labels[idxSensor].button_group.button(CONST_UNSUPERVISED).setChecked(True)
            self.ui.evaluation_tab.userinterface_labels[idxSensor].prev_checkID = CONST_UNSUPERVISED
            self.ui.evaluation_tab.userinterface_labels[idxSensor].spinbox1.setValue(self.ui.unsupervised.CalibrationParam[idxSensor][3])
            self.ui.evaluation_tab.userinterface_labels[idxSensor].spinbox2.setValue(self.ui.unsupervised.CalibrationParam[idxSensor][4])
            self.ui.evaluation_tab.userinterface_labels[idxSensor].spinbox3.setValue(self.ui.unsupervised.CalibrationParam[idxSensor][2] * 180 / math.pi)
            self.ui.evaluation_tab.custom_calibration_param[idxSensor] = copy.deepcopy(self.ui.unsupervised.CalibrationParam[idxSensor])

    def CopyList(self, source, target):
        keys = list(source.keys())
        values = list(source.values())

        for i in range(len(keys)):
            target[keys[i]] = values[i].copy()

class EvaluationTab(QWidget):
    def __init__(self, ui):
        super().__init__()
        self.ui = ui
        self.prev_checkID = 0
        self.evaluation_status = CONST_STOP
        self.custom_calibration_param = {}

        self.userinterface_labels = {}
        self.eval_lidar = {}
        self.eval_calibration_param = {}
        self.eval_calib_x = []
        self.eval_calib_y = []
        self.eval_calib_yaw = []

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
        groupbox = QGroupBox('Select The Method')
        vbox = QVBoxLayout()

        hbox = QHBoxLayout()
        label = QLabel()
        hbox.addWidget(label, 10)
        label = QLabel('HandEye')
        hbox.addWidget(label, 10)
        label = QLabel('Unsupervised')
        hbox.addWidget(label, 10)
        label = QLabel('Custom')
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

        self.sampling_interval_layout = element.SpinBoxLabelLayout('Eval Sampling Interval [Count]', self.ui)
        vbox.addLayout(self.sampling_interval_layout)

        self.time_speed_threshold_layout = element.DoubleSpinBoxLabelLayout('Eval Vehicle Minimum Speed [km/h]', self.ui)
        vbox.addLayout(self.time_speed_threshold_layout)

        groupbox.setLayout(vbox)
        return groupbox

    def UserInterface_LimitTime_Groupbox(self):
        groupbox = QGroupBox('Limit Time')
        vbox = QVBoxLayout()

        self.limit_time_layout = element.SlideLabelLayouts(self.ui)
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

        self.eval_data_pose_fig = plt.figure()
        self.eval_data_pose_canvas = FigureCanvas(self.eval_data_pose_fig)
        self.eval_data_pose_ax = self.eval_data_pose_fig.add_subplot(1, 1, 1)
        self.eval_data_pose_ax.grid()
        self.eval_data_pose_canvas.draw()
        vbox.addWidget(self.eval_data_pose_canvas)

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
        self.ui.ViewLiDAR(self.eval_calib_x, self.eval_calib_y, self.eval_calib_yaw, self.eval_lidar)

    def ViewPointCloud(self):
        eval_df_info = copy.deepcopy(self.ui.evaluation.df_info)
        eval_Map = copy.deepcopy(self.ui.evaluation.Map)

        self.ui.ViewPointCloud(eval_df_info, eval_Map, self.eval_lidar)

    def StartBtn(self):
        if self.ui.config_tab.is_lidar_num_changed == True:
            self.ui.ErrorPopUp('Please import after changing lidar number')
            return False

        is_handeye_selected = False
        is_optimization_selected = False
        for idxSensor in self.eval_lidar['CheckedSensorList']:
            method = self.userinterface_labels[idxSensor].button_group.checkedId()
            if method == CONST_HANDEYE:
                is_handeye_selected = True
            elif method == CONST_UNSUPERVISED:
                is_optimization_selected = True
        if not self.ui.handeye.complete_calibration:
            if is_handeye_selected:
                self.ui.ErrorPopUp('Please complete the HandEye calibration')
                return False
        elif not self.ui.unsupervised.complete_calibration:
            if is_optimization_selected:
                self.ui.ErrorPopUp('Please complete the Opimization calibration')
                return False

        for idxSensor in self.eval_lidar['CheckedSensorList']:
            if self.ui.importing.PointCloudSensorList.get(idxSensor) is None:
                self.ui.ErrorPopUp('Import pointcloud {}'.format(idxSensor))
                return False
        if self.evaluation_status is not CONST_STOP:
            return False
        self.evaluation_status = CONST_PLAY

        self.eval_calibration_param.clear()
        self.eval_calib_x.clear()
        self.eval_calib_y.clear()
        self.eval_calib_yaw.clear()
        for idxSensor in self.eval_lidar['CheckedSensorList']:
            method = self.userinterface_labels[idxSensor].button_group.checkedId()

            if method == CONST_HANDEYE:
                self.eval_calibration_param[idxSensor] = copy.deepcopy(self.ui.handeye.CalibrationParam[idxSensor])
                self.eval_calib_x.append(self.ui.handeye.CalibrationParam[idxSensor][3])
                self.eval_calib_y.append(self.ui.handeye.CalibrationParam[idxSensor][4])
                self.eval_calib_yaw.append(self.ui.handeye.CalibrationParam[idxSensor][2] * 180/math.pi)
            elif method == CONST_UNSUPERVISED:
                self.eval_calibration_param[idxSensor] = copy.deepcopy(self.ui.unsupervised.CalibrationParam[idxSensor])
                self.eval_calib_x.append(self.ui.unsupervised.CalibrationParam[idxSensor][3])
                self.eval_calib_y.append(self.ui.unsupervised.CalibrationParam[idxSensor][4])
                self.eval_calib_yaw.append(self.ui.unsupervised.CalibrationParam[idxSensor][2] * 180 / math.pi)
            elif method == CONST_CUSTOM:
                self.eval_calibration_param[idxSensor] = copy.deepcopy(self.custom_calibration_param[idxSensor])
                self.eval_calib_x.append(self.custom_calibration_param[idxSensor][3])
                self.eval_calib_y.append(self.custom_calibration_param[idxSensor][4])
                self.eval_calib_yaw.append(self.custom_calibration_param[idxSensor][2] * 180 / math.pi)

        self.StartEvaluation(self.limit_time_layout.start_time,
                             self.limit_time_layout.end_time,
                             self.eval_lidar,
                             self.eval_calibration_param,
                             [self.text_edit.clear, self.pbar.reset],
                             self.text_edit.append,
                             self.pbar.setValue,
                             self.EndEvaluation)

    def StopBtn(self):
        self.ui.thread.toggle_status()
        self.evaluation_status = CONST_STOP

    def StartEvaluation(self, start_time, end_time, sensor_list, calibration_param, targets_clear, text_edit_callback, progress_callback, end_callback):
        self.ui.thread._status = True

        if self.ui.handeye.complete_calibration:
            using_gnss_motion = self.ui.handeye_tab.using_gnss_motion
        elif self.ui.unsupervised.complete_calibration:
            using_gnss_motion = self.ui.unsupervised_tab.using_gnss_motion

        self.ui.thread.SetFunc(self.ui.evaluation.Evaluation, [start_time, end_time, sensor_list, calibration_param, using_gnss_motion])
        try:
            self.ui.thread.change_value.disconnect()
        except:
            pass
        try:
            self.ui.thread.iteration_percentage.disconnect()
        except:
            pass
        try:
            self.ui.thread.end.disconnect()
        except:
            pass
        try:
            self.ui.thread.emit_string.disconnect()
        except:
            pass

        for target_clear in targets_clear:
            target_clear()

        self.ui.thread.emit_string.connect(text_edit_callback)
        self.ui.thread.change_value.connect(progress_callback)
        self.ui.thread.end.connect(end_callback)
        self.ui.thread.start()

    def EndEvaluation(self):
        self.evaluation_status = CONST_STOP

        ## Plot 'Result Graph'
        self.eval_graph_ax.clear()
        self.ui.ViewPointCloud(self.ui.evaluation.df_info,
                               self.ui.evaluation.Map,
                               self.ui.evaluation.PARM_LIDAR,
                               self.eval_graph_ax,
                               self.eval_graph_canvas)

        ## Plot 'Lidar Position Result of Calibration'
        self.eval_data_pose_ax.clear()
        self.ui.ViewLiDAR(self.eval_calib_x, self.eval_calib_y, self.eval_calib_yaw, self.ui.evaluation.PARM_LIDAR, self.eval_data_pose_ax, self.eval_data_pose_canvas)

        ## Plot 'Result RMSE'
        self.eval_rmse_xy_ax.clear()
        self.eval_rmse_yaw_ax.clear()
        self.ui.ViewRMSE(self.ui.evaluation.rmse_x,
                         self.ui.evaluation.rmse_y,
                         self.ui.evaluation.rmse_yaw,
                         self.ui.evaluation.lidarlist,
                         self.ui.evaluation.PARM_LIDAR,
                         self.eval_rmse_xy_ax,
                         self.eval_rmse_yaw_ax,
                         self.eval_rmse_canvas)
        
        print('end evaluation')

class MyApp(QMainWindow):
    def __init__(self, parent=None):
        super(MyApp, self).__init__(parent)
        self.form_widget = FormWidget(self)
        self.setWindowIcon(QIcon(self.form_widget.config.PATH['Image_path'] + 'exe_icon.ico'))

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

    def center(self): # ?????? ????????? ????????? ???????????? ???????????? ??? ??? ????????????????????????.
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
                    self.form_widget.config.WriteFile(fname[0])
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
                    self.form_widget.config.WriteFile(fname[0])
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
        is_validation = False
        is_handeye = False
        is_single_optimization = False
        is_multi_optimization = False
        is_import = False
        is_evaluation = False
        is_path = False

        for line in fileinput.input(self.form_widget.config.configuration_file, inplace=True):
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
            elif '[Validation]' in line:
                is_validation = True
                is_handeye = False
            elif ('MaximumIteration' in line) and is_validation:
                line = line.replace(line, 'MaximumIteration = ' + str(self.form_widget.config.PARM_DV['MaximumIteration']) + '\n')
            elif ('Tolerance' in line) and is_validation:
                line = line.replace(line, 'Tolerance = ' + str(self.form_widget.config.PARM_DV['Tolerance']) + '\n')
            elif ('OutlierDistance_m' in line) and is_validation:
                line = line.replace(line, 'OutlierDistance_m = ' + str(self.form_widget.config.PARM_DV['OutlierDistance_m']) + '\n')
            elif ('filter_HeadingThreshold' in line) and is_validation:
                line = line.replace(line, 'filter_HeadingThreshold = ' + str(self.form_widget.config.PARM_DV['filter_HeadingThreshold']) + '\n')
            elif ('filter_DistanceThreshold' in line) and is_validation:
                line = line.replace(line, 'filter_DistanceThreshold = ' + str(self.form_widget.config.PARM_DV['filter_DistanceThreshold']) + '\n')                
            elif '[Handeye]' in line:
                is_handeye = True
                is_validation = False
            elif ('MaximumIteration' in line) and is_handeye:
                line = line.replace(line, 'MaximumIteration = ' + str(self.form_widget.config.PARM_HE['MaximumIteration']) + '\n')
            elif ('Tolerance' in line) and is_handeye:
                line = line.replace(line, 'Tolerance = ' + str(self.form_widget.config.PARM_HE['Tolerance']) + '\n')
            elif ('OutlierDistance_m' in line) and is_handeye:
                line = line.replace(line, 'OutlierDistance_m = ' + str(self.form_widget.config.PARM_HE['OutlierDistance_m']) + '\n')
            elif ('filter_HeadingThreshold' in line) and is_handeye:
                line = line.replace(line, 'filter_HeadingThreshold = ' + str(self.form_widget.config.PARM_HE['filter_HeadingThreshold']) + '\n')
            elif ('filter_DistanceThreshold' in line) and is_handeye:
                line = line.replace(line, 'filter_DistanceThreshold = ' + str(self.form_widget.config.PARM_HE['filter_DistanceThreshold']) + '\n')
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
            elif '[Path]' in line:
                is_path = True
            elif ('Logging_file_path' in line) and is_path:
                line = line.replace(line, 'Logging_file_path = ' + str(self.form_widget.importing_tab.logging_file_path_layout.path_file_str + '\n'))
            sys.stdout.write(line)
        self.form_widget.value_changed = False

        print('Save ' + str(self.form_widget.config.configuration_file))

    def SaveAsInitFile(self):
        fname = self.SaveDialog()
        if fname[0]:
            self.form_widget.config.configuration_file = fname[0]
            self.form_widget.config.WriteFile(fname[0])
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
                    self.form_widget.config.WriteFile(fname[0])
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
        self.validation_thread = QThread.Thread()
        self.handeye_thread = QThread.Thread()
        self.opti_thread = QThread.Thread()

        ### setting configuration file structure each taps default path of each parameters defining in config.WriteDefaultFile() and initialize in InitConfiguration()
        self.config = v1_step1_configuration.Configuration()
        self.importing = v1_step2_import_data.Import(self.config)
        self.datavalidation = v1_step2_2_data_validation.DataValidation(self.config, self.importing)
        self.handeye = v1_step3_handeye.HandEye(self.config, self.importing)
        self.unsupervised = v1_step4_unsupervised.Unsupervised(self.config, self.importing)
        self.evaluation = v1_step5_evaluation.Evaluation(self.config, self.importing)
        self.config.WriteDefaultFile()
        self.config.InitConfiguration()

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
        self.datavalidation_tab = DataValidation(self)
        self.handeye_tab = HandEyeTab(self)
        self.unsupervised_tab = UnsupervisedTab(self)
        self.evaluation_tab = EvaluationTab(self)

        self.tabs.addTab(self.config_tab, '1. Configuration')
        self.tabs.setTabEnabled(CONST_CONFIG_TAB, True)

        self.tabs.addTab(self.importing_tab, '2.1. Import Data')
        self.tabs.setTabEnabled(CONST_IMPORTDATA_TAB, False)
        
        self.tabs.addTab(self.datavalidation_tab, '2.2. Data Validation')
        self.tabs.setTabEnabled(CONST_VALIDATION_TAB, False)

        self.tabs.addTab(self.handeye_tab, '3. HandEye')
        self.tabs.setTabEnabled(CONST_HANDEYE_TAB, False)

        self.tabs.addTab(self.unsupervised_tab, '4. Unsupervised')
        self.tabs.setTabEnabled(CONST_UNSUPERVISED_TAB, False)

        self.tabs.addTab(self.evaluation_tab, '5. Evaluation')
        self.tabs.setTabEnabled(CONST_EVALUATION_TAB, False)

        ### Set Basic Window
        self.hbox.addWidget(self.tabs)
        self.setLayout(self.hbox)
        self.setGeometry(300, 100, 1000, 600)

    def SetConfiguration(self):
        ### Setting configuration tab
        self.config_tab.lidar_num_label_layout.spin_box.setValue(len(self.config.PARM_LIDAR['SensorList']))
        self.config_tab.select_using_sensor_list_layout.AddWidgetItem(self.config.PARM_LIDAR['SensorList'], self.config.PARM_LIDAR['CheckedSensorList'])

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
        self.importing_tab.logging_file_path_layout.label_edit.setText(self.config.PATH['Logging_file_path'])
        self.importing_tab.logging_file_path_layout.path_file_str = self.config.PATH['Logging_file_path']
        self.importing_tab.sampling_interval_layout.spin_box.setValue(PARM_IM['SamplingInterval'])
        self.importing_tab.time_speed_threshold_layout.double_spin_box.setValue(PARM_IM['VehicleSpeedThreshold'])

        ### Setting data validation tab
        PARM_DV = self.config.PARM_DV
        self.datavalidation_tab.maximum_interation_layout.spin_box.setValue(PARM_DV['MaximumIteration'])
        self.datavalidation_tab.tolerance_layout.double_spin_box.setValue(PARM_DV['Tolerance'])
        self.datavalidation_tab.outlier_distance_layout.double_spin_box.setValue(PARM_DV['OutlierDistance_m'])
        self.datavalidation_tab.heading_threshold_layout.double_spin_box.setValue(PARM_DV['filter_HeadingThreshold'])
        self.datavalidation_tab.distance_threshold_layout.double_spin_box.setValue(PARM_DV['filter_DistanceThreshold'])

        ### Setting handeye tab
        PARM_HE = self.config.PARM_HE
        self.handeye_tab.maximum_interation_layout.spin_box.setValue(PARM_HE['MaximumIteration'])
        self.handeye_tab.tolerance_layout.double_spin_box.setValue(PARM_HE['Tolerance'])
        self.handeye_tab.outlier_distance_layout.double_spin_box.setValue(PARM_HE['OutlierDistance_m'])
        self.handeye_tab.heading_threshold_layout.double_spin_box.setValue(PARM_HE['filter_HeadingThreshold'])
        self.handeye_tab.distance_threshold_layout.double_spin_box.setValue(PARM_HE['filter_DistanceThreshold'])

        ### Setting unsupervised tab
        PARM_MO = self.config.PARM_MO
        self.unsupervised_tab.point_sampling_ratio_layout.double_spin_box.setValue(PARM_MO['PointSamplingRatio'])
        self.unsupervised_tab.num_points_plane_modeling_layout.spin_box.setValue(PARM_MO['NumPointsPlaneModeling'])
        self.unsupervised_tab.outlier_distance_layout.double_spin_box.setValue(PARM_MO['OutlierDistance_m'])
        self.unsupervised_tab.select_principle_sensor_list_layout.AddWidgetItem(self.config.PARM_LIDAR['SensorList'], self.config.PARM_LIDAR['CheckedSensorList'])

        ### Setting evaluation tab
        PARM_EV = self.config.PARM_EV
        self.evaluation_tab.eval_lidar['CheckedSensorList'] = copy.deepcopy(self.config.PARM_LIDAR['CheckedSensorList'])
        self.evaluation_tab.sampling_interval_layout.spin_box.setValue(PARM_EV['SamplingInterval'])
        self.evaluation_tab.time_speed_threshold_layout.double_spin_box.setValue(PARM_EV['VehicleSpeedThreshold'])

        print('Set all tab\'s configuration')

    def ResetResultsLabels(self, PARM_LIDAR):
        self.ResetValidationLabel(CONST_UNEDITABLE_LABEL, PARM_LIDAR, self.datavalidation_tab.scroll_box.layout, self.datavalidation_tab.result_labels,
                               self.datavalidation.RMSETranslationErrorDict, self.datavalidation.RMSERotationErrorDict, self.datavalidation_tab.label_heading_threshold, self.datavalidation_tab.label_distance_threshold)
        # reset handeye calibration result
        self.ResetResultsLabel(CONST_UNEDITABLE_LABEL, PARM_LIDAR, self.handeye_tab.scroll_box.layout, self.handeye_tab.result_labels,
                               self.handeye.CalibrationParam)
        # reset unsupervised calibration result
        self.ResetResultsLabel(CONST_UNEDITABLE_LABEL, PARM_LIDAR, self.unsupervised_tab.scroll_box.layout, self.unsupervised_tab.result_labels,
                               self.unsupervised.CalibrationParam)

        # reset evaluation select method
        self.ResetResultsLabel(CONST_EVAULATION_LABEL, PARM_LIDAR, self.evaluation_tab.scroll_box.layout,
                               self.evaluation_tab.userinterface_labels,
                               self.handeye.CalibrationParam)

        # reset unsupervised initial value of handeye
        self.ResetResultsLabel(CONST_EDITABLE_LABEL2, PARM_LIDAR, self.unsupervised_tab.optimization_initial_value_tab.handeye_scroll_box.layout,
                               self.unsupervised_tab.handeye_result_labels,
                               self.unsupervised_tab.edit_handeye_calibration_parm)
        # reset unsupervised initial value of custom
        self.ResetResultsLabel(CONST_EDITABLE_LABEL2, PARM_LIDAR, self.unsupervised_tab.optimization_initial_value_tab.user_define_scroll_box.layout,
                               self.unsupervised_tab.user_define_initial_labels,
                               self.config.CalibrationParam)

        

    def ResetResultsLabel(self, label_type, PARM_LIDAR, layout, labels, calibration_param):
        self.RemoveLayout(layout)
        labels.clear()
        for idxSensor in PARM_LIDAR['CheckedSensorList']:
            if label_type is CONST_UNEDITABLE_LABEL:
                result_label = element.CalibrationResultLabel(idxSensor)
                if calibration_param.get(idxSensor) is not None:
                    result_label.label_edit_x.setText(str(round(calibration_param[idxSensor][3], 4)))
                    result_label.label_edit_y.setText(str(round(calibration_param[idxSensor][4], 4)))
                    result_label.label_edit_yaw.setText(str(round(calibration_param[idxSensor][2] * 180.0 / math.pi, 4)))
            elif label_type is CONST_EVAULATION_LABEL:
                result_label = element.EvaluationLable(idxSensor, self)

            elif label_type is CONST_EDITABLE_LABEL2:
                result_label = element.CalibrationResultEditLabel2(idxSensor, calibration_param, self)
                if calibration_param.get(idxSensor) is not None:
                    result_label.double_spin_box_x.setValue(calibration_param[idxSensor][3])
                    result_label.double_spin_box_y.setValue(calibration_param[idxSensor][4])
                    result_label.double_spin_box_yaw.setValue(calibration_param[idxSensor][2] * 180.0 / math.pi)
            labels[idxSensor] = result_label
            layout.addLayout(result_label)
        layout.addStretch(1)
        
        
        
    def ResetValidationLabel(self, label_type, PARM_LIDAR, layout, labels, RMSETranslationError, RMSERotationError, heading_threshold, distance_threshold):
        self.RemoveLayout(layout)
        labels.clear()
        
        
        #result_config_label = element.ValidationConfigLabel(heading_threshold, distance_threshold)
        #result_config_label = element.ValidationConfigLabel(3,3)
        
        #layout.addLayout(result_config_label)
        
        
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

    def ViewLiDAR(self, calib_x, calib_y, calib_yaw, PARM_LIDAR,  ax=None, canvas=None):
        lidar_num = len(PARM_LIDAR['CheckedSensorList'])
        column = '2'
        row = str(math.ceil(lidar_num / 2))
        fig = plt.figure(figsize=(16, 12), dpi=70)
        veh_path = self.config.PATH['Image_path'] + 'vehicle2.png'
        # veh = plt.imread(veh_path)
        # Open vehicle image
        veh = Image.open(veh_path)
        veh2 = veh.resize((1100, 1100))
        veh = np.asarray(veh2)

        # veh = plt.imread(veh_path)
        # veh = plt.set_size_inches(18.5, 10.5, forward=True)
        for i in range(len(PARM_LIDAR['CheckedSensorList'])):
            idxSensor = list(PARM_LIDAR['CheckedSensorList'])
            if canvas is None:
                plot_num_str = column + row + str(i + 1)
                ax = fig.add_subplot(int(plot_num_str))

            # T = np.array([[np.cos(calib_yaw[i]), -np.sin(calib_yaw[i]), calib_x[i]],[np.sin(calib_yaw[i]), np.cos(calib_yaw[i]), calib_y[i]],[0,0,1]])
            # inv_T = np.linalg.inv(T)
            # calib_x[i] = inv_T[0][2]
            # calib_y[i] = inv_T[1][2]
            # calib_yaw[i] = np.arctan2(inv_T[1, 0], inv_T[0, 0])
            # x = int(calib_x[i]) * 200 + 520
            # y = 1000 - 1 * int(calib_y[i]) * 200 - 500
            x = -calib_y[i] * 120 + 542
            y = 725 - 1 * calib_x[i] * 160
            # car_length = 1.75
            lidar_num = 'lidar' + str(idxSensor[i])
            ax.scatter(x, y, s=300, label=lidar_num, color=self.color_list[(idxSensor[i]) % len(self.color_list)],
                       edgecolor='none', alpha=0.5)

            s = 'x[m]: ' + str(round(calib_x[i], 4)) + '\ny[m]: ' + str(round(calib_y[i], 4)) + '\nyaw[deg]: ' + str(
                round(calib_yaw[i], 4))

            if canvas is None:
                if calib_y[i] > 0:
                    ax.text(x, y + 300, s, fontsize=7)
                else:
                    ax.text(x, y - 70, s, fontsize=7)

            ax.arrow(x, y, 100 * np.cos((calib_yaw[i] + 90) * np.pi / 180),
                     -100 * np.sin((calib_yaw[i] + 90) * np.pi / 180), head_width=10,
                     head_length=10,
                     fc='k', ec='k')
            ax.plot(np.linspace(542, x, 100), np.linspace(725, y, 100),
                    self.color_list[(idxSensor[i]) % len(self.color_list)] + '--')

            if canvas is None:
                ax.imshow(veh)
                ax.set_xlim([-460, 1540])
                ax.set_ylim([1000, 200])
                ax.grid()
                ax.legend()
                ax.set_title('Result of calibration - LiDAR' + str(idxSensor[i]))
                ax.axes.xaxis.set_visible(False)
                ax.axes.yaxis.set_visible(False)

        if canvas is not None:
            ax.imshow(veh)
            ax.set_xlim([-460, 1540])
            ax.set_ylim([1000, 200])
            ax.grid()
            ax.legend()
            ax.set_title('Result of calibration')
            ax.axes.xaxis.set_visible(False)
            ax.axes.yaxis.set_visible(False)
            canvas.draw()
        else:
            root = Tk.Tk()
            canvas = FigureCanvasTkAgg(fig, master=root)
            nav = NavigationToolbar2Tk(canvas, root)
            canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            canvas._tkcanvas.pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            root.mainloop()

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


            strColIndex = 'XYZRGB_' + str(idxSensor[i])
            ax.plot(error[i], color=self.color_list[(idxSensor[i]) % len(self.color_list)], label='LiDAR' + str(idxSensor[i]))

            if canvas is None:
                ax.axis('equal')
                ax.grid()
                ax.legend(markerscale=2)
                ax.set_title('Translation RMSE between Vehicle and LiDAR' + str(idxSensor[i]))
                ax.set_xlabel('Step [cnt]')
                ax.set_ylabel('RMSE [m]')
                ax.set_xlim([-10,len(error[i])+10])
                ax.set_ylim([-1,1])
                #plt.axis('scaled')


        if canvas is not None:
            ax.axis('equal')
            ax.grid()
            ax.legend(markerscale=2)
            ax.set_title('Translation RMSE')
            ax.set_xlabel('Step [cnt]')
            ax.set_ylabel('RMSE [m]')
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
        #fig = plt.figure(figsize=(16, 12), dpi=70)
        fig = plt.figure(figsize=(16, 12))
        
        #if ax is not None:

        for i in range(len(PARM_LIDAR['CheckedSensorList'])):
            idxSensor = list(PARM_LIDAR['CheckedSensorList'])
             
            if canvas is None:
                plot_num_str = column + row + str(i + 1)
                ax = fig.add_subplot(int(plot_num_str))

            strColIndex = 'XYZRGB_' + str(idxSensor[i])
            ax.plot(error[i], color=self.color_list[(idxSensor[i]) % len(self.color_list)], label='LiDAR' + str(idxSensor[i]))

            if canvas is None:
                ax.axis('equal')
                ax.grid()
                ax.legend(markerscale=2)
                ax.set_title('Rotation RMSE between Vehicle and LiDAR' + str(idxSensor[i]))
                ax.set_xlabel('Step [cnt]')
                ax.set_ylabel('RMSE [deg]')
                ax.set_xlim([-10,len(error[i])+10])
                ax.set_ylim([-1,1])
                #plt.axis('scaled')
                #ax.set_ylim([-np.trunc(np.max(error[i])) -1, np.trunc(np.max(error[i]))+1])
                #plt.ylim([-np.trunc(np.max(error[i])) -1, np.trunc(np.max(error[i]))+1])
                print(np.trunc(np.max(error[i]))*1.5)
                
        if canvas is not None:
            ax.axis('equal')
            ax.grid()
            ax.legend(markerscale=2)
            ax.set_title('Rotation RMSE')
            ax.set_xlabel('Step [cnt]')
            ax.set_ylabel('RMSE [deg]')
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

            self.unsupervised_tab.select_principle_sensor_list_layout.listWidget.setFixedHeight(CONST_SCROLL_BOX_HEIGHT-40)
            self.unsupervised_tab.text_edit.setFixedHeight(CONST_SCROLL_BOX_HEIGHT)
            self.unsupervised_tab.optimization_initial_value_tab.tabs.setFixedHeight(CONST_SCROLL_BOX_HEIGHT+50)

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
        w = MyApp()
        w.show()
        sys.exit(app.exec_())

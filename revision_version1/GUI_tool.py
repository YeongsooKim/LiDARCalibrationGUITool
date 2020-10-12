# -*- coding: utf-8 -*-
"""
@author: kimkys768@gmail.com, yondoo20@gmail.com
@date: 2020-09-22
@version: 0.0.2
"""

import fileinput
import math
import sys
import copy

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

import element
from process import get_result
from process import v1_step1_configuration
from process import v1_step2_import_data
from process import v1_step3_handeye
from process import v1_step4_optimization
from process import v1_step5_evaluation
from widget.QButton import *
from widget import QThread
from widget.QScrollarea import *

import tkinter as Tk
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

CONST_CONFIG = 0
CONST_IMPORTDATA = 1
CONST_HANDEYE = 2
CONST_OPTIMIZATION = 3
CONST_EVALUATION = 4

CONST_CALIBRATION_RESULT_TAB = 0
CONST_HANDEYE_EVALUATION_TAB = 1
CONST_OPTIMIZATION_EVALUATION_TAB = 2

CONST_DISPLAY_HANDEYE = 0
CONST_DISPLAY_OPTIMIZATION = 1

CONST_NEXT_BTN_HEIGHT = 80
CONST_SCROLL_BOX_HEIGHT = 120

## label type
CONST_UNEDITABLE_LABEL = 0
CONST_EDITABLE_LABEL = 1
CONST_EDITABLE_LABEL2 = 2

## optimization-calibration status
CONST_STOP = 0
CONST_PLAY = 1
CONST_PAUSE = 2

class ConfigurationTab(QWidget):
    def __init__(self, ui):
        super().__init__()
        self.ui = ui

        self.initUi()

    def initUi(self):
        self.InterfaceLayout_v_configuration()
        self.MainLayout_h_interface()

        self.setLayout(self.interface_h_box)

    ## Layout

    def MainLayout_h_interface(self):
        self.interface_h_box = QHBoxLayout()
        self.interface_h_box.addLayout(self.configuration_v_box)

    def InterfaceLayout_v_configuration(self):
        self.configuration_v_box = QVBoxLayout()

        self.configuration_v_box.addWidget(self.Configuration_g_configuration())
        self.next_btn = QPushButton('Next step')
        self.next_btn.clicked.connect(self.NextBtn)
        self.configuration_v_box.addWidget(self.next_btn)

    ## Groupbox

    def Configuration_g_configuration(self):
        groupbox = QGroupBox('Set Configuration')
        vbox = QVBoxLayout()

        hbox = QHBoxLayout()
        hbox.addLayout(self.Configuration_g_lidar())
        hbox.addLayout(self.Configuration_g_pointcloud())
        vbox.addLayout(hbox)

        self.image_display_widget = element.ImageDisplay(self.ui.config.PATH['Image_path'])
        vbox.addWidget(self.image_display_widget)

        groupbox.setLayout(vbox)
        return groupbox

    ## Groupbox layout

    def Configuration_g_lidar(self):
        vbox = QVBoxLayout()
        liDAR_configuration_label = QLabel('[ LiDAR Configuration ]', self)
        vbox.addWidget(liDAR_configuration_label)

        self.lidar_num_label_layout = element.SpinBoxLabelLayout('LiDAR Num', self.ui)
        vbox.addLayout(self.lidar_num_label_layout)

        self.select_using_sensor_list_layout = element.CheckBoxListLayout(self.ui, 'Select Using Sensor List')
        vbox.addLayout(self.select_using_sensor_list_layout)

        return vbox

    def Configuration_g_pointcloud(self):
        vbox = QVBoxLayout()
        pointcloud_configuration_label = QLabel('[ PointCloud Configuration ]', self)
        vbox.addWidget(pointcloud_configuration_label)

        self.minimum_threshold_distance_layout = element.DoubleSpinBoxLabelLayout("Minimum Threshold Distance [m]",
                                                                                  self.ui)
        vbox.addLayout(self.minimum_threshold_distance_layout)

        self.maximum_threshold_istance_layout = element.DoubleSpinBoxLabelLayout("Maximum Threshold Distance [m]",
                                                                                 self.ui)
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
        return vbox

    ## Callback Function

    def NextBtn(self):
        # if self.select_using_sensor_list_layout.listWidget.count() == 0:
        #     self.ErrorPopUp('Please open a ini file')
        # else:
        #     self.ui.tabs.setTabEnabled(CONST_IMPORTDATA, True)
        #     self.ui.tabs.setCurrentIndex(CONST_IMPORTDATA)
        self.ui.evaluation_tab.eval_handeye_graph_ax.clear()

    def ErrorPopUp(self, error_message):
        widget = QWidget()

        qr = widget.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        widget.move(qr.topLeft())

        QMessageBox.information(widget, 'Information', error_message)
    
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

class ImportDataTab(QWidget):
    def __init__(self, ui):
        super().__init__()
        self.ui = ui

        self.initUi()

    def initUi(self):
        self.InterfaceLayout_v_setting()
        self.MainLayout_h_interface()

        self.setLayout(self.interface_h_box)

    ## Layout

    def MainLayout_h_interface(self):
        self.interface_h_box = QHBoxLayout()

        self.interface_h_box.addLayout(self.setting_v_box)

    def InterfaceLayout_v_setting(self):
        self.setting_v_box = QVBoxLayout()

        self.setting_v_box.addWidget(self.Setting_g_logging_data())
        self.setting_v_box.addStretch(1)
        self.setting_v_box.addWidget(self.Setting_g_limit_time())

        self.next_btn = QPushButton('Next step')
        self.next_btn.clicked.connect(self.NextBtn)
        self.setting_v_box.addWidget(self.next_btn)

    ## Groupbox

    def Setting_g_logging_data(self):
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

    def Setting_g_limit_time(self):
        groupbox = QGroupBox('Limit Time Data')
        vbox = QVBoxLayout()

        self.limit_time_layout = element.SlideLabelLayouts(self.ui, '[ Limit Time ]')
        vbox.addLayout(self.limit_time_layout)

        groupbox.setLayout(vbox)
        return groupbox

    ## Callback func

    def NextBtn(self):
        if self.logging_file_path_layout.pbar.value() is not 100:
            self.ErrorPopUp('Please import logging file path')
        else:
            self.ui.tabs.setTabEnabled(CONST_HANDEYE, True)
            self.ui.tabs.setCurrentIndex(CONST_HANDEYE)

    def ErrorPopUp(self, error_message):
        widget = QWidget()

        qr = widget.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        widget.move(qr.topLeft())

        QMessageBox.information(widget, 'Information', error_message)

    def InterationPercentage(self, percentage_dict):
        idxSensor = list(percentage_dict.keys())[0]
        percentage = list(percentage_dict.values())[0]
        text = 'PointCloud {}'.format(idxSensor) + ' ' + str(int(percentage)) + '%'

        self.logging_file_path_layout.lidar_buttons[idxSensor].setText(text)

    def EndImport(self):
        self.ui.importing.ResampleTime()

        parsed_pandas_dataframe = self.ui.importing.text_pointcloud
        self.logging_file_path_layout.parsed_bin = parsed_pandas_dataframe.to_string()

        default_start_time = self.ui.importing.DefaultStartTime
        default_end_time = self.ui.importing.DefaultEndTime

        # Import tab
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

        # Evaluation tab
        ## set slider default time
        self.ui.evaluation_tab.handeye_limit_time_layout.start_time_layout.slider.setMaximum(default_end_time)
        self.ui.evaluation_tab.handeye_limit_time_layout.start_time_layout.slider.setMinimum(default_start_time)
        self.ui.evaluation_tab.handeye_limit_time_layout.end_time_layout.slider.setMaximum(default_end_time)
        self.ui.evaluation_tab.handeye_limit_time_layout.end_time_layout.slider.setMinimum(default_start_time)

        ## set slider and double_spin_box value
        self.ui.evaluation_tab.handeye_limit_time_layout.end_time_layout.slider.setValue(self.ui.evaluation_tab.handeye_limit_time_layout.end_time_layout.slider.maximum())
        self.ui.evaluation_tab.handeye_limit_time_layout.end_time_layout.double_spin_box.setValue(default_end_time)
        self.ui.evaluation_tab.handeye_limit_time_layout.start_time_layout.slider.setValue(self.ui.evaluation_tab.handeye_limit_time_layout.end_time_layout.slider.minimum())
        self.ui.evaluation_tab.handeye_limit_time_layout.start_time_layout.double_spin_box.setValue(default_start_time)

        # Optimization tab
        ## set slider default time
        self.ui.evaluation_tab.optimization_limit_time_layout.start_time_layout.slider.setMaximum(default_end_time)
        self.ui.evaluation_tab.optimization_limit_time_layout.start_time_layout.slider.setMinimum(default_start_time)
        self.ui.evaluation_tab.optimization_limit_time_layout.end_time_layout.slider.setMaximum(default_end_time)
        self.ui.evaluation_tab.optimization_limit_time_layout.end_time_layout.slider.setMinimum(default_start_time)

        ## set slider and double_spin_box value
        self.ui.evaluation_tab.optimization_limit_time_layout.end_time_layout.slider.setValue(self.ui.evaluation_tab.optimization_limit_time_layout.end_time_layout.slider.maximum())
        self.ui.evaluation_tab.optimization_limit_time_layout.end_time_layout.double_spin_box.setValue(default_end_time)
        self.ui.evaluation_tab.optimization_limit_time_layout.start_time_layout.slider.setValue(self.ui.evaluation_tab.optimization_limit_time_layout.end_time_layout.slider.minimum())
        self.ui.evaluation_tab.optimization_limit_time_layout.start_time_layout.double_spin_box.setValue(default_start_time)

class CalibrationTab(QWidget):
    def __init__(self, ui):
        super().__init__()
        self.calibration_status = CONST_STOP
        self.result_labels = {}
        self.ui = ui
        self.initUi()

    def initUi(self):
        self.InterfaceLayout_v_result()
        self.InterfaceLayout_v_configuration()
        self.MainLayout_h_interface()

        self.setLayout(self.interface_h_box)

    ## Layout

    def MainLayout_h_interface(self):
        self.interface_h_box = QHBoxLayout()
        self.interface_h_box.addLayout(self.configuration_v_box)
        self.interface_h_box.addLayout(self.result_v_box)
        self.interface_h_box.setStretchFactor(self.configuration_v_box, 2);
        self.interface_h_box.setStretchFactor(self.result_v_box, 3);
        
    def InterfaceLayout_v_configuration(self):
        pass

    def InterfaceLayout_v_result(self):
        self.result_v_box = QVBoxLayout()

        self.result_v_box.addWidget(self.Result_g_result_data())
        self.result_v_box.addWidget(self.Result_g_result_graph())

    ## Groupbox

    def Configuration_g_configuration(self):
        groupbox = QGroupBox('Set Configuration')
        vbox = QVBoxLayout()

        groupbox.setLayout(vbox)
        return groupbox

    def Result_g_result_data(self):
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

    def Result_g_result_graph(self):
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

    def StartCalibration(self):
        pass

    def ViewLiDAR(self):
        pass

    def ViewPointCloud(self):
        pass

class HandEyeTab(CalibrationTab):
    def __init__(self, ui):
        super().__init__(ui)

    ## Layout

    def InterfaceLayout_v_configuration(self):
        self.configuration_v_box = QVBoxLayout()

        self.configuration_v_box.addWidget(self.Configuration_g_configuration())

        self.btn = QPushButton('Start Calibration')
        self.configuration_v_box.addWidget(self.btn)

        self.label = QLabel('[ HandEye Calibration Progress ]')
        self.configuration_v_box.addWidget(self.label)

        self.pbar = QProgressBar(self)
        self.btn.clicked.connect(lambda: self.StartCalibration(self.ui.importing_tab.limit_time_layout.start_time,
                                                               self.ui.importing_tab.limit_time_layout.end_time,
                                                               self.ui.config.PARM_LIDAR,
                                                               self.pbar.setValue))
        self.configuration_v_box.addWidget(self.pbar)

        self.configuration_v_box.addWidget(self.Configuration_g_result_calibration_data())

    ## Groupbox

    def Configuration_g_configuration(self):
        groupbox = QGroupBox('Set Configuration')
        vbox = QVBoxLayout()

        liDAR_configuration_label = QLabel('[ HandEye Configuration ]', self)
        vbox.addWidget(liDAR_configuration_label)

        self.sampling_interval_layout = element.SpinBoxLabelLayout('Sampling Interval', self.ui)
        vbox.addLayout(self.sampling_interval_layout)

        self.maximum_interation_layout = element.SpinBoxLabelLayout('Maximum Iteration', self.ui)
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

    def Configuration_g_result_calibration_data(self):
        groupbox = QGroupBox('Result Calibration Data')
        vbox = QVBoxLayout(self)

        self.scroll_box = ScrollAreaV()
        vbox.addWidget(self.scroll_box)

        groupbox.setLayout(vbox)
        return groupbox

    ## Callback func

    def StartCalibration(self, start_time, end_time, sensor_list, progress_callback_func):
        if self.calibration_status is not CONST_STOP:
            return False
        self.calibration_status = CONST_PLAY

        for idxSensor in sensor_list['CheckedSensorList']:
            if self.ui.importing.PointCloudSensorList.get(idxSensor) is None:
                self.PopUp('Import pointcloud {}'.format(idxSensor))
                return False

        self.ui.tabs.setTabEnabled(CONST_OPTIMIZATION, True)
        self.ui.tabs.setTabEnabled(CONST_EVALUATION, True)

        self.ui.thread.SetFunc(self.ui.handeye.Calibration, [start_time, end_time, sensor_list])
        try:
            self.ui.thread.change_value.disconnect()
        except:
            pass
        try:
            self.ui.thread.interation_percentage.disconnect()
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


        self.ui.thread.change_value.connect(progress_callback_func)
        self.ui.thread.end.connect(self.EndHandEyeCalibration)
        self.ui.thread.start()

    def ViewLiDAR(self):
        self.ui.ViewLiDAR(self.ui.handeye.calib_x, self.ui.handeye.calib_y, self.ui.handeye.calib_yaw)

    def ViewPointCloud(self):
        df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.ui.config,
                                                                                           self.ui.importing,
                                                                                           self.ui.handeye.CalibrationParam,
                                                                                           self.ui.importing_tab.limit_time_layout.start_time,
                                                                                           self.ui.importing_tab.limit_time_layout.end_time)
        self.ui.ViewPointCloud(df_info, accum_pointcloud, PARM_LIDAR)

    def EndHandEyeCalibration(self):
        self.calibration_status = CONST_STOP
        self.ui.tabs.setTabEnabled(CONST_OPTIMIZATION, True)

        df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.ui.config,
                                                                                           self.ui.importing,
                                                                                           self.ui.handeye.CalibrationParam,
                                                                                           self.ui.importing_tab.limit_time_layout.start_time,
                                                                                           self.ui.importing_tab.limit_time_layout.end_time)
        # Handeye tab

        ## Set 'Result Calibration Data'
        for idxSensor in self.ui.config.PARM_LIDAR['CheckedSensorList']:
            self.result_labels[idxSensor].label_edit_x.setText(str(round(self.ui.handeye.CalibrationParam[idxSensor][3], 2)))
            self.result_labels[idxSensor].label_edit_y.setText(str(round(self.ui.handeye.CalibrationParam[idxSensor][4], 2)))
            self.result_labels[idxSensor].label_edit_yaw.setText(str(round(self.ui.handeye.CalibrationParam[idxSensor][2] * 180 / math.pi, 2)))

        ## Plot 'Result Data'
        self.result_data_pose_ax.clear()
        self.ui.ViewLiDAR(self.ui.handeye.calib_x, self.ui.handeye.calib_y, self.ui.handeye.calib_yaw, self.result_data_pose_ax, self.result_data_pose_canvas)

        ## Plot 'Result Graph'
        self.result_graph_ax.clear()
        self.ui.ViewPointCloud(df_info, accum_pointcloud, PARM_LIDAR, self.result_graph_ax, self.result_graph_canvas)

        ## Transfer
        self.CopyList(self.ui.handeye.CalibrationParam, self.ui.optimization_tab.edit_handeye_calibration_parm)
        self.CopyList(self.ui.handeye.CalibrationParam, self.ui.evaluation_tab.edit_handeye_calibration_parm)


        # Optimization tab

        ## Set 'Optimization Initial Value'
        for idxSensor in self.ui.config.PARM_LIDAR['CheckedSensorList']:
            self.ui.optimization_tab.handeye_result_labels[idxSensor].double_spin_box_roll.setValue(self.ui.handeye.CalibrationParam[idxSensor][0] * 180 / math.pi)
            self.ui.optimization_tab.handeye_result_labels[idxSensor].double_spin_box_pitch.setValue(self.ui.handeye.CalibrationParam[idxSensor][1] * 180 / math.pi)
            self.ui.optimization_tab.handeye_result_labels[idxSensor].double_spin_box_yaw.setValue(self.ui.handeye.CalibrationParam[idxSensor][2] * 180 / math.pi)
            self.ui.optimization_tab.handeye_result_labels[idxSensor].double_spin_box_x.setValue(self.ui.handeye.CalibrationParam[idxSensor][3])
            self.ui.optimization_tab.handeye_result_labels[idxSensor].double_spin_box_y.setValue(self.ui.handeye.CalibrationParam[idxSensor][4])
            self.ui.optimization_tab.handeye_result_labels[idxSensor].double_spin_box_z.setValue(self.ui.handeye.CalibrationParam[idxSensor][5])

        ## Transfer
        self.ui.optimization.initial_calibration_param = copy.deepcopy(self.ui.handeye.CalibrationParam)


        # Evaluation tab

        if self.ui.evaluation_tab.button_group.checkedId() == CONST_DISPLAY_HANDEYE:
            ## Plot 'Result of Calibration'
            self.ui.evaluation_tab.result_data_pose_ax.clear()
            self.ui.ViewLiDAR(self.ui.handeye.calib_x, self.ui.handeye.calib_y, self.ui.handeye.calib_yaw, self.ui.evaluation_tab.result_data_pose_ax, self.ui.evaluation_tab.result_data_pose_canvas)

            ## Plot 'Before Aplly Calibration Result'
            self.ui.evaluation_tab.result_before_graph_ax.clear()
            self.ui.ViewPointCloud(df_info, accum_pointcloud_, PARM_LIDAR, self.ui.evaluation_tab.result_before_graph_ax, self.ui.evaluation_tab.result_before_graph_canvas)

            ## Plot 'After Aplly Calibration Result'
            self.ui.evaluation_tab.result_after_graph_ax.clear()
            self.ui.ViewPointCloud(df_info, accum_pointcloud, PARM_LIDAR, self.ui.evaluation_tab.result_after_graph_ax, self.ui.evaluation_tab.result_after_graph_canvas)

        ## Set 'Select The Method'
        for idxSensor in self.ui.config.PARM_LIDAR['CheckedSensorList']:
            self.ui.evaluation_tab.handeye_result_labels[idxSensor].double_spin_box_x.setValue(self.ui.handeye.CalibrationParam[idxSensor][3])
            self.ui.evaluation_tab.handeye_result_labels[idxSensor].double_spin_box_y.setValue(self.ui.handeye.CalibrationParam[idxSensor][4])
            self.ui.evaluation_tab.handeye_result_labels[idxSensor].double_spin_box_yaw.setValue(self.ui.handeye.CalibrationParam[idxSensor][2] * 180 / math.pi)

    def PopUp(self, message):
        widget = QWidget()

        qr = widget.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        widget.move(qr.topLeft())

        QMessageBox.information(widget, 'Information', message)

    def CopyList(self, source, target):
        keys = list(source.keys())
        values = list(source.values())

        for i in range(len(keys)):
            target[keys[i]] = values[i].copy()

class OptimizationTab(CalibrationTab):
    def __init__(self, ui):
        super().__init__(ui)
        self.edit_handeye_calibration_parm = {}
        self.user_define_initial_labels = {}
        self.handeye_result_labels = {}
    ## Layout

    def InterfaceLayout_v_configuration(self):
        self.configuration_v_box = QVBoxLayout()
        self.configuration_v_box.addWidget(self.Configuration_g_configuration())
        self.configuration_v_box.addWidget(self.Configuration_g_progress())
        self.configuration_v_box.addWidget(self.Configuration_g_result_calibration_data())

    ## Groupbox

    def Configuration_g_configuration(self):
        groupbox = QGroupBox('Set Configuration')
        vbox = QVBoxLayout()

        liDAR_configuration_label = QLabel('[ LiDAR Configuration ]', self)
        vbox.addWidget(liDAR_configuration_label)

        self.select_principle_sensor_list_layout = element.CheckBoxListLayout(self.ui, 'Select Principle Sensor List')
        vbox.addLayout(self.select_principle_sensor_list_layout)

        liDAR_configuration_label = QLabel('[ Optimization Configuration ]', self)
        vbox.addWidget(liDAR_configuration_label)

        self.pose_sampling_ratio_layout = element.DoubleSpinBoxLabelLayout('Pose Sampling Ratio', self.ui)
        vbox.addLayout(self.pose_sampling_ratio_layout)

        self.point_sampling_ratio_layout = element.DoubleSpinBoxLabelLayout('Point Sampling Ratio', self.ui)
        vbox.addLayout(self.point_sampling_ratio_layout)

        self.num_points_plane_modeling_layout = element.SpinBoxLabelLayout('Num Points Plane Modeling', self.ui)
        vbox.addLayout(self.num_points_plane_modeling_layout)

        self.outlier_distance_layout = element.DoubleSpinBoxLabelLayout('Outlier Distance  [m]', self.ui)
        vbox.addLayout(self.outlier_distance_layout)

        optimization_initial_value_label = QLabel('[ Optimization Initial Value ]', self)
        vbox.addWidget(optimization_initial_value_label)

        self.optimization_initial_value_tab = element.ResultTab(self.ui)
        vbox.addLayout(self.optimization_initial_value_tab)

        groupbox.setLayout(vbox)
        return groupbox

    def Configuration_g_result_calibration_data(self):
        groupbox = QGroupBox('Result Calibration Data')
        vbox = QVBoxLayout(self)

        self.scroll_box = ScrollAreaV()
        vbox.addWidget(self.scroll_box)

        groupbox.setLayout(vbox)
        return groupbox

    def Configuration_g_progress(self):
        groupbox = QGroupBox()
        vbox = QVBoxLayout(self)

        hbox = QHBoxLayout(self)
        btn = QPushButton('Start')
        btn.clicked.connect(lambda: self.StartCalibration(self.ui.importing_tab.limit_time_layout.start_time,
                                                          self.ui.importing_tab.limit_time_layout.end_time,
                                                          self.ui.config.PARM_LIDAR,
                                                          self.text_edit))
        hbox.addWidget(btn)

        self.pause_btn = QPushButton('Pause')
        self.pause_btn.clicked.connect(self.PauseCalibration)
        hbox.addWidget(self.pause_btn)
        vbox.addLayout(hbox)

        label = QLabel('[ Optimization Progress ]')
        vbox.addWidget(label)

        self.text_edit = QTextEdit()
        vbox.addWidget(self.text_edit)

        groupbox.setLayout(vbox)
        return groupbox

    ## Callback func

    def StartCalibration(self, start_time, end_time, sensor_list, progress_callback_func):
        if self.calibration_status is not CONST_STOP:
            return False
        self.calibration_status = CONST_PLAY

        for idxSensor in sensor_list['CheckedSensorList']:
            if self.ui.importing.PointCloudSensorList.get(idxSensor) is None:
                self.PopUp('Import pointcloud {}'.format(idxSensor))
                return False

            if self.ui.optimization.initial_calibration_param.get(idxSensor) is None:
                self.PopUp('Set Lidar {} initial calibration param'.format(idxSensor))
                return False

        self.ui.tabs.setTabEnabled(CONST_EVALUATION, True)
        self.ui.tabs.setTabEnabled(CONST_OPTIMIZATION, True)

        self.ui.thread.SetFunc(self.ui.optimization.Calibration, [start_time, end_time, sensor_list])
        try:
            self.ui.thread.change_value.disconnect()
        except:
            pass
        try:
            self.ui.thread.interation_percentage.disconnect()
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
        progress_callback_func.clear()

        self.ui.thread.emit_string.connect(progress_callback_func.append)
        self.ui.thread.end.connect(self.EndOptimizationCalibration)
        self.ui.thread.start()

    def PauseCalibration(self):
        self.ui.thread.toggle_status()

        if self.calibration_status is CONST_PLAY:
            self.pause_btn.setText("Replay")
            self.calibration_status = CONST_PAUSE

            ## Set 'Result Calibration Data'
            for idxSensor in self.ui.config.PARM_LIDAR['CheckedSensorList']:
                self.result_labels[idxSensor].label_edit_x.setText(str(round(self.ui.optimization.CalibrationParam[idxSensor][3], 2)))
                self.result_labels[idxSensor].label_edit_y.setText(str(round(self.ui.optimization.CalibrationParam[idxSensor][4], 2)))
                self.result_labels[idxSensor].label_edit_yaw.setText(str(round(self.ui.optimization.CalibrationParam[idxSensor][2] * 180 / math.pi, 2)))
        else:
            self.pause_btn.setText("Pause")
            self.calibration_status = CONST_PLAY

    def ViewLiDAR(self):
        if self.calibration_status is not CONST_STOP:
            return False
        self.ui.ViewLiDAR(self.ui.optimization.calib_x, self.ui.optimization.calib_y, self.ui.optimization.calib_yaw)

    def ViewPointCloud(self):
        if self.calibration_status is not CONST_STOP:
            return False
        df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.ui.config,
                                                                                           self.ui.importing,
                                                                                           self.ui.optimization.CalibrationParam,
                                                                                           self.ui.importing_tab.limit_time_layout.start_time,
                                                                                           self.ui.importing_tab.limit_time_layout.end_time)
        self.ui.ViewPointCloud(df_info, accum_pointcloud, PARM_LIDAR)

    def EndOptimizationCalibration(self):
        self.calibration_status = CONST_STOP
        self.ui.tabs.setTabEnabled(CONST_OPTIMIZATION, True)

        df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.ui.config,
                                                                                           self.ui.importing,
                                                                                           self.ui.optimization.CalibrationParam,
                                                                                           self.ui.importing_tab.limit_time_layout.start_time,
                                                                                           self.ui.importing_tab.limit_time_layout.end_time)
        # Optimization tab

        ## Set 'Result Calibration Data'
        for idxSensor in self.ui.config.PARM_LIDAR['CheckedSensorList']:
            self.result_labels[idxSensor].label_edit_x.setText(str(round(self.ui.optimization.CalibrationParam[idxSensor][3], 2)))
            self.result_labels[idxSensor].label_edit_y.setText(str(round(self.ui.optimization.CalibrationParam[idxSensor][4], 2)))
            self.result_labels[idxSensor].label_edit_yaw.setText(str(round(self.ui.optimization.CalibrationParam[idxSensor][2] * 180 / math.pi, 2)))

        ## Plot 'Result Data'
        self.result_data_pose_ax.clear()
        self.ui.ViewLiDAR(self.ui.optimization.calib_x, self.ui.optimization.calib_y, self.ui.optimization.calib_yaw, self.result_data_pose_ax, self.result_data_pose_canvas)

        ## Plot 'Result Graph''
        self.result_graph_ax.clear()
        self.ui.ViewPointCloud(df_info, accum_pointcloud, PARM_LIDAR, self.result_graph_ax, self.result_graph_canvas)

        ## Transfer
        self.CopyList(self.ui.optimization.CalibrationParam, self.ui.evaluation_tab.edit_optimization_calibration_parm)

        # Evaluation tab

        if self.ui.evaluation_tab.button_group.checkedId() == CONST_DISPLAY_OPTIMIZATION:
            ## Plot 'Result of Calibration'
            self.ui.evaluation_tab.result_data_pose_ax.clear()
            self.ui.ViewLiDAR(self.ui.optimization.calib_x, self.ui.optimization.calib_y, self.ui.optimization.calib_yaw,
                              self.ui.evaluation_tab.result_data_pose_ax,
                              self.ui.evaluation_tab.result_data_pose_canvas)

            ## Plot 'Before Aplly Calibration Result'
            self.ui.evaluation_tab.result_before_graph_ax.clear()
            self.ui.ViewPointCloud(df_info, accum_pointcloud, PARM_LIDAR, self.ui.evaluation_tab.result_before_graph_ax,
                                   self.ui.evaluation_tab.result_before_graph_canvas)

            ## Plot 'After Aplly Calibration Result'
            self.ui.evaluation_tab.result_after_graph_ax.clear()
            self.ui.ViewPointCloud(df_info, accum_pointcloud_, PARM_LIDAR, self.ui.evaluation_tab.result_after_graph_ax,
                                   self.ui.evaluation_tab.result_after_graph_canvas)

        ## Set 'Select The Method'
        for idxSensor in self.ui.config.PARM_LIDAR['CheckedSensorList']:
            self.ui.evaluation_tab.opt_result_labels[idxSensor].double_spin_box_x.setValue(self.ui.optimization.CalibrationParam[idxSensor][3])
            self.ui.evaluation_tab.opt_result_labels[idxSensor].double_spin_box_y.setValue(self.ui.optimization.CalibrationParam[idxSensor][4])
            self.ui.evaluation_tab.opt_result_labels[idxSensor].double_spin_box_yaw.setValue(self.ui.optimization.CalibrationParam[idxSensor][2] * 180 / math.pi)

    def PopUp(self, message):
        widget = QWidget()

        qr = widget.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        widget.move(qr.topLeft())

        QMessageBox.information(widget, 'Information', message)

    def CopyList(self, source, target):
        keys = list(source.keys())
        values = list(source.values())

        for i in range(len(keys)):
            target[keys[i]] = values[i].copy()

    def PopUp(self, message):
        widget = QWidget()

        qr = widget.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        widget.move(qr.topLeft())

        QMessageBox.information(widget, 'Information', message)

class EvaluationTab(QWidget):
    def __init__(self, ui):
        super().__init__()
        self.ui = ui
        self.prev_checkID = 0
        self.evaluation_status = CONST_STOP
        self.evaluation_id = None
        self.handeye_eval_df_info = None
        self.handeye_eval_Map = None
        self.handeye_eval_PARM_LIDAR = None
        self.optimization_eval_df_info = None
        self.optimization_eval_Map = None
        self.optimization_eval_PARM_LIDAR = None

        # handeye
        self.handeye_result_labels = {}
        self.edit_handeye_calibration_parm = {}
        self.handeye_eval_lidar = {}

        # optimization
        self.opt_result_labels = {}
        self.edit_optimization_calibration_parm = {}
        self.optimization_eval_lidar = {}

        self.initUi()

    def initUi(self):
        self.InterfaceLayout_v_main()

        self.setLayout(self.main_h_box)

    ## Layout

    def InterfaceLayout_v_main(self):
        self.main_h_box = QHBoxLayout()
        tabs = QTabWidget(self)

        tabs.addTab(self.CalibrationResult(), 'Calibration Result')
        tabs.addTab(self.HandeyeEval(), 'Handeye Evaluation')
        tabs.addTab(self.OptimizationEval(), 'Optimization Evaluation')

        self.main_h_box.addWidget(tabs)

    ## Tab
    def CalibrationResult(self):
        self.cali_result = QWidget(self)
        hbox = QHBoxLayout()

        vbox1 = QVBoxLayout()
        vbox1.addWidget(self.Main_g_select_method())
        vbox1.addWidget(self.Main_g_before_apply_calibration_result())

        hbox.addLayout(vbox1)
        hbox.setStretchFactor(vbox1, 2)

        vbox2 = QVBoxLayout()
        vbox2.addWidget(self.Main_g_result_of_calibration())
        vbox2.addWidget(self.Main_g_after_apply_calibration_result())

        hbox.addLayout(vbox2)
        hbox.setStretchFactor(vbox2, 2)
        self.cali_result.setLayout(hbox)
        return self.cali_result

    def HandeyeEval(self):
        self.handeye_eval = QWidget(self)
        vbox = QVBoxLayout()

        hbox1 = QHBoxLayout()
        hbox1.addWidget(self.HandeyeEvaluation_g_using_sensor_list())
        hbox1.addWidget(self.HandeyeEvaluation_g_limit_time())
        vbox.addLayout(hbox1)

        hbox2 = QHBoxLayout()
        hbox2.addLayout(self.Eval_v_handeye_result_graph())
        hbox2.addLayout(self.Eval_v_handeye_RMSE_graph())
        vbox.addLayout(hbox2)

        self.handeye_eval.setLayout(vbox)
        return self.handeye_eval

    def OptimizationEval(self):
        self.optimization_eval = QWidget(self)
        vbox = QVBoxLayout()

        hbox1 = QHBoxLayout()
        hbox1.addWidget(self.OptimizationEvaluation_g_using_sensor_list())
        hbox1.addWidget(self.OptimizationEvaluation_g_limit_time())
        vbox.addLayout(hbox1)

        vbox.addLayout(hbox1)

        hbox2 = QHBoxLayout()
        hbox2.addLayout(self.Eval_v_optimization_result_graph())
        hbox2.addLayout(self.Eval_v_optimization_RMSE_graph())
        vbox.addLayout(hbox2)

        self.optimization_eval.setLayout(vbox)
        return self.optimization_eval

    ## Groupbox

    def Main_g_select_method(self):
        groupbox = QGroupBox('Select The Method')
        self.select_method_vbox = QVBoxLayout()

        self.button_group = QButtonGroup()
        method1_radio_btn = QRadioButton('Method1. HandEye')
        method1_radio_btn.setChecked(True)
        method1_radio_btn.clicked.connect(self.DisplayCalibrationGraph)
        self.button_group.addButton(method1_radio_btn, 0)
        self.select_method_vbox.addWidget(method1_radio_btn)

        self.handeye_result_scroll_box = ScrollAreaV()
        self.select_method_vbox.addWidget(self.handeye_result_scroll_box)

        method2_radio_btn = QRadioButton('Mtehod2. Optimization')
        method2_radio_btn.setChecked(False)
        method2_radio_btn.clicked.connect(self.DisplayCalibrationGraph)
        self.button_group.addButton(method2_radio_btn, 1)
        self.select_method_vbox.addWidget(method2_radio_btn)

        self.opt_result_scroll_box = ScrollAreaV()
        self.select_method_vbox.addWidget(self.opt_result_scroll_box)

        groupbox.setLayout(self.select_method_vbox)
        return groupbox

    def Main_g_result_of_calibration(self):
        groupbox = QGroupBox('Result Of Calibration')
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

    def Main_g_before_apply_calibration_result(self):
        groupbox = QGroupBox('Before Apply Calibration Result')
        vbox = QVBoxLayout()

        self.result_before_graph_fig = plt.figure()
        self.result_before_graph_canvas = FigureCanvas(self.result_before_graph_fig)
        self.result_before_graph_ax = self.result_before_graph_fig.add_subplot(1, 1, 1)
        self.result_before_graph_ax.grid()
        self.result_before_graph_canvas.draw()
        vbox.addWidget(self.result_before_graph_canvas)

        btn = QPushButton('View')
        btn.clicked.connect(lambda: self.ViewPointCloud(CONST_CALIBRATION_RESULT_TAB))
        vbox.addWidget(btn)

        groupbox.setLayout(vbox)
        return groupbox

    def Main_g_after_apply_calibration_result(self):
        groupbox = QGroupBox('After Apply Calibration Result')
        vbox = QVBoxLayout()

        self.result_after_graph_fig = plt.figure()
        self.result_after_graph_canvas = FigureCanvas(self.result_after_graph_fig)
        self.result_after_graph_ax = self.result_after_graph_fig.add_subplot(1, 1, 1)
        self.result_after_graph_ax.grid()
        self.result_after_graph_canvas.draw()
        vbox.addWidget(self.result_after_graph_canvas)

        btn = QPushButton('View')
        btn.clicked.connect(lambda: self.ViewPointCloud(CONST_CALIBRATION_RESULT_TAB))
        vbox.addWidget(btn)

        groupbox.setLayout(vbox)
        return groupbox

    def HandeyeEvaluation_g_using_sensor_list(self):
        groupbox = QGroupBox('Using Sensor List')
        vbox = QVBoxLayout()

        self.handeye_using_sensor_list_layout = element.CheckBoxListLayout(self.ui)
        vbox.addLayout(self.handeye_using_sensor_list_layout)

        groupbox.setLayout(vbox)
        return groupbox

    def HandeyeEvaluation_g_limit_time(self):
        groupbox = QGroupBox('Limit Time')
        vbox = QVBoxLayout()

        vbox.addLayout(self.Eval_v_handeye_time_limit())

        groupbox.setLayout(vbox)
        return groupbox

    def OptimizationEvaluation_g_using_sensor_list(self):
        groupbox = QGroupBox('Using Sensor List')
        vbox = QVBoxLayout()

        self.optimization_using_sensor_list_layout = element.CheckBoxListLayout(self.ui)
        vbox.addLayout(self.optimization_using_sensor_list_layout)

        groupbox.setLayout(vbox)
        return groupbox

    def OptimizationEvaluation_g_limit_time(self):
        groupbox = QGroupBox('Limit Time')
        vbox = QVBoxLayout()

        vbox.addLayout(self.Eval_v_optimization_time_limit())

        groupbox.setLayout(vbox)
        return groupbox

    ## Layout

    def Eval_v_handeye_result_graph(self):
        vbox = QVBoxLayout()

        self.eval_handeye_graph_fig = plt.figure()
        self.eval_handeye_graph_canvas = FigureCanvas(self.eval_handeye_graph_fig)
        self.eval_handeye_graph_ax = self.eval_handeye_graph_fig.add_subplot(1, 1, 1)
        self.eval_handeye_graph_ax.grid()
        self.eval_handeye_graph_canvas.draw()
        vbox.addWidget(self.eval_handeye_graph_canvas)

        btn = QPushButton('View')
        btn.clicked.connect(lambda: self.ViewPointCloud(CONST_HANDEYE_EVALUATION_TAB))
        vbox.addWidget(btn)

        return vbox

    def Eval_v_handeye_RMSE_graph(self):
        vbox = QVBoxLayout()

        self.eval_handeye_rmse_fig = plt.figure()
        self.eval_handeye_rmse_canvas = FigureCanvas(self.eval_handeye_rmse_fig)

        self.eval_handeye_rmse_xy_ax = self.eval_handeye_rmse_fig.add_subplot(2, 1, 1)
        self.eval_handeye_rmse_xy_ax.set_xlabel('LiDAR List')
        self.eval_handeye_rmse_xy_ax.set_ylabel('RMSE [m]')
        self.eval_handeye_rmse_xy_ax.set_title('RMSE x, y')

        self.eval_handeye_rmse_yaw_ax = self.eval_handeye_rmse_fig.add_subplot(2, 1, 2)
        self.eval_handeye_rmse_yaw_ax.set_xlabel('LiDAR List')
        self.eval_handeye_rmse_yaw_ax.set_ylabel('RMSE [deg]')
        self.eval_handeye_rmse_yaw_ax.set_title('RMSE yaw')

        self.eval_handeye_rmse_fig.tight_layout()
        self.eval_handeye_rmse_canvas.draw()
        vbox.addWidget(self.eval_handeye_rmse_canvas)

        return vbox

    def Eval_v_handeye_time_limit(self):
        vbox = QVBoxLayout()

        self.handeye_limit_time_layout = element.SlideLabelLayouts(self.ui)
        vbox.addLayout(self.handeye_limit_time_layout)

        hbox = QHBoxLayout()
        btn = QPushButton('Start')
        btn.clicked.connect(lambda: self.StartEvaluation(CONST_HANDEYE,
                                                         self.handeye_limit_time_layout.start_time,
                                                         self.handeye_limit_time_layout.end_time,
                                                         self.handeye_eval_lidar,
                                                         self.ui.handeye.CalibrationParam,
                                                         self.handeye_pbar.setValue,
                                                         self.EndEvaluation))
        hbox.addWidget(btn)

        self.pause_btn = QPushButton('Pause')
        self.pause_btn.clicked.connect(self.PauseCalibration)
        hbox.addWidget(self.pause_btn)
        vbox.addLayout(hbox)

        label = QLabel('[ HandEye Evaluation Progress ]')
        vbox.addWidget(label)

        self.handeye_pbar = QProgressBar(self)
        vbox.addWidget(self.handeye_pbar)

        return vbox

    def Eval_v_optimization_result_graph(self):
        vbox = QVBoxLayout()

        self.eval_optimization_graph_fig = plt.figure()
        self.eval_optimization_graph_canvas = FigureCanvas(self.eval_optimization_graph_fig)
        self.eval_optimization_graph_ax = self.eval_optimization_graph_fig.add_subplot(1, 1, 1)
        self.eval_optimization_graph_ax.grid()
        self.eval_optimization_graph_canvas.draw()
        vbox.addWidget(self.eval_optimization_graph_canvas)

        btn = QPushButton('View')
        btn.clicked.connect(lambda: self.ViewPointCloud(CONST_OPTIMIZATION_EVALUATION_TAB))
        vbox.addWidget(btn)

        return vbox

    def Eval_v_optimization_RMSE_graph(self):
        vbox = QVBoxLayout()

        self.eval_optimization_rmse_fig = plt.figure()
        self.eval_optimization_rmse_canvas = FigureCanvas(self.eval_optimization_rmse_fig)

        self.eval_optimization_rmse_xy_ax = self.eval_optimization_rmse_fig.add_subplot(2, 1, 1)
        self.eval_optimization_rmse_xy_ax.set_xlabel('LiDAR List')
        self.eval_optimization_rmse_xy_ax.set_ylabel('RMSE [m]')
        self.eval_optimization_rmse_xy_ax.set_title('RMSE x, y')

        self.eval_optimization_rmse_yaw_ax = self.eval_optimization_rmse_fig.add_subplot(2, 1, 2)
        self.eval_optimization_rmse_yaw_ax.set_xlabel('LiDAR List')
        self.eval_optimization_rmse_yaw_ax.set_ylabel('RMSE [deg]')
        self.eval_optimization_rmse_yaw_ax.set_title('RMSE yaw')

        self.eval_optimization_rmse_fig.tight_layout()
        self.eval_optimization_rmse_canvas.draw()
        vbox.addWidget(self.eval_optimization_rmse_canvas)

        return vbox

    def Eval_v_optimization_time_limit(self):
        vbox = QVBoxLayout()

        self.optimization_limit_time_layout = element.SlideLabelLayouts(self.ui)
        vbox.addLayout(self.optimization_limit_time_layout)

        hbox = QHBoxLayout()
        btn = QPushButton('Start')
        btn.clicked.connect(lambda: self.StartEvaluation(CONST_OPTIMIZATION,
                                                         self.optimization_limit_time_layout.start_time,
                                                         self.optimization_limit_time_layout.end_time,
                                                         self.optimization_eval_lidar,
                                                         self.ui.optimization.CalibrationParam,
                                                         self.optimization_pbar.setValue,
                                                         self.EndEvaluation))
        hbox.addWidget(btn)

        self.pause_btn = QPushButton('Pause')
        self.pause_btn.clicked.connect(self.PauseCalibration)
        hbox.addWidget(self.pause_btn)
        vbox.addLayout(hbox)

        label = QLabel('[ Optimization Evaluation Progress ]')
        vbox.addWidget(label)

        self.optimization_pbar = QProgressBar(self)
        vbox.addWidget(self.optimization_pbar)

        return vbox

    # Callback
    def ViewLiDAR(self):
        status = self.button_group.checkedId()
        if status == CONST_DISPLAY_HANDEYE:
            calib_x, calib_y, calib_yaw = self.ui.handeye.calib_x, self.ui.handeye.calib_y, self.ui.handeye.calib_yaw

        elif status == CONST_DISPLAY_OPTIMIZATION:
            calib_x, calib_y, calib_yaw = self.ui.optimization.calib_x, self.ui.optimization.calib_y, self.ui.optimization.calib_yaw

        self.ui.ViewLiDAR(calib_x, calib_y, calib_yaw)

    def ViewPointCloud(self, tab_id):
        if tab_id == CONST_CALIBRATION_RESULT_TAB:
            status = self.button_group.checkedId()
            if status == CONST_DISPLAY_HANDEYE:
                df_info, PARM_LIDAR, pointcloud, origin_pointcloud = get_result.GetPlotParam(self.ui.config, self.ui.importing, self.ui.handeye.CalibrationParam,
                                                                                               self.ui.importing_tab.limit_time_layout.start_time,
                                                                                               self.ui.importing_tab.limit_time_layout.end_time)

            elif status == CONST_DISPLAY_OPTIMIZATION:
                df_info, PARM_LIDAR, pointcloud, origin_pointcloud = get_result.GetPlotParam(self.ui.config, self.ui.importing, self.ui.optimization.CalibrationParam,
                                                                                               self.ui.importing_tab.limit_time_layout.start_time,
                                                                                               self.ui.importing_tab.limit_time_layout.end_time)

        elif tab_id == CONST_HANDEYE_EVALUATION_TAB:
            if self.handeye_eval_df_info is None:
                return False
            df_info = self.handeye_eval_df_info
            PARM_LIDAR = self.handeye_eval_PARM_LIDAR
            pointcloud = self.handeye_eval_Map

        elif tab_id == CONST_OPTIMIZATION_EVALUATION_TAB:
            if self.optimization_eval_df_info is None:
                return False
            df_info = self.optimization_eval_df_info
            PARM_LIDAR = self.optimization_eval_PARM_LIDAR
            pointcloud = self.optimization_eval_Map

        self.ui.ViewPointCloud(df_info, pointcloud, PARM_LIDAR)

    def DisplayCalibrationGraph(self):
        status = self.button_group.checkedId()
        if status == CONST_DISPLAY_HANDEYE:
            if not self.ui.handeye.complete_calibration:
                self.button_group.button(self.prev_checkID).setChecked(True)
                return False
        elif status == CONST_DISPLAY_OPTIMIZATION:
            if not self.ui.optimization.complete_calibration:
                self.button_group.button(self.prev_checkID).setChecked(True)
                return False

        if status == CONST_DISPLAY_HANDEYE:
            df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.ui.config, self.ui.importing,self.edit_handeye_calibration_parm,
                                                                                           self.ui.importing_tab.limit_time_layout.start_time,
                                                                                           self.ui.importing_tab.limit_time_layout.end_time)
            calib_x, calib_y, calib_yaw = self.ui.handeye.calib_x, self.ui.handeye.calib_y, self.ui.handeye.calib_yaw
        elif status == CONST_DISPLAY_OPTIMIZATION:
            df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.ui.config, self.ui.importing, self.edit_optimization_calibration_parm,
                                                                                           self.ui.importing_tab.limit_time_layout.start_time,
                                                                                           self.ui.importing_tab.limit_time_layout.end_time)
            calib_x, calib_y, calib_yaw = self.ui.optimization.calib_x, self.ui.optimization.calib_y, self.ui.optimization.calib_yaw

        ## Plot 'Result of Calibration'
        self.result_data_pose_ax.clear()
        self.ui.ViewLiDAR(calib_x, calib_y, calib_yaw, self.result_data_pose_ax, self.result_data_pose_canvas)

        ## Plot 'After Apply Calibration Result'
        self.result_after_graph_ax.clear()
        self.ui.ViewPointCloud(df_info, accum_pointcloud, PARM_LIDAR, self.result_after_graph_ax, self.result_after_graph_canvas)

        self.prev_checkID = self.button_group.checkedId()

    def PauseCalibration(self):
        pass
        # self.ui.thread.toggle_status()
        #
        # if self.calibration_status is CONST_PLAY:
        #     self.pause_btn.setText("Replay")
        #     self.calibration_status = CONST_PAUSE
        #
        #     ## Set 'Result Calibration Data'
        #     for idxSensor in self.ui.config.PARM_LIDAR['CheckedSensorList']:
        #         self.result_labels[idxSensor].label_edit_x.setText(
        #             str(round(self.ui.optimization.CalibrationParam[idxSensor][3], 2)))
        #         self.result_labels[idxSensor].label_edit_y.setText(
        #             str(round(self.ui.optimization.CalibrationParam[idxSensor][4], 2)))
        #         self.result_labels[idxSensor].label_edit_yaw.setText(
        #             str(round(self.ui.optimization.CalibrationParam[idxSensor][2] * 180 / math.pi, 2)))
        # else:
        #     self.pause_btn.setText("Pause")
        #     self.calibration_status = CONST_PLAY

    def StartEvaluation(self, evaluation_id, start_time, end_time, sensor_list, calibration_param, progress_callback_func, end_callback_func):
        if self.evaluation_status is not CONST_STOP:
            return False
        self.evaluation_status = CONST_PLAY
        self.evaluation_id = evaluation_id

        for idxSensor in sensor_list['CheckedSensorList']:
            if self.ui.importing.PointCloudSensorList.get(idxSensor) is None:
                self.PopUp('Import pointcloud {}'.format(idxSensor))
                return False

        self.ui.thread.SetFunc(self.ui.evaluation.Evaluation, [start_time, end_time, sensor_list, calibration_param])
        try:
            self.ui.thread.change_value.disconnect()
        except:
            pass
        try:
            self.ui.thread.interation_percentage.disconnect()
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

        self.ui.thread.change_value.connect(progress_callback_func)
        self.ui.thread.end.connect(end_callback_func)
        self.ui.thread.start()

    def EndEvaluation(self):
        self.evaluation_status = CONST_STOP

        if self.evaluation_id == CONST_HANDEYE:
            ## Plot 'Result Graph'
            self.eval_handeye_graph_ax.clear()
            self.handeye_eval_df_info = copy.deepcopy(self.ui.evaluation.df_info)
            self.handeye_eval_Map = copy.deepcopy(self.ui.evaluation.Map)
            self.handeye_eval_PARM_LIDAR = copy.deepcopy(self.ui.evaluation.PARM_LIDAR)
            self.ui.ViewPointCloud(self.handeye_eval_df_info,
                                   self.handeye_eval_Map,
                                   self.handeye_eval_PARM_LIDAR,
                                   self.eval_handeye_graph_ax,
                                   self.eval_handeye_graph_canvas)

            ## Plot 'Result RMSE'
            self.eval_handeye_rmse_xy_ax.clear()
            self.eval_handeye_rmse_yaw_ax.clear()
            self.ui.ViewRMSE(self.ui.evaluation.rmse_x,
                             self.ui.evaluation.rmse_y,
                             self.ui.evaluation.rmse_yaw,
                             self.ui.evaluation.lidarlist,
                             self.ui.evaluation.PARM_LIDAR,
                             self.eval_handeye_rmse_xy_ax,
                             self.eval_handeye_rmse_yaw_ax,
                             self.eval_handeye_rmse_canvas)

        elif self.evaluation_id == CONST_OPTIMIZATION:
            ## Plot 'Result Graph'
            self.eval_optimization_graph_ax.clear()
            self.optimization_eval_df_info = copy.deepcopy(self.ui.evaluation.df_info)
            self.optimization_eval_Map = copy.deepcopy(self.ui.evaluation.Map)
            self.optimization_eval_PARM_LIDAR = copy.deepcopy(self.ui.evaluation.PARM_LIDAR)
            self.ui.ViewPointCloud(self.optimization_eval_df_info,
                                   self.optimization_eval_Map,
                                   self.optimization_eval_PARM_LIDAR,
                                   self.eval_optimization_graph_ax,
                                   self.eval_optimization_graph_canvas)

            ## Plot 'Result RMSE'
            self.eval_optimization_rmse_xy_ax.clear()
            self.eval_optimization_rmse_yaw_ax.clear()
            self.ui.ViewRMSE(self.ui.evaluation.rmse_x,
                             self.ui.evaluation.rmse_y,
                             self.ui.evaluation.rmse_yaw,
                             self.ui.evaluation.lidarlist,
                             self.ui.evaluation.PARM_LIDAR,
                             self.eval_optimization_rmse_xy_ax,
                             self.eval_optimization_rmse_yaw_ax,
                             self.eval_optimization_rmse_canvas)

        print('end evaluation')

    def OptimizationEvaluation(self):

        pass

class MyApp(QMainWindow):
    def __init__(self, parent=None):
        super(MyApp, self).__init__(parent)
        self.form_widget = FormWidget(self)

        self.InitUi()

    def InitUi(self):
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

        self.setWindowTitle('Calibration Tool')
        # self.showFullScreen()
        # self.resize(1000, 600)
        self.center()

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
        fname = QFileDialog.getOpenFileName(widget, 'Open file', self.form_widget.config.PATH['Configuration'], "Configuration file (*.ini)")

        if fname[0]:
            self.form_widget.config.configuration_file = fname[0]
            self.form_widget.config.InitConfiguration()
            self.form_widget.SetConfiguration()
            print('Open '+ str(fname[0]))

    def OpenNewFile(self): # Ctrl+N
        if self.form_widget.value_changed:
            reply = QMessageBox.question(self, 'Open New File', 'Do you want to save?',
                                         QMessageBox.Save | QMessageBox.No | QMessageBox.Cancel, QMessageBox.Save)
            if reply == QMessageBox.Save:
                fname = self.SaveDialog()
                if fname[0]:
                    self.form_widget.config.configuration_file = fname[0]
                    self.form_widget.config.WriteFile(fname[0])
                    self.SaveIniFile()
                    print('Save Current ini File')
                else:
                    print('Cancel Save ini File')
                    return False
            elif reply == QMessageBox.Cancel:
                print('Cancel Save ini File')
                return False

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
        is_handeye = False
        is_multi_optimization = False

        for line in fileinput.input(self.form_widget.config.configuration_file, inplace=True):
            if 'PrincipalSensor' in line:
                line = line.replace(line, 'PrincipalSensor = ' + str(self.form_widget.config.PARM_LIDAR['PrincipalSensor']) + '\n')
            elif 'CheckedSensorList' in line:
                line = line.replace(line, 'CheckedSensorList = ' + str(checked_sensor_list) + '\n')
            elif 'SensorList' in line:
                line = line.replace(line, 'SensorList = ' + str(sensor_list) + '\n')
            elif 'MinThresholdDist_m' in line:
                is_pointcloud = True
                line = line.replace(line, 'MinThresholdDist_m = ' + str(self.form_widget.config.PARM_PC['MinThresholdDist_m']) + '\n')
            elif 'MaxThresholdDist_m' in line:
                line = line.replace(line, 'MaxThresholdDist_m = ' + str(self.form_widget.config.PARM_PC['MaxThresholdDist_m']) + '\n')
            elif 'MinThresholdX_m' in line:
                line = line.replace(line, 'MinThresholdX_m = ' + str(self.form_widget.config.PARM_PC['MinThresholdX_m']) + '\n')
            elif 'MaxThresholdX_m' in line:
                line = line.replace(line, 'MaxThresholdX_m = ' + str(self.form_widget.config.PARM_PC['MaxThresholdX_m']) + '\n')
            elif 'MinThresholdY_m' in line:
                line = line.replace(line, 'MinThresholdY_m = ' + str(self.form_widget.config.PARM_PC['MinThresholdY_m']) + '\n')
            elif 'MaxThresholdY_m' in line:
                line = line.replace(line, 'MaxThresholdY_m = ' + str(self.form_widget.config.PARM_PC['MaxThresholdY_m']) + '\n')
            elif 'MinThresholdZ_m' in line:
                line = line.replace(line, 'MinThresholdZ_m = ' + str(self.form_widget.config.PARM_PC['MinThresholdZ_m']) + '\n')
            elif 'MaxThresholdZ_m' in line:
                line = line.replace(line, 'MaxThresholdZ_m = ' + str(self.form_widget.config.PARM_PC['MaxThresholdZ_m']) + '\n')
            elif 'SamplingInterval' in line:
                is_handeye = True
                line = line.replace(line, 'SamplingInterval = ' + str(self.form_widget.config.PARM_HE['SamplingInterval']) + '\n')
            elif 'MaximumIteration' in line:
                line = line.replace(line, 'MaximumIteration = ' + str(self.form_widget.config.PARM_HE['MaximumIteration']) + '\n')
            elif 'Tolerance' in line:
                line = line.replace(line, 'Tolerance = ' + str(self.form_widget.config.PARM_HE['Tolerance']) + '\n')
            elif ('OutlierDistance_m' in line) and (is_multi_optimization == False):
                line = line.replace(line, 'OutlierDistance_m = ' + str(self.form_widget.config.PARM_HE['OutlierDistance_m']) + '\n')
            elif 'filter_HeadingThreshold' in line:
                line = line.replace(line, 'filter_HeadingThreshold = ' + str(self.form_widget.config.PARM_HE['filter_HeadingThreshold']) + '\n')
            elif 'filter_DistanceThreshold' in line:
                line = line.replace(line, 'filter_DistanceThreshold = ' + str(self.form_widget.config.PARM_HE['filter_DistanceThreshold']) + '\n')
            elif 'PoseSamplingRatio' in line:
                is_multi_optimization = True
                line = line.replace(line, 'PoseSamplingRatio = ' + str(self.form_widget.config.PARM_MO['PoseSamplingRatio']) + '\n')
            elif 'PointSamplingRatio' in line:
                line = line.replace(line, 'PointSamplingRatio = ' + str(self.form_widget.config.PARM_MO['PointSamplingRatio']) + '\n')
            elif 'NumPointsPlaneModeling' in line:
                line = line.replace(line, 'NumPointsPlaneModeling = ' + str(self.form_widget.config.PARM_MO['NumPointsPlaneModeling']) + '\n')
            elif ('OutlierDistance_m' in line) and (is_multi_optimization == True):
                line = line.replace(line, 'OutlierDistance_m = ' + str(self.form_widget.config.PARM_MO['OutlierDistance_m']) + '\n')
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
        reply = QMessageBox.question(self, 'Window Close', 'Are you sure you want to close the window?',
                                     QMessageBox.Save | QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
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
        elif reply == QMessageBox.No:
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
        words = self.form_widget.config.configuration_file.split('/')
        file = words[-1]
        directory = self.form_widget.config.PATH['Configuration']
        default_file = directory + '/' + file
        widget = QWidget()
        fname = QFileDialog().getSaveFileName(widget, caption='Save File', directory=default_file, filter="Configuration file (*.ini)")
        return fname

class FormWidget(QWidget):
    def __init__(self, parent):
        super(FormWidget, self).__init__(parent)
        self.thread = QThread.Thread()

        self.config = v1_step1_configuration.Configuration()
        self.importing = v1_step2_import_data.Import(self.config)
        self.handeye = v1_step3_handeye.HandEye(self.config, self.importing)
        self.optimization = v1_step4_optimization.Optimization(self.config, self.importing)
        self.evaluation = v1_step5_evaluation.Evaluation(self.config, self.importing)
        self.config.WriteDefaultFile()
        self.config.InitConfiguration()

        self.InitUi()

        self.SetConfiguration()
        self.value_changed = False

        self.color_list = ['r', 'b', 'c', 'm', 'g', 'y']

    def InitUi(self):
        self.hbox = QHBoxLayout(self)
        self.tabs = QTabWidget(self)

        self.config_tab = ConfigurationTab(self)
        self.importing_tab = ImportDataTab(self)
        self.handeye_tab = HandEyeTab(self)
        self.optimization_tab = OptimizationTab(self)
        self.evaluation_tab = EvaluationTab(self)

        self.tabs.addTab(self.config_tab, 'Step1. Configuration')
        self.tabs.setTabEnabled(CONST_CONFIG, True)

        self.tabs.addTab(self.importing_tab, 'Step2. Import Data')
        self.tabs.setTabEnabled(CONST_IMPORTDATA, True)

        self.tabs.addTab(self.handeye_tab, 'Step3. HandEye')
        self.tabs.setTabEnabled(CONST_HANDEYE, True)

        self.tabs.addTab(self.optimization_tab, 'Step4. Optimization')
        self.tabs.setTabEnabled(CONST_OPTIMIZATION, True)

        self.tabs.addTab(self.evaluation_tab, 'Evaluation')
        self.tabs.setTabEnabled(CONST_EVALUATION, True)

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
        self.importing_tab.logging_file_path_layout.label_edit.setText(self.config.PATH['Logging_file_path'])
        self.importing_tab.logging_file_path_layout.path_file_str = self.config.PATH['Logging_file_path']

        ### Setting handeye tab
        PARM_HE = self.config.PARM_HE
        self.handeye_tab.sampling_interval_layout.spin_box.setValue(PARM_HE['SamplingInterval'])
        self.handeye_tab.maximum_interation_layout.spin_box.setValue(PARM_HE['MaximumIteration'])
        self.handeye_tab.tolerance_layout.double_spin_box.setValue(PARM_HE['Tolerance'])
        self.handeye_tab.outlier_distance_layout.double_spin_box.setValue(PARM_HE['OutlierDistance_m'])
        self.handeye_tab.heading_threshold_layout.double_spin_box.setValue(PARM_HE['filter_HeadingThreshold'])
        self.handeye_tab.distance_threshold_layout.double_spin_box.setValue(PARM_HE['filter_DistanceThreshold'])

        ### Setting optimization tab
        PARM_MO = self.config.PARM_MO
        self.optimization_tab.pose_sampling_ratio_layout.double_spin_box.setValue(PARM_MO['PoseSamplingRatio'])
        self.optimization_tab.point_sampling_ratio_layout.double_spin_box.setValue(PARM_MO['PointSamplingRatio'])
        self.optimization_tab.num_points_plane_modeling_layout.spin_box.setValue(PARM_MO['NumPointsPlaneModeling'])
        self.optimization_tab.outlier_distance_layout.double_spin_box.setValue(PARM_MO['OutlierDistance_m'])
        self.optimization_tab.select_principle_sensor_list_layout.AddWidgetItem(self.config.PARM_LIDAR['SensorList'], self.config.PARM_LIDAR['CheckedSensorList'])

        ### Setting evaluation tab
        self.evaluation_tab.handeye_eval_lidar['CheckedSensorList'] = copy.deepcopy(self.config.PARM_LIDAR['CheckedSensorList'])
        self.evaluation_tab.optimization_eval_lidar['CheckedSensorList'] = copy.deepcopy(self.config.PARM_LIDAR['CheckedSensorList'])

        print('Set all tab\'s configuration')

    def ResetResultsLabels(self):
        self.ResetResultsLabel(CONST_UNEDITABLE_LABEL, self.handeye_tab.scroll_box.layout, self.handeye_tab.result_labels,
                               self.handeye.CalibrationParam)
        self.ResetResultsLabel(CONST_UNEDITABLE_LABEL, self.optimization_tab.scroll_box.layout, self.optimization_tab.result_labels,
                               self.optimization.CalibrationParam)

        self.ResetResultsLabel(CONST_EDITABLE_LABEL, self.evaluation_tab.handeye_result_scroll_box.layout,
                               self.evaluation_tab.handeye_result_labels,
                               self.evaluation_tab.edit_handeye_calibration_parm,
                               CONST_DISPLAY_HANDEYE)
        self.ResetResultsLabel(CONST_EDITABLE_LABEL, self.evaluation_tab.opt_result_scroll_box.layout,
                               self.evaluation_tab.opt_result_labels,
                               self.evaluation_tab.edit_optimization_calibration_parm,
                               CONST_DISPLAY_OPTIMIZATION)

        self.ResetResultsLabel(CONST_EDITABLE_LABEL2, self.optimization_tab.optimization_initial_value_tab.user_define_scroll_box.layout,
                               self.optimization_tab.user_define_initial_labels,
                               self.config.CalibrationParam)
        self.ResetResultsLabel(CONST_EDITABLE_LABEL2, self.optimization_tab.optimization_initial_value_tab.handeye_scroll_box.layout,
                               self.optimization_tab.handeye_result_labels,
                               self.optimization_tab.edit_handeye_calibration_parm)

    def ResetResultsLabel(self, label_type, layout, labels, calibration_param, display_type=0):
        self.RemoveLayout(layout)
        labels.clear()
        for idxSensor in self.config.PARM_LIDAR['CheckedSensorList']:
            if label_type is CONST_UNEDITABLE_LABEL:
                result_label = element.CalibrationResultLabel(idxSensor)
                if calibration_param.get(idxSensor) is not None:
                    result_label.label_edit_x.setText(str(round(calibration_param[idxSensor][3], 2)))
                    result_label.label_edit_y.setText(str(round(calibration_param[idxSensor][4], 2)))
                    result_label.label_edit_yaw.setText(str(round(calibration_param[idxSensor][2] * 180 / math.pi, 2)))
            elif label_type is CONST_EDITABLE_LABEL:
                result_label = element.CalibrationResultEditLabel(display_type, idxSensor, calibration_param, self)
                if calibration_param.get(idxSensor) is not None:
                    result_label.double_spin_box_x.setValue(calibration_param[idxSensor][3])
                    result_label.double_spin_box_y.setValue(calibration_param[idxSensor][4])
                    result_label.double_spin_box_yaw.setValue(calibration_param[idxSensor][2] * 180 / math.pi)
            elif label_type is CONST_EDITABLE_LABEL2:
                result_label = element.CalibrationResultEditLabel2(idxSensor, calibration_param, self)
                if calibration_param.get(idxSensor) is not None:
                    result_label.double_spin_box_roll.setValue(calibration_param[idxSensor][0] * 180 / math.pi)
                    result_label.double_spin_box_pitch.setValue(calibration_param[idxSensor][1] * 180 / math.pi)
                    result_label.double_spin_box_yaw.setValue(calibration_param[idxSensor][2] * 180 / math.pi)
                    result_label.double_spin_box_x.setValue(calibration_param[idxSensor][3])
                    result_label.double_spin_box_y.setValue(calibration_param[idxSensor][4])
                    result_label.double_spin_box_z.setValue(calibration_param[idxSensor][5])
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

        layout = target.itemAt(0)
        target.removeItem(layout)

    # def _onPick(self,event):
    #     thisline = event.artist
    #     thisline.set_linewidth(5)
    #     fig.canvas.draw()

    def ViewLiDAR(self, calib_x, calib_y, calib_yaw, ax=None, canvas=None):
        if len(calib_x) is not len(self.config.PARM_LIDAR['CheckedSensorList']):
            print('The length of calibration x does not match the checked sensor list')
            return 0
        if len(calib_y) is not len(self.config.PARM_LIDAR['CheckedSensorList']):
            print('The length of calibration y does not match the checked sensor list')
            return 0
        if len(calib_yaw) is not len(self.config.PARM_LIDAR['CheckedSensorList']):
            print('The length of calibration yaw does not match the checked sensor list')
            return 0
        
        lidar_num = len(self.handeye.config.PARM_LIDAR['CheckedSensorList'])
        column = '2'
        row = str(math.ceil(lidar_num / 2))
        fig = plt.figure(figsize=(20, 20))

        veh_path = self.config.PATH['Image_path'] + 'vehicle1.png'
        veh = plt.imread(veh_path)

        for i in range(len(self.config.PARM_LIDAR['CheckedSensorList'])):
            idxSensor = list(self.config.PARM_LIDAR['CheckedSensorList'])
            if canvas is None:
                plot_num_str = column + row + str(i + 1)
                ax = fig.add_subplot(int(plot_num_str))

            x = int(calib_x[i]) * 200 + 520
            y = 1000 - 1 * int(calib_y[i]) * 200 - 500
            # car_length = 1.75
            lidar_num = 'lidar' + str(idxSensor[i])
            ax.scatter(x, y, s=300, label=lidar_num, color=self.color_list[(idxSensor[i])%len(self.color_list)], edgecolor='none', alpha=0.5)
            ax.arrow(x, y, 100 * np.cos(calib_yaw[i] * np.pi / 180),
                     -100 * np.sin(calib_yaw[i] * np.pi / 180), head_width=10,
                     head_length=10,
                     fc='k', ec='k')
            ax.plot(np.linspace(520, x, 100), np.linspace(500, y, 100), self.color_list[(idxSensor[i])%len(self.color_list)] + '--')

            if canvas is None:
                ax.imshow(veh)
                ax.set_xlim([-500, 1500])
                ax.set_ylim([1000, 0])
                ax.grid()
                ax.legend()
                ax.set_title('Result of calibration - LiDAR' + str(idxSensor[i]))
                ax.axes.xaxis.set_visible(False)
                ax.axes.yaxis.set_visible(False)

        if canvas is not None:
            ax.imshow(veh)
            ax.set_xlim([-500, 1500])
            ax.set_ylim([1000, 0])
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
        fig = plt.figure(figsize=(20, 20))

        for i in range(len(PARM_LIDAR['CheckedSensorList'])):
            idxSensor = list(PARM_LIDAR['CheckedSensorList'])
            if canvas is None:
                plot_num_str = column + row + str(i + 1)
                ax = fig.add_subplot(int(plot_num_str))

            strColIndex = 'PointCloud_' + str(idxSensor[i])

            ax.plot(df_info['east_m'].values, df_info['north_m'].values, '.', label='trajectory', color='gray')
            ax.plot(pointcloud[idxSensor[i]][:, 0], pointcloud[idxSensor[i]][:, 1], ',',
                    color=self.color_list[(idxSensor[i]) % len(self.color_list)], label=strColIndex)

            if canvas is None:
                ax.axis('equal')
                ax.grid()
                ax.legend()
                ax.set_title('Result of calibration - LiDAR' + str(idxSensor[i]))

        if canvas is not None:
            ax.axis('equal')
            ax.grid()
            ax.legend()
            ax.set_title('Result of calibration')
            canvas.draw()
        else:
            root = Tk.Tk()
            canvas = FigureCanvasTkAgg(fig, master=root)
            nav = NavigationToolbar2Tk(canvas, root)
            canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            canvas._tkcanvas.pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)
            root.mainloop()


    def ViewRMSE(self, rmse_x, rmse_y, rmse_yaw, lidarlist, PARM_LIDAR, xy_ax, yaw_ax, canvas=None):
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
        xy_ax.set_title('RMSE x, y')
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
        yaw_ax.set_title('RMSE yaw')
        yaw_ax.legend()
        canvas.draw()

    def resizeEvent(self, e):
        width = self.geometry().width()
        self.config_tab.next_btn.setFixedSize(width-42, CONST_NEXT_BTN_HEIGHT)
        self.importing_tab.next_btn.setFixedSize(width-42, CONST_NEXT_BTN_HEIGHT)
        self.importing_tab.lidar_scroll_box.setFixedSize(width-62, CONST_SCROLL_BOX_HEIGHT)
        self.importing_tab.gnss_scroll_box.setFixedSize(width-62, CONST_SCROLL_BOX_HEIGHT)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MyApp()
    w.show()
    sys.exit(app.exec_())
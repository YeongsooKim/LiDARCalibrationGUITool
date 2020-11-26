# -*- coding: utf-8 -*-
"""
@author: kimkys768@gmail.com, yondoo20@gmail.com
@date: 2020-09-22
@version: 0.0.2
"""

import os
import math
import copy

from widget.DoubleSlider import DoubleSlider
from widget.QScrollarea import *
from widget.QButton import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap

CONST_DISPLAY_HANDEYE = 0
CONST_DISPLAY_OPTIMIZATION = 1

CONST_GREEN = 0
CONST_RED = 1
CONST_LIDAR = 0
CONST_GNSS = 1

CONST_CUSTOM = 1
CONST_HANDEYE = 2
CONST_OPTIMIZATION = 3

CONST_CONFIG_MINIMUM_THRESHOLD_DISTANCE = 1
CONST_CONFIG_MAXIMUM_THRESHOLD_DISTANCE = 2
CONST_CONFIG_MINIMUM_THRESHOLD_X = 3
CONST_CONFIG_MAXIMUM_THRESHOLD_X = 4
CONST_CONFIG_MINIMUM_THRESHOLD_Y = 5
CONST_CONFIG_MAXIMUM_THRESHOLD_Y = 6
CONST_CONFIG_MINIMUM_THRESHOLD_Z = 7
CONST_CONFIG_MAXIMUM_THRESHOLD_Z = 8

CONST_IMPORT_VEHICLE_MINIMUM_SPEED = 9

CONST_HANDEYE_TOLERANCE = 10
CONST_HANDEYE_OUTLIER_DISTANCE = 11
CONST_HANDEYE_HEADING_THRESHOLD = 12
CONST_HANDEYE_DISTANCE_THRESHOLD = 13

CONST_OPTI_POINT_SAMPLING_RATIO = 14
CONST_OPTI_OUTLIER_DISTANCE = 15

CONST_EVAL_VEHICLE_MINIMUM_SPEED = 16

class FileInputWithCheckBtnLayout(QVBoxLayout):
    instance_number = 1
    def __init__(self, label_str, ui):
        super().__init__()
        self.id = FileInputWithCheckBtnLayout.instance_number
        self.label_str = label_str
        self.path_file_str = ''
        self.ui = ui
        self.parsed_bin = ''
        self.END_LINE_INDEX = 15

        FileInputWithCheckBtnLayout.instance_number += 1
        self.InitUi()

    def InitUi(self):
        self.window = QMainWindow()
        hbox = QHBoxLayout()

        self.label_edit = QLineEdit()
        hbox.addWidget(self.label_edit)

        self.btn = QPushButton('...')
        self.btn.clicked.connect(self.GetFileBtn)
        hbox.addWidget(self.btn)

        self.import_btn = QPushButton('Import')
        self.import_btn.clicked.connect(self.ImportFile)
        hbox.addWidget(self.import_btn)
        self.addLayout(hbox)

        self.pbar = QProgressBar()
        self.addWidget(self.pbar)

    def GetFileBtn(self):
        widget = QWidget()
        export_file_path = str(QFileDialog.getExistingDirectory(widget, 'Select Directory'))

        if export_file_path:
            self.path_file_str = export_file_path
            self.label_edit.setText(self.path_file_str)

    def ImportFile(self):
        self.ui.importing.has_gnss_file = False
        self.ui.importing.has_motion_file = False
        self.CheckGnssFile()
        has_pointcloud_file = self.CheckPointCloudFile()

        if not self.ui.importing.has_gnss_file:
            self.ui.ErrorPopUp('Gnss.csv is missing')
        if not self.ui.importing.has_motion_file:
            self.ui.ErrorPopUp('Motion.csv is missing')
        if not has_pointcloud_file:
            self.ui.ErrorPopUp('XYZRGB.bin is missing')

        self.label_edit.setText(self.path_file_str)
        self.GenerateCSVBtn()
        if self.ui.importing.has_gnss_file:
            self.gnss_button.setText('Gnss.csv 100%')
        else:
            self.gnss_button.setText('Gnss.csv 0%')

        if self.ui.importing.has_motion_file:
            self.motion_button.setText('Motion.csv 100%')
        else:
            self.motion_button.setText('Motion.csv 0%')

        self.GeneratePointCloudBtn()

        if (self.ui.importing.has_gnss_file or self.ui.importing.has_motion_file) and has_pointcloud_file:
            if self.ui.importing.has_gnss_file:
                self.ui.importing.ParseGnss()
            if self.ui.importing.has_motion_file:
                self.ui.importing.ParseMotion()

            self.ui.thread.SetFunc(self.ui.importing.ParsePointCloud)
            self.ui.thread._status = True
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

            self.ui.thread.change_value.connect(self.pbar.setValue)
            self.ui.thread.iteration_percentage.connect(self.ui.importing_tab.IterationPercentage)
            self.ui.thread.end.connect(self.ui.importing_tab.EndImport)

            self.ui.thread.start()

        self.ui.config_tab.is_lidar_num_changed = False

    def IsSetPath(self):
        if self.path_file_str == '':
            error_message = 'The \'' + self.label_str + '\' is not set'
            widget = QWidget()

            qr = widget.frameGeometry()
            cp = QDesktopWidget().availableGeometry().center()
            qr.moveCenter(cp)
            widget.move(qr.topLeft())

            QMessageBox.information(widget, 'Information', error_message)

            return True
        return False

    def ErrorPopUp(self, error_message):
        widget = QWidget()

        qr = widget.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        widget.move(qr.topLeft())

        QMessageBox.information(widget, 'Information', error_message)

    def CheckGnssFile(self):
        self.ui.importing.gnss_logging_file = self.path_file_str
        self.ui.RemoveLayout(self.ui.importing_tab.gnss_scroll_box.layout)
        if os.path.isfile(self.ui.importing.gnss_logging_file + '/Gnss.csv'):
            self.ui.importing.has_gnss_file = True

        if os.path.isfile(self.ui.importing.gnss_logging_file + '/Motion.csv'):
            self.ui.importing.has_motion_file = True

    def CheckPointCloudFile(self):
        self.ui.importing.point_cloud_logging_path = self.path_file_str
        self.ui.RemoveLayout(self.ui.importing_tab.lidar_scroll_box.layout)
        has_pointcloud_file = True
        for idxSensor in self.ui.config.PARM_LIDAR['CheckedSensorList']:
            if not os.path.isfile(self.ui.importing.point_cloud_logging_path + '/XYZRGB_' + str(
                    idxSensor) + '.bin') == True:
                has_pointcloud_file = False

        return has_pointcloud_file

    def GenerateCSVBtn(self):
        self.ui.importing.gnss_logging_file = self.path_file_str
        self.ui.RemoveLayout(self.ui.importing_tab.gnss_scroll_box.layout)
        if os.path.isfile(self.ui.importing.gnss_logging_file + '/Gnss.csv') == True:
            self.gnss_button = Button('Gnss.csv', CONST_GREEN, CONST_GNSS, self.ui.config.PATH['Image_path'])
            self.ui.importing_tab.gnss_scroll_box.layout.addWidget(self.gnss_button)
        else:
            self.gnss_button = Button('Gnss.csv', CONST_RED, CONST_GNSS, self.ui.config.PATH['Image_path'])
            self.ui.importing_tab.gnss_scroll_box.layout.addWidget(self.gnss_button)

        if os.path.isfile(self.ui.importing.gnss_logging_file + '/Motion.csv') == True:
            self.motion_button = Button('Motion.csv', CONST_GREEN, CONST_GNSS, self.ui.config.PATH['Image_path'])
            self.ui.importing_tab.gnss_scroll_box.layout.addWidget(self.motion_button)
        else:
            self.motion_button = Button('Motion.csv', CONST_RED, CONST_GNSS, self.ui.config.PATH['Image_path'])
            self.ui.importing_tab.gnss_scroll_box.layout.addWidget(self.motion_button)

    def GeneratePointCloudBtn(self):
        self.ui.importing.point_cloud_logging_path = self.path_file_str
        self.ui.RemoveLayout(self.ui.importing_tab.lidar_scroll_box.layout)
        self.lidar_buttons = {}
        for idxSensor in self.ui.config.PARM_LIDAR['CheckedSensorList']:
            if os.path.isfile(self.ui.importing.point_cloud_logging_path + '/XYZRGB_' + str(
                    idxSensor) + '.bin') == True:
                ## Add button
                btn = Button('XYZRGB {}'.format(idxSensor), CONST_GREEN, CONST_LIDAR,
                             self.ui.config.PATH['Image_path'])
                self.lidar_buttons[idxSensor] = btn
                self.ui.importing_tab.lidar_scroll_box.layout.addWidget(btn)
            else:
                ## Add button
                btn = Button('XYZRGB {}'.format(idxSensor), CONST_RED, CONST_LIDAR,
                             self.ui.config.PATH['Image_path'])
                self.lidar_buttons[idxSensor] = btn
                self.ui.importing_tab.lidar_scroll_box.layout.addWidget(btn)

class CheckBoxListLayout(QVBoxLayout):
    instance_num = 1
    def __init__(self, ui, label_str=None):
        super().__init__()
        self.id = CheckBoxListLayout.instance_num
        CheckBoxListLayout.instance_num += 1

        self.label_str = label_str
        self.ui = ui
        self.LiDAR_list = []
        self.lidar_buttons = {}

        self.InitUi()

    def InitUi(self):
        if self.label_str is not None:
            self.label = QLabel(self.label_str)
            self.addWidget(self.label)

        if self.id == 1: # Instance name is 'select_using_sensor_list_layout'
            self.config_scroll_box = ScrollAreaH()
            self.addWidget(self.config_scroll_box)
        else:
            self.listWidget = QListWidget()
            self.listWidget.itemChanged.connect(self.ItemChanged)
            self.addWidget(self.listWidget)

    def AddWidgetItem(self, PARM_LIDAR_SENSOR_LIST, PARM_LIDAR_CHECKED_SENSOR_LIST):
        self.LiDAR_list.clear()
        self.lidar_buttons.clear()
        if self.id == 1:    # instance name is 'select_using_sensor_list_layout'
            self.ui.RemoveLayout(self.config_scroll_box.layout)
        else:
            listItems = self.listWidget.count()
            if not listItems == 0:
                for item_index in reversed(range(listItems)):
                    self.listWidget.takeItem(item_index)
                    self.listWidget.removeItemWidget(self.listWidget.takeItem(item_index))

        self.button_group = QButtonGroup()

        ## Adding configuration tab sensor list
        if self.id == 1:    # instance name is 'select_using_sensor_list_layout'
            for sensor_index in PARM_LIDAR_SENSOR_LIST:
                label = 'XYZRGB ' + str(sensor_index)
                ## Add button
                if sensor_index in PARM_LIDAR_CHECKED_SENSOR_LIST:
                    check_btn = CheckButton(click_status=True,
                                            label_str=label,
                                            color=CONST_GREEN,
                                            btn_type=CONST_LIDAR,
                                            image_path=self.ui.config.PATH['Image_path'],
                                            callback=self.ItemChanged)

                else:
                    check_btn = CheckButton(click_status=False,
                                            label_str=label,
                                            color=CONST_RED,
                                            btn_type=CONST_LIDAR,
                                            image_path=self.ui.config.PATH['Image_path'],
                                            callback=self.ItemChanged)
                self.lidar_buttons[sensor_index] = check_btn
                self.config_scroll_box.layout.addLayout(check_btn)

        ## Adding optimization tab sensor list
        elif self.id == 2:    # instance name is 'select_principle_sensor_list_layout'
            for sensor_index in PARM_LIDAR_CHECKED_SENSOR_LIST:
                item = QListWidgetItem()
                item.setFlags(Qt.ItemIsEnabled)
                self.listWidget.addItem(item)

                radio_btn = QRadioButton('LiDAR %i' % sensor_index)
                if sensor_index == self.ui.config.PARM_LIDAR['PrincipalSensor']:
                    radio_btn.setChecked(True)
                radio_btn.clicked.connect(self.SetPrincipalSensor)
                self.button_group.addButton(radio_btn, sensor_index)

                self.listWidget.setItemWidget(item, radio_btn)
                self.LiDAR_list.append(item)

    def ItemChanged(self):
        items = []
        if self.id == 1:
            for key in self.lidar_buttons.keys():
                status = self.lidar_buttons[key].btn.status
                if status == 'green':
                    items.append(key)
        else:
            for item in self.LiDAR_list:
                words = item.text().split()
                if not item.checkState() == 0:
                    items.append(int(words[1]))

        if self.id == 1:    # instance name is 'select_using_sensor_list_layout'
            self.ui.config.PARM_LIDAR['CheckedSensorList'] = copy.deepcopy(items)
            self.ui.evaluation_tab.eval_lidar['CheckedSensorList'] = copy.deepcopy(items)

            if not len(items) == 0:
                configuration_first_checked_sensor = items[0]
                checked_principal_sensor = self.ui.optimization_tab.select_principle_sensor_list_layout.button_group.checkedId()

                if checked_principal_sensor in items:
                    self.ui.config.PARM_LIDAR['PrincipalSensor'] = checked_principal_sensor
                else:
                    self.ui.config.PARM_LIDAR['PrincipalSensor'] = configuration_first_checked_sensor
            else:
                self.ui.config.PARM_LIDAR['PrincipalSensor'] = None

            self.ui.optimization_tab.select_principle_sensor_list_layout.AddWidgetItem(self.ui.config.PARM_LIDAR['SensorList'], self.ui.config.PARM_LIDAR['CheckedSensorList'])
            self.ui.ResetResultsLabels(self.ui.config.PARM_LIDAR)

    def SetPrincipalSensor(self):
        self.ui.config.PARM_LIDAR['PrincipalSensor'] = self.button_group.checkedId()

class CheckButton(QVBoxLayout):
    def __init__(self, click_status, label_str, color, btn_type, image_path, callback=None):
        super().__init__()
        self.click_status = click_status
        self.label_str = label_str
        self.color = color
        self.btn_type = btn_type
        self.image_path = image_path
        self.callback = callback

        self.InitUi()

    def InitUi(self):
        self.checkbox = QCheckBox()
        self.checkbox.setChecked(self.click_status)
        self.checkbox.stateChanged.connect(self.StateChanged)
        self.addWidget(self.checkbox)

        self.btn = Button(self.label_str, self.color, self.btn_type, self.image_path, click=False)
        self.addWidget(self.btn)

    def StateChanged(self):
        self.btn.ChangeStatus()
        self.callback()

class SpinBoxLabelLayout(QVBoxLayout):
    instance_number = 1
    def __init__(self, label_str, ui):
        super().__init__()
        self.id = SpinBoxLabelLayout.instance_number
        self.label_str = label_str
        self.ui = ui

        SpinBoxLabelLayout.instance_number += 1
        self.InitUi()

    def InitUi(self):
        hbox = QHBoxLayout()

        self.label = QLabel(self.label_str)
        hbox.addWidget(self.label)

        self.spin_box = QSpinBox()
        self.spin_box.setSingleStep(1)
        self.spin_box.setMaximum(1000)
        self.spin_box.setMinimum(0)
        self.spin_box.valueChanged.connect(self.SpinBoxChanged)
        hbox.addWidget(self.spin_box)

        self.addLayout(hbox)

    def SpinBoxChanged(self):
        self.ui.value_changed = True
        if self.label_str == 'LiDAR Num':
            ## Check PARM_LIDAR is empty
            if self.ui.config.PARM_LIDAR.get('SensorList') == None:
                return False
            if len(self.ui.config.PARM_LIDAR['SensorList']) <= 0:
                return False

            self.ui.handeye.complete_calibration = False
            self.ui.optimization.complete_calibration = False
            self.ui.config_tab.is_lidar_num_changed = True

            is_minus = False
            last_lidar_num = self.ui.config.PARM_LIDAR['SensorList'][-1]
            spin_box_value = self.spin_box.value()
            PARM_LIDAR_num = len(self.ui.config.PARM_LIDAR['SensorList'])
            add_lidar_num = spin_box_value - PARM_LIDAR_num

            ## Determine adding or delete lidar list
            if add_lidar_num < 0:
                is_minus = True
                add_lidar_num = add_lidar_num * -1

            if is_minus:
                for i in range(add_lidar_num):
                    if len(self.ui.config.PARM_LIDAR['SensorList']) <= 1:
                        self.spin_box.setValue(1)
                        return False

                    sensor_index = self.ui.config.PARM_LIDAR['SensorList'][-1]
                    if sensor_index in self.ui.config.PARM_LIDAR['CheckedSensorList']:
                        list_index = self.ui.config.PARM_LIDAR['CheckedSensorList'].index(sensor_index)
                        del self.ui.config.PARM_LIDAR['CheckedSensorList'][list_index]
                    if sensor_index in self.ui.evaluation_tab.eval_lidar['CheckedSensorList']:
                        list_index = self.ui.evaluation_tab.eval_lidar['CheckedSensorList'].index(sensor_index)
                        del self.ui.evaluation_tab.eval_lidar['CheckedSensorList'][list_index]
                    del self.ui.config.PARM_LIDAR['SensorList'][-1]
            else:
                for i in range(add_lidar_num):
                    self.ui.config.PARM_LIDAR['SensorList'].append(last_lidar_num + 1)
                    self.ui.config.PARM_LIDAR['CheckedSensorList'].append(last_lidar_num + 1)
                    self.ui.evaluation_tab.eval_lidar['CheckedSensorList'].append(last_lidar_num + 1)
                    last_lidar_num = last_lidar_num + 1

            for idxSensor in self.ui.config.PARM_LIDAR['SensorList']:
                if self.ui.config.CalibrationParam.get(idxSensor):
                    continue
                calib = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.ui.config.CalibrationParam[idxSensor] = calib

            ## Add widget item of lidar list in configuration tab and optimization tab
            self.ui.config_tab.select_using_sensor_list_layout.AddWidgetItem(self.ui.config.PARM_LIDAR['SensorList'], self.ui.config.PARM_LIDAR['CheckedSensorList'])
            self.ui.optimization_tab.select_principle_sensor_list_layout.AddWidgetItem(self.ui.config.PARM_LIDAR['SensorList'], self.ui.config.PARM_LIDAR['CheckedSensorList'])

            ## Add Reset result label in rph tab, handeye tab, optimization tab and evaulation tab
            self.ui.ResetResultsLabels(self.ui.config.PARM_LIDAR)

        elif self.label_str == 'Sampling Interval [Count]':
            self.ui.config.PARM_IM['SamplingInterval'] = self.spin_box.value()

        elif self.label_str == 'Maximum Iteration [Count]':
            self.ui.config.PARM_HE['MaximumIteration'] = self.spin_box.value()

        elif self.label_str == 'Num Points Plane Modeling':
            self.ui.config.PARM_MO['NumPointsPlaneModeling'] = self.spin_box.value()

        elif self.label_str == 'Eval Sampling Interval [Count]':
            self.ui.config.PARM_EV['SamplingInterval'] = self.spin_box.value()

class DoubleSpinBoxLabelLayout(QVBoxLayout):
    instance_number = 1
    def __init__(self, string, ui):
        super().__init__()
        self.id = DoubleSpinBoxLabelLayout.instance_number
        self.label_str = string
        self.ui = ui
        self.text = ''

        DoubleSpinBoxLabelLayout.instance_number += 1
        self.InitUi()

    def InitUi(self):
        self.h_box = QHBoxLayout()

        self.label = QLabel(self.label_str)
        self.h_box.addWidget(self.label)

        # self.h_box.addStretch(0.3)

        self.double_spin_box = QDoubleSpinBox()
        self.double_spin_box.setSingleStep(0.01)
        self.double_spin_box.setMaximum(1000.0)
        if self.label_str == 'Tolerance':
            self.double_spin_box. setDecimals(12)
        self.double_spin_box.setMinimum(-1000.0)
        self.double_spin_box.valueChanged.connect(self.DoubleSpinBoxChanged)
        self.h_box.addWidget(self.double_spin_box)

        self.addLayout(self.h_box)

    def DoubleSpinBoxChanged(self):
        self.ui.value_changed = True
        if self.id == CONST_CONFIG_MINIMUM_THRESHOLD_DISTANCE: # Configuration tab Minimum Threshold Distance [m]
            self.ui.config.PARM_PC['MinThresholdDist_m'] = self.double_spin_box.value()
        elif self.id == CONST_CONFIG_MAXIMUM_THRESHOLD_DISTANCE:  # Configuration tab Maximum Threshold Distance [m]
            self.ui.config.PARM_PC['MaxThresholdDist_m'] = self.double_spin_box.value()
        elif self.id == CONST_CONFIG_MINIMUM_THRESHOLD_X:  # Configuration tab Minimum Threshold X [m]
            self.ui.config.PARM_PC['MinThresholdX_m'] = self.double_spin_box.value()
        elif self.id == CONST_CONFIG_MAXIMUM_THRESHOLD_X:  # Configuration tab Maximum Threshold X [m]
            self.ui.config.PARM_PC['MaxThresholdX_m'] = self.double_spin_box.value()
        elif self.id == CONST_CONFIG_MINIMUM_THRESHOLD_Y:  # Configuration tab Minimum Threshold Y [m]
            self.ui.config.PARM_PC['MinThresholdY_m'] = self.double_spin_box.value()
        elif self.id == CONST_CONFIG_MAXIMUM_THRESHOLD_Y:  # Configuration tab Maximum Threshold Y [m]
            self.ui.config.PARM_PC['MaxThresholdY_m'] = self.double_spin_box.value()
        elif self.id == CONST_CONFIG_MINIMUM_THRESHOLD_Z:  # Configuration tab Minimum Threshold Z [m]
            self.ui.config.PARM_PC['MinThresholdZ_m'] = self.double_spin_box.value()
        elif self.id == CONST_CONFIG_MAXIMUM_THRESHOLD_Z:  # Configuration tab Minimum Threshold Distance [m]
            self.ui.config.PARM_PC['MaxThresholdZ_m'] = self.double_spin_box.value()

        elif self.id == CONST_IMPORT_VEHICLE_MINIMUM_SPEED :  # Import tab Vehicle Minimum Speed [km/h]
            self.ui.config.PARM_IM['VehicleSpeedThreshold'] = self.double_spin_box.value()

        elif self.id == CONST_HANDEYE_TOLERANCE:  # Handeye tab Tolerance
            self.ui.config.PARM_HE['Tolerance'] = self.double_spin_box.value()
        elif self.id == CONST_HANDEYE_OUTLIER_DISTANCE:  # Handeye tab Outlier Distance [m]
            self.ui.config.PARM_HE['OutlierDistance_m'] = self.double_spin_box.value()
        elif self.id == CONST_HANDEYE_HEADING_THRESHOLD:  # Handeye tab Heading Threshold (filter)
            self.ui.config.PARM_HE['filter_HeadingThreshold'] = self.double_spin_box.value()
        elif self.id == CONST_HANDEYE_DISTANCE_THRESHOLD:  # Handeye tab Distance Threshold (filter)
            self.ui.config.PARM_HE['filter_DistanceThreshold'] = self.double_spin_box.value()

        elif self.id == CONST_OPTI_POINT_SAMPLING_RATIO:  # Optimization tab Point Sampling Ratio
            self.ui.config.PARM_MO['PointSamplingRatio'] = self.double_spin_box.value()
        elif self.id == CONST_OPTI_OUTLIER_DISTANCE:  # Optimization tab Outlier Distance [m]
            self.ui.config.PARM_MO['OutlierDistance_m'] = self.double_spin_box.value()

        elif self.id == CONST_EVAL_VEHICLE_MINIMUM_SPEED:  # Evaluation tab Eval Vehicle Minimum Speed [km/h]
            self.ui.config.PARM_EV['VehicleSpeedThreshold'] = self.double_spin_box.value()

class SlideLabelLayouts(QVBoxLayout):
    instance_num = 1
    def __init__(self, ui, label_str=None):
        super().__init__()
        self.id = SlideLabelLayouts.instance_num
        SlideLabelLayouts.instance_num += 1

        self.ui = ui
        self.label_str = label_str

        self.end_editing_finished = False
        self.start_editing_finished = False

        self.start_time = 0.0
        self.end_time = 0.0

        self.InitUi()

    def InitUi(self):
        if self.label_str is not None:
            limit_time_label = QLabel(self.label_str)
            self.addWidget(limit_time_label)

        self.start_time_layout = SlideLabelLayout('Start Time [s]:', self)
        self.addLayout(self.start_time_layout)
        self.end_time_layout = SlideLabelLayout('End Time [s]:', self)
        self.addLayout(self.end_time_layout)

    def SliderChanged(self):
        label_str = self.CheckChangedSlideLabelLayout()
        if label_str == 'Start Time [s]:':
            if self.start_editing_finished:
                self.start_editing_finished = False
            else:
                end_time = self.end_time_layout.double_spin_box.value()
                if self.start_time_layout.slider.value() > end_time:
                    self.start_time_layout.double_spin_box.setValue(end_time)
                    self.start_time_layout.slider.setValue(end_time)
                else:
                    self.start_time_layout.double_spin_box.setValue(self.start_time_layout.slider.value())

            self.start_time_layout.prev_slider_value = self.start_time_layout.slider.value()
            self.start_time_layout.prev_double_spin_box_value = self.start_time_layout.double_spin_box.value()
            self.start_time = self.start_time_layout.double_spin_box.value()

        elif label_str == 'End Time [s]:':
            if self.end_editing_finished:
                self.end_editing_finished = False
            else:
                start_time = self.start_time_layout.double_spin_box.value()
                if self.end_time_layout.slider.value() < start_time:
                    self.end_time_layout.double_spin_box.setValue(start_time)
                    self.end_time_layout.slider.setValue(start_time)
                else:
                    self.end_time_layout.double_spin_box.setValue(self.end_time_layout.slider.value())

            self.end_time_layout.prev_slider_value = self.end_time_layout.slider.value()
            self.end_time_layout.prev_double_spin_box_value = self.end_time_layout.double_spin_box.value()
            self.end_time = self.end_time_layout.double_spin_box.value()

    def DoubleSpinBoxChanged(self):
        label_str = self.CheckChangedDoubleSpinBoxLayout()
        if label_str == 'Start Time [s]:':
            self.start_editing_finished = True

            end_time = self.end_time_layout.double_spin_box.value()
            if self.start_time_layout.double_spin_box.value() > end_time:
                self.start_time_layout.double_spin_box.setValue(end_time)
                self.start_time_layout.slider.setValue(end_time)
                self.ErrorPopUp('Start time cannot be higher than end time')
            else:
                self.start_time_layout.slider.setValue(self.start_time_layout.double_spin_box.value())

            self.start_time_layout.prev_slider_value = self.start_time_layout.slider.value()
            self.start_time_layout.prev_double_spin_box_value = self.start_time_layout.double_spin_box.value()
            self.start_time = self.start_time_layout.double_spin_box.value()

        elif label_str == 'End Time [s]:':
            self.end_editing_finished = True

            start_time = self.start_time_layout.double_spin_box.value()
            if self.end_time_layout.double_spin_box.value() < start_time:
                self.end_time_layout.double_spin_box.setValue(start_time)
                self.end_time_layout.slider.setValue(start_time)
                self.ErrorPopUp('End time cannot be lower than start time')
            else:
                self.end_time_layout.slider.setValue(self.end_time_layout.double_spin_box.value())

            self.end_time_layout.prev_slider_value = self.end_time_layout.slider.value()
            self.end_time_layout.prev_double_spin_box_value = self.end_time_layout.double_spin_box.value()
            self.end_time = self.end_time_layout.double_spin_box.value()

    def ErrorPopUp(self, error_message):
        widget = QWidget()

        qr = widget.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        widget.move(qr.topLeft())

        QMessageBox.information(widget, 'Information', error_message)

    def CheckChangedSlideLabelLayout(self):
        prev_start_time = self.start_time_layout.prev_slider_value
        curr_start_time = self.start_time_layout.slider.value()

        prev_end_time = self.end_time_layout.prev_slider_value
        curr_end_time = self.end_time_layout.slider.value()

        if not prev_start_time == curr_start_time:
            return self.start_time_layout.label_str
        elif not prev_end_time == curr_end_time:
            return self.end_time_layout.label_str
        else:
            return 'Slider nothing'

    def CheckChangedDoubleSpinBoxLayout(self):
        prev_start_time = self.start_time_layout.prev_double_spin_box_value
        curr_start_time = self.start_time_layout.double_spin_box.value()

        prev_end_time = self.end_time_layout.prev_double_spin_box_value
        curr_end_time = self.end_time_layout.double_spin_box.value()

        if not prev_start_time == curr_start_time:
            return self.start_time_layout.label_str
        elif not prev_end_time == curr_end_time:
            return self.end_time_layout.label_str
        else:
            return 'Double nothing'

class SlideLabelLayout(QGridLayout):
    def __init__(self, label_str, parent):
        super().__init__()
        self.label_str = label_str
        self.parent = parent
        self.InitUi()

        self.prev_slider_value = 0.0
        self.prev_double_spin_box_value = 0.0

    def InitUi(self):
        self.slider = DoubleSlider(Qt.Horizontal)
        self.slider.valueChanged.connect(self.parent.SliderChanged)
        self.prev_slider_value = self.slider.value()
        self.addWidget(self.slider, 0, 0)

        self.label = QLabel(self.label_str)
        self.addWidget(self.label, 1, 0)

        self.double_spin_box = QDoubleSpinBox()
        self.double_spin_box.setSingleStep(10)
        self.double_spin_box.setMaximum(10000.0)
        self.double_spin_box.setMinimum(0.0)
        self.double_spin_box.setValue(self.slider.value())
        self.double_spin_box.editingFinished.connect(self.parent.DoubleSpinBoxChanged)
        self.prev_double_spin_box_value = self.double_spin_box.value()
        self.addWidget(self.double_spin_box, 1, 1)

class CalibrationResultEditLabel(QVBoxLayout):
    def __init__(self, id, idxSensor, calibration_param, ui):
        super().__init__()
        self.id = id
        self.idxSensor = idxSensor
        self.calibration_param = calibration_param
        self.ui = ui

        self.InitUi()

    def InitUi(self):
        self.label = QLabel('LiDAR {}'.format(self.idxSensor))
        self.addWidget(self.label)

        self.hbox = QHBoxLayout()
        self.double_spin_box_x = QDoubleSpinBox()
        self.double_spin_box_x.setSingleStep(0.01)
        self.double_spin_box_x.setMaximum(10000.0)
        self.double_spin_box_x.setMinimum(-10000.0)
        self.hbox.addWidget(self.double_spin_box_x)

        self.double_spin_box_y = QDoubleSpinBox()
        self.double_spin_box_y.setSingleStep(0.01)
        self.double_spin_box_y.setMaximum(10000.0)
        self.double_spin_box_y.setMinimum(-10000.0)
        self.hbox.addWidget(self.double_spin_box_y)

        self.double_spin_box_yaw = QDoubleSpinBox()
        self.double_spin_box_yaw.setSingleStep(0.01)
        self.double_spin_box_yaw.setMaximum(10000.0)
        self.double_spin_box_yaw.setMinimum(-10000.0)
        self.hbox.addWidget(self.double_spin_box_yaw)

        self.change_btn = QPushButton('Change')
        self.change_btn.clicked.connect(self.DisplayCalibrationGraph)
        self.hbox.addWidget(self.change_btn)

        self.addLayout(self.hbox)

    def DisplayCalibrationGraph(self):
        status = self.ui.evaluation_tab.button_group.checkedId()
        if not self.id == status:
            return False

        self.calibration_param[self.idxSensor][3] = self.double_spin_box_x.value() * math.pi / 180.0
        self.calibration_param[self.idxSensor][4] = self.double_spin_box_y.value() * math.pi / 180.0
        self.calibration_param[self.idxSensor][2] = self.double_spin_box_yaw.value() * math.pi / 180.0

        self.ui.evaluation_tab.DisplayCalibrationGraph()

class CalibrationResultLabel(QVBoxLayout):
    def __init__(self, idxSensor):
        super().__init__()
        self.idxSensor = idxSensor

        self.InitUi()

    def InitUi(self):
        self.label = QLabel('LiDAR {}'.format(self.idxSensor))
        self.addWidget(self.label)
        self.hbox = QHBoxLayout()
        self.label_x = QLabel('x [m]')
        self.hbox.addWidget(self.label_x)
        self.label_edit_x = QLineEdit()
        self.label_edit_x.setReadOnly(True)
        self.label_edit_x.setText('0.0')
        self.label_edit_x.setStyleSheet("background-color: #F0F0F0;")
        self.hbox.addWidget(self.label_edit_x)

        self.label_y = QLabel('y [m]')
        self.hbox.addWidget(self.label_y)
        self.label_edit_y = QLineEdit()
        self.label_edit_y.setReadOnly(True)
        self.label_edit_y.setText('0.0')
        self.label_edit_y.setStyleSheet("background-color: #F0F0F0;")
        self.hbox.addWidget(self.label_edit_y)

        self.label_yaw = QLabel('yaw [deg]')
        self.hbox.addWidget(self.label_yaw)
        self.label_edit_yaw = QLineEdit()
        self.label_edit_yaw.setReadOnly(True)
        self.label_edit_yaw.setText('0.0')
        self.label_edit_yaw.setStyleSheet("background-color: #F0F0F0;")
        self.hbox.addWidget(self.label_edit_yaw)

        self.addLayout(self.hbox)

class CalibrationResultEditLabel2(QVBoxLayout):
    def __init__(self, idxSensor, calibration_param, ui):
        super().__init__()
        self.idxSensor = idxSensor
        # self.calibration_param = calibration_param
        self.ui = ui
        self.calibration_param = self.ui.optimization_tab.edit_handeye_calibration_parm

        self.InitUi()

    def InitUi(self):
        self.addLayout(self.layer1())
        self.addLayout(self.layer2())

    def layer1(self):
        hbox = QHBoxLayout()

        label = QLabel('LiDAR {}'.format(self.idxSensor))
        hbox.addWidget(label)

        self.button = QPushButton('Apply')
        self.button.clicked.connect(self.SetCaliPARM)
        hbox.addWidget(self.button)

        return hbox

    def layer2(self):
        hbox = QHBoxLayout()
        label = QLabel('X [m]')
        hbox.addWidget(label)

        self.double_spin_box_x = QDoubleSpinBox()
        self.double_spin_box_x.setSingleStep(0.01)
        self.double_spin_box_x.setMaximum(10000.0)
        self.double_spin_box_x.setMinimum(-10000.0)
        self.double_spin_box_x.setDecimals(4)
        hbox.addWidget(self.double_spin_box_x)

        label = QLabel('Y [m]')
        hbox.addWidget(label)

        self.double_spin_box_y = QDoubleSpinBox()
        self.double_spin_box_y.setSingleStep(0.01)
        self.double_spin_box_y.setMaximum(10000.0)
        self.double_spin_box_y.setMinimum(-10000.0)
        self.double_spin_box_y.setDecimals(4)
        hbox.addWidget(self.double_spin_box_y)

        label = QLabel('Yaw [deg]')
        hbox.addWidget(label)

        self.double_spin_box_yaw = QDoubleSpinBox()
        self.double_spin_box_yaw.setSingleStep(0.01)
        self.double_spin_box_yaw.setMaximum(10000.0)
        self.double_spin_box_yaw.setMinimum(-10000.0)
        self.double_spin_box_yaw.setDecimals(4)
        hbox.addWidget(self.double_spin_box_yaw)

        return hbox

    def SetCaliPARM(self):
        if self.calibration_param.get(self.idxSensor) == None:
            return False

        x = self.double_spin_box_x.value()
        y = self.double_spin_box_y.value()
        yaw_deg = self.double_spin_box_yaw.value()
        self.calibration_param[self.idxSensor][2] = yaw_deg * math.pi / 180.0
        self.calibration_param[self.idxSensor][3] = x
        self.calibration_param[self.idxSensor][4] = y

        self.ui.optimization.CalibrationParam[self.idxSensor] = copy.deepcopy(self.calibration_param[self.idxSensor])

        lidar = 'Set Lidar {} initial value\n'.format(self.idxSensor)
        changed_value = 'X: ' + str(round(x, 2)) + ' [m], Y: ' + str(round(y, 2)) + ' [m], Yaw: ' + str(round(yaw_deg, 2)) + ' [Deg]\n'
        message = lidar + changed_value

        self.PopUp(message)

    def PopUp(self, message):
        widget = QWidget()

        qr = widget.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        widget.move(qr.topLeft())

        QMessageBox.information(widget, 'Information', message)

class ResultTab(QVBoxLayout):
    def __init__(self, ui):
        super().__init__()
        self.ui = ui

        self.initUI()

    def initUI(self):
        self.handeye_init_value = self.HandEyeInit()
        self.user_define_init_value = self.UserDefineInit()

        self.tabs = QTabWidget()
        self.tabs.addTab(self.handeye_init_value, 'Handeye')
        self.tabs.addTab(self.user_define_init_value, 'User Define')
        self.addWidget(self.tabs)

    def UserDefineInit(self):
        self.user_define_scroll_box = ScrollAreaV()

        return self.user_define_scroll_box

    def HandEyeInit(self):
        self.handeye_scroll_box = ScrollAreaV()

        return self.handeye_scroll_box

class ImageDisplay(QWidget):
    def __init__(self, path):
        super().__init__()
        self.path = path

        self.InitUi()

    def InitUi(self):
        self.im = QPixmap(self.path + 'config.png')
        self.label = QLabel()
        self.label.setPixmap(self.im)

        self.grid = QGridLayout()
        self.grid.setContentsMargins(100,50,10,5)
        self.grid.addWidget(self.label)
        #self.grid.setHorizontalSpacing(1000)
        self.setLayout(self.grid)

        #self.setGeometry(350,150,100,50)
        self.setWindowTitle("PyQT show image")
        self.show()

class EvaluationLable(QHBoxLayout):
    def __init__(self, idxSensor, ui):
        super().__init__()
        self.prev_checkID = CONST_HANDEYE

        self.idxSensor = idxSensor
        self.ui = ui

        self.InitUi()

    def InitUi(self):
        self.checkbox = QCheckBox('LiDAR {}'.format(str(self.idxSensor)))
        self.checkbox.setChecked(True)
        self.checkbox.stateChanged.connect(self.CheckBox)
        self.addWidget(self.checkbox)

        # button for Handeye calibration
        self.button_group = QButtonGroup()
        rbn1 = QRadioButton()
        rbn1.setChecked(True)
        rbn1.clicked.connect(self.RadioButton)
        self.addWidget(rbn1)
        self.button_group.addButton(rbn1, CONST_HANDEYE)

        # button for optimization calibration
        rbn2 = QRadioButton()
        rbn2.clicked.connect(self.RadioButton)
        self.addWidget(rbn2)
        self.button_group.addButton(rbn2, CONST_OPTIMIZATION)

        # button for custom calibration
        rbn3 = QRadioButton()
        rbn3.clicked.connect(self.RadioButton)
        self.addWidget(rbn3)
        self.button_group.addButton(rbn3, CONST_CUSTOM)

        # spinbox for x
        self.spinbox1 = QDoubleSpinBox()
        self.spinbox1.setReadOnly(True)
        self.spinbox1.setSingleStep(0.01)
        self.spinbox1.setMaximum(1000.0)
        self.spinbox1.setMinimum(-1000.0)
        self.spinbox1.setDecimals(4)
        self.spinbox1.setStyleSheet("background-color: #F0F0F0;")
        self.spinbox1.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spinbox1.valueChanged.connect(self.DoubleSpinBoxChanged1)
        self.addWidget(self.spinbox1)

        # spinbox for y
        self.spinbox2 = QDoubleSpinBox()
        self.spinbox2.setReadOnly(True)
        self.spinbox2.setSingleStep(0.01)
        self.spinbox2.setMaximum(1000.0)
        self.spinbox2.setMinimum(-1000.0)
        self.spinbox2.setDecimals(4)
        self.spinbox2.setStyleSheet("background-color: #F0F0F0;")
        self.spinbox2.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spinbox2.valueChanged.connect(self.DoubleSpinBoxChanged2)
        self.addWidget(self.spinbox2)

        # spinbox for yaw
        self.spinbox3 = QDoubleSpinBox()
        self.spinbox3.setReadOnly(True)
        self.spinbox3.setSingleStep(0.01)
        self.spinbox3.setMaximum(1000.0)
        self.spinbox3.setMinimum(-1000.0)
        self.spinbox3.setDecimals(4)
        self.spinbox3.setStyleSheet("background-color: #F0F0F0;")
        self.spinbox3.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spinbox3.valueChanged.connect(self.DoubleSpinBoxChanged3)
        self.addWidget(self.spinbox3)

    def CheckBox(self):
        if self.checkbox.checkState() == 0:
            if self.idxSensor in self.ui.evaluation_tab.eval_lidar['CheckedSensorList']:
                self.ui.evaluation_tab.eval_lidar['CheckedSensorList'].remove(self.idxSensor)
        else:
            if not self.idxSensor in self.ui.evaluation_tab.eval_lidar['CheckedSensorList']:
                self.ui.evaluation_tab.eval_lidar['CheckedSensorList'].append(self.idxSensor)
                self.ui.evaluation_tab.eval_lidar['CheckedSensorList'].sort()

    def RadioButton(self):
        if self.ui.config_tab.is_lidar_num_changed == True:
            self.button_group.button(self.prev_checkID).setChecked(True)
            self.ui.ErrorPopUp('Please import after changing lidar number')
            return False

        status = self.button_group.checkedId()
        if status == CONST_HANDEYE:
            if not self.ui.handeye.complete_calibration:
                self.button_group.button(self.prev_checkID).setChecked(True)
                self.ui.ErrorPopUp('Please complete the HandEye calibration')
                return False
        elif status == CONST_OPTIMIZATION:
            if not self.ui.optimization.complete_calibration:
                self.button_group.button(self.prev_checkID).setChecked(True)
                self.ui.ErrorPopUp('Please complete the Optimization calibration')
                return False
        elif status == CONST_CUSTOM:
            if self.ui.evaluation_tab.custom_calibration_param.get(self.idxSensor) == None:
                self.ui.evaluation_tab.custom_calibration_param[self.idxSensor] = [0., 0., 0., 0., 0., 0.]

        if status == CONST_CUSTOM:
            self.spinbox1.setReadOnly(False)
            self.spinbox1.setStyleSheet("background-color: #FFFFFF;")
            self.spinbox1.setButtonSymbols(QAbstractSpinBox.UpDownArrows)
            self.spinbox2.setReadOnly(False)
            self.spinbox2.setStyleSheet("background-color: #FFFFFF;")
            self.spinbox2.setButtonSymbols(QAbstractSpinBox.UpDownArrows)
            self.spinbox3.setReadOnly(False)
            self.spinbox3.setStyleSheet("background-color: #FFFFFF;")
            self.spinbox3.setButtonSymbols(QAbstractSpinBox.UpDownArrows)
        else:
            self.spinbox1.setReadOnly(True)
            self.spinbox1.setStyleSheet("background-color: #F0F0F0;")
            self.spinbox1.setButtonSymbols(QAbstractSpinBox.NoButtons)
            self.spinbox2.setReadOnly(True)
            self.spinbox2.setStyleSheet("background-color: #F0F0F0;")
            self.spinbox2.setButtonSymbols(QAbstractSpinBox.NoButtons)
            self.spinbox3.setReadOnly(True)
            self.spinbox3.setStyleSheet("background-color: #F0F0F0;")
            self.spinbox3.setButtonSymbols(QAbstractSpinBox.NoButtons)

            
        if status == CONST_HANDEYE:
            self.spinbox1.setValue(self.ui.handeye.CalibrationParam[self.idxSensor][3])
            self.spinbox2.setValue(self.ui.handeye.CalibrationParam[self.idxSensor][4])
            self.spinbox3.setValue(self.ui.handeye.CalibrationParam[self.idxSensor][2] * 180.0 / math.pi)
        elif status == CONST_OPTIMIZATION:
            self.spinbox1.setValue(self.ui.optimization.CalibrationParam[self.idxSensor][3])
            self.spinbox2.setValue(self.ui.optimization.CalibrationParam[self.idxSensor][4])
            self.spinbox3.setValue(self.ui.optimization.CalibrationParam[self.idxSensor][2] * 180.0 / math.pi)
        elif status == CONST_CUSTOM:
            self.spinbox1.setValue(self.ui.evaluation_tab.custom_calibration_param[self.idxSensor][3])
            self.spinbox2.setValue(self.ui.evaluation_tab.custom_calibration_param[self.idxSensor][4])
            self.spinbox3.setValue(self.ui.evaluation_tab.custom_calibration_param[self.idxSensor][2] * 180.0 / math.pi)

        self.prev_checkID = self.button_group.checkedId()

    def DoubleSpinBoxChanged1(self):
        if self.button_group.checkedId() == CONST_CUSTOM:
            self.ui.evaluation_tab.custom_calibration_param[self.idxSensor][3] = self.spinbox1.value()

    def DoubleSpinBoxChanged2(self):
        if self.button_group.checkedId() == CONST_CUSTOM:
            self.ui.evaluation_tab.custom_calibration_param[self.idxSensor][4] = self.spinbox2.value()

    def DoubleSpinBoxChanged3(self):
        if self.button_group.checkedId() == CONST_CUSTOM:
            self.ui.evaluation_tab.custom_calibration_param[self.idxSensor][2] = self.spinbox3.value() * math.pi / 180.0
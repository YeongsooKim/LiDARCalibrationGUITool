# -*- coding: utf-8 -*-
"""
@author: kimkys768@gmail.com, yondoo20@gmail.com
@date: 2020-09-22
@version: 0.0.2
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import math

from widget.DoubleSlider import DoubleSlider
from widget.QScrollarea import *
from widget.QButton import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
from process import get_result

CONST_DISPLAY_HANDEYE = 0
CONST_DISPLAY_OPTIMIZATION = 1

CONST_GREEN = 0
CONST_RED = 1
CONST_LIDAR = 0
CONST_GNSS = 1

class FileInputWithCheckBtnLayout(QVBoxLayout):
    def __init__(self, label_str, ui):
        super().__init__()
        self.label_str = label_str
        self.path_file_str = ''
        self.ui = ui
        self.parsed_bin = ''
        self.END_LINE_INDEX = 15

        self.InitUi()

    def InitUi(self):
        self.window = QMainWindow()

        hbox1 = QHBoxLayout()
        self.label = QLabel(self.label_str)
        hbox1.addWidget(self.label)
        self.btn = QPushButton('...')
        self.btn.clicked.connect(self.GetFileBtn)
        hbox1.addWidget(self.btn)
        self.addLayout(hbox1)

        hbox2 = QHBoxLayout()
        self.label_edit = QLineEdit()
        hbox2.addWidget(self.label_edit)
        self.import_btn = QPushButton('Import')
        self.import_btn.clicked.connect(self.ImportFile)
        hbox2.addWidget(self.import_btn)
        # self.check_btn = QPushButton('Check')
        # self.check_btn.clicked.connect(self.CheckFile)
        # hbox2.addWidget(self.check_btn)
        self.addLayout(hbox2)

        self.pbar = QProgressBar()
        self.addWidget(self.pbar)

    def GetFileBtn(self):
        widget = QWidget()
        export_file_path = str(QFileDialog.getExistingDirectory(widget, 'Select Directory'))

        if export_file_path:
            self.path_file_str = export_file_path
            self.label_edit.setText(self.path_file_str)

    def ImportFile(self):
        has_file = self.CheckGnssFile()
        if has_file:
            self.ui.importing.ParseGnss()
            self.gnss_button.setText('Gnss.csv 100%')
            self.label_edit.setText(self.path_file_str)

        has_file = self.CheckPointCloudFile()
        if has_file:
            self.ui.thread.SetFunc(self.ui.importing.ParsePointCloud)
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

            self.ui.thread.change_value.connect(self.pbar.setValue)
            self.ui.thread.interation_percentage.connect(self.ui.importing_tab.InterationPercentage)
            self.ui.thread.end.connect(self.ParsePointCloud)
            self.ui.thread.start()

    def CheckFile(self):
        if self.path_file_str:
            self.window.text_edit = QTextEdit()
            self.window.setCentralWidget(self.window.text_edit)
            self.window.setWindowTitle('File Check')

            lines = ''
            if self.label_str == 'Select GNSS Logging File' or self.label_str == 'Initial Data':
                lines = open(self.path_file_str, 'r')
            elif self.label_str == 'Select PointCloud Logging Path':
                lines = self.parsed_bin.splitlines(True)

            for line_no, line in enumerate(lines):
                if line_no == 0:
                    lines = line
                elif line_no < self.END_LINE_INDEX:
                    lines = lines + '\n' + line
                else:
                    lines = lines + '\n' + '...'
                    break

            self.window.text_edit.setText(lines)
            self.window.setGeometry(300, 300, 800, 400)
            self.window.show()

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
        self.RemoveLayout(self.ui.importing_tab.gnss_scroll_box.layout)
        if os.path.isfile(self.ui.importing.gnss_logging_file + '/Gnss.csv') == True:
            self.gnss_button = Button('Gnss.csv', CONST_GREEN, CONST_GNSS, self.ui.config.PATH['Image_path'])
            self.ui.importing_tab.gnss_scroll_box.layout.addWidget(self.gnss_button)
            return True
        else:
            self.gnss_button = Button('Gnss.csv', CONST_RED, CONST_GNSS, self.ui.config.PATH['Image_path'])
            self.ui.importing_tab.gnss_scroll_box.layout.addWidget(self.gnss_button)
            return False

    def CheckPointCloudFile(self):
        self.ui.importing.point_cloud_logging_path = self.path_file_str
        self.RemoveLayout(self.ui.importing_tab.lidar_scroll_box.layout)
        self.exist_arr = []
        self.lidar_buttons = {}
        for idxSensor in self.ui.config.PARM_LIDAR['CheckedSensorList']:
            if os.path.isfile(self.ui.importing.point_cloud_logging_path + '/PointCloud_' + str(idxSensor) + '.bin') == True:
                ## Add button
                btn = Button('PointCloud {}'.format(idxSensor), CONST_GREEN, CONST_LIDAR, self.ui.config.PATH['Image_path'])
                self.lidar_buttons[idxSensor] = btn
                self.ui.importing_tab.lidar_scroll_box.layout.addWidget(btn)

                ## Error check
                self.exist_arr.append(1)
            else:
                ## Add button
                btn = Button('PointCloud {}'.format(idxSensor), CONST_RED, CONST_LIDAR, self.ui.config.PATH['Image_path'])
                self.lidar_buttons[idxSensor] = btn
                self.ui.importing_tab.lidar_scroll_box.layout.addWidget(btn)

                ## Error check
                self.ErrorPopUp('There are no PointCloud' + str(idxSensor) + '.bin file')
                self.exist_arr.append(0)

        non_error = True
        for i in self.exist_arr:
            non_error = non_error * i

        return non_error

    def RemoveLayout(self, target):
        while target.count():
            item = target.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
            else:
                self.RemoveLayout(item)

        layout = target.itemAt(0)
        target.removeItem(layout)

    def ParsePointCloud(self):
        parsed_pandas_dataframe = self.ui.importing.text_pointcloud
        self.parsed_bin = parsed_pandas_dataframe.to_string()

        default_start_time = self.ui.importing.DefaultStartTime
        default_end_time = self.ui.importing.DefaultEndTime

        # set slider default time
        self.ui.importing_tab.start_time_layout.slider.setMaximum(default_end_time)
        self.ui.importing_tab.start_time_layout.slider.setMinimum(default_start_time)
        self.ui.importing_tab.end_time_layout.slider.setMaximum(default_end_time)
        self.ui.importing_tab.end_time_layout.slider.setMinimum(default_start_time)

        # set slider and double_spin_box value
        self.ui.importing_tab.end_time_layout.slider.setValue(self.ui.importing_tab.end_time_layout.slider.maximum())
        self.ui.importing_tab.end_time_layout.double_spin_box.setValue(default_end_time)
        self.ui.importing_tab.start_time_layout.slider.setValue(self.ui.importing_tab.end_time_layout.slider.minimum())
        self.ui.importing_tab.start_time_layout.double_spin_box.setValue(default_start_time)

class CheckBoxListLayout(QHBoxLayout):
    def __init__(self, label_str, ui):
        super().__init__()
        self.label_str = label_str
        self.ui = ui
        self.LiDAR_list = []

        self.InitUi()

    def InitUi(self):
        self.label = QLabel(self.label_str)
        self.addWidget(self.label)

        self.listWidget = QListWidget()
        #TODO resize listWidget
        # if self.label_str is 'Select Principle Sensor List':
        #     self.listWidget.setMinimumWidth(self.listWidget.sizeHintForColumn(0))
        self.listWidget.itemChanged.connect(self.ItemChanged)
        self.addWidget(self.listWidget)

    def AddWidgetItem(self, PARM_LIDAR_SENSOR_LIST, PARM_LIDAR_CHECKED_SENSOR_LIST):
        self.LiDAR_list.clear()
        listItems = self.listWidget.count()
        if not listItems == 0:
            for item_index in reversed(range(listItems)):
                self.listWidget.takeItem(item_index)
                self.listWidget.removeItemWidget(self.listWidget.takeItem(item_index))

        self.button_group = QButtonGroup()
        is_first = True

        ## Adding configuration tab sensor list
        if self.label_str == 'Select Using Sensor List':
            for sensor_index in PARM_LIDAR_SENSOR_LIST:
                item = QListWidgetItem('LiDAR %i' % sensor_index)
                if sensor_index in PARM_LIDAR_CHECKED_SENSOR_LIST:
                    item.setCheckState(Qt.Checked)
                else:
                    item.setCheckState(Qt.Unchecked)
                self.listWidget.addItem(item)
                self.LiDAR_list.append(item)

        ## Adding optimization tab sensor list
        elif self.label_str == 'Select Principle Sensor List':
            for sensor_index in PARM_LIDAR_CHECKED_SENSOR_LIST:
                item = QListWidgetItem()
                item.setFlags(Qt.ItemIsEnabled)
                self.listWidget.addItem(item)

                radio_btn = QRadioButton('LiDAR %i' % sensor_index)
                if is_first:
                    is_first = False
                    radio_btn.setChecked(True)
                radio_btn.clicked.connect(self.SetPrincipalSensor)
                self.button_group.addButton(radio_btn, sensor_index)

                self.listWidget.setItemWidget(item, radio_btn)
                self.LiDAR_list.append(item)

    def ItemChanged(self):
        items = []
        for item in self.LiDAR_list:
            words = item.text().split()
            if not item.checkState() == 0:
                items.append(int(words[1]))
        if self.label_str == 'Select Using Sensor List':
            self.ui.config.PARM_LIDAR['CheckedSensorList'] = items
            self.ui.optimization_tab.select_principle_sensor_list_layout.AddWidgetItem(self.ui.config.PARM_LIDAR['SensorList'], self.ui.config.PARM_LIDAR['CheckedSensorList'])
            self.ui.ResetResultsLabels()

    def SetPrincipalSensor(self):
        self.ui.config.PARM_LIDAR['PrincipalSensor'] = self.button_group.checkedId()

class SpinBoxLabelLayout(QVBoxLayout):
    def __init__(self, label_str, ui):
        super().__init__()
        self.label_str = label_str
        self.ui = ui

        self.InitUi()

    def InitUi(self):
        hbox = QHBoxLayout()

        self.label = QLabel(self.label_str)
        hbox.addWidget(self.label)

        if not self.label_str == 'LiDAR Num':
            hbox.addStretch(1.2)

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

            is_minus = False
            last_lidar_num = self.ui.config.PARM_LIDAR['SensorList'][-1]
            spin_box_value = self.spin_box.value()
            PARM_LIDAR_num = len(self.ui.config.PARM_LIDAR['SensorList'])
            add_lidar_num = spin_box_value - PARM_LIDAR_num

            ## Determine adding or delete lidar list
            if add_lidar_num < 0:
                is_minus = True
                add_lidar_num = add_lidar_num * -1

            if not is_minus:
                for i in range(add_lidar_num):
                    self.ui.config.PARM_LIDAR['SensorList'].append(last_lidar_num + 1)
                    self.ui.config.PARM_LIDAR['CheckedSensorList'].append(last_lidar_num + 1)
                    last_lidar_num = last_lidar_num + 1
            else:
                for i in range(add_lidar_num):
                    if len(self.ui.config.PARM_LIDAR['SensorList']) <= 1:
                        self.spin_box.setValue(1)
                        return False

                    sensor_index = self.ui.config.PARM_LIDAR['SensorList'][-1]
                    if sensor_index in self.ui.config.PARM_LIDAR['CheckedSensorList']:
                        list_index = self.ui.config.PARM_LIDAR['CheckedSensorList'].index(sensor_index)
                        del self.ui.config.PARM_LIDAR['CheckedSensorList'][list_index]
                    del self.ui.config.PARM_LIDAR['SensorList'][-1]

            for idxSensor in self.ui.config.PARM_LIDAR['SensorList']:
                if self.ui.config.CalibrationParam.get(idxSensor):
                    continue
                calib = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.ui.config.CalibrationParam[idxSensor] = calib

            ## Add widget item of lidar list in configuration tab and optimization tab
            self.ui.config_tab.select_using_sensor_list_layout.AddWidgetItem(self.ui.config.PARM_LIDAR['SensorList'], self.ui.config.PARM_LIDAR['CheckedSensorList'])
            self.ui.optimization_tab.select_principle_sensor_list_layout.AddWidgetItem(self.ui.config.PARM_LIDAR['SensorList'], self.ui.config.PARM_LIDAR['CheckedSensorList'])
            ## Add Reset result label in handeye tab, optimization tab and evaulation tab
            self.ui.ResetResultsLabels()

        elif self.label_str == 'Sampling Interval':
            self.ui.config.PARM_HE['SamplingInterval'] = self.spin_box.value()
        elif self.label_str == 'Maximum Iteration':
            self.ui.config.PARM_HE['MaximumIteration'] = self.spin_box.value()
        elif self.label_str == 'Num Points Plane Modeling':
            self.ui.config.PARM_MO['NumPointsPlaneModeling'] = self.spin_box.value()

class DoubleSpinBoxLabelLayout(QVBoxLayout):
    def __init__(self, string, ui):
        super().__init__()
        self.label_str = string
        self.ui = ui
        self.text = ''

        self.InitUi()

    def InitUi(self):
        self.h_box = QHBoxLayout()

        self.label = QLabel(self.label_str)
        self.h_box.addWidget(self.label)

        self.h_box.addStretch(1)

        self.double_spin_box = QDoubleSpinBox()
        self.double_spin_box.setSingleStep(0.01)
        self.double_spin_box.setMaximum(1000.0)
        if self.label_str == 'Heading Threshold (filter)':
            self.double_spin_box.setMinimum(-1000.0)
        self.double_spin_box.setMinimum(0.0)
        self.double_spin_box.valueChanged.connect(self.DoubleSpinBoxChanged)
        self.h_box.addWidget(self.double_spin_box)

        self.addLayout(self.h_box)

    def DoubleSpinBoxChanged(self):
        self.ui.value_changed = True
        if self.label_str == 'Minimum Threshold Distance [m]':
            self.ui.config.PARM_PC['MinThresholdDist_m'] = self.double_spin_box.value()
        elif self.label_str == 'Maximum Threshold Distance [m]':
            self.ui.config.PARM_PC['MaxThresholdDist_m'] = self.double_spin_box.value()
        elif self.label_str == 'Minimum Threshold X [m]':
            self.ui.config.PARM_PC['MinThresholdX_m'] = self.double_spin_box.value()
        elif self.label_str == 'Maximum Threshold X [m]':
            self.ui.config.PARM_PC['MaxThresholdX_m'] = self.double_spin_box.value()
        elif self.label_str == 'Minimum Threshold Y [m]':
            self.ui.config.PARM_PC['MinThresholdY_m'] = self.double_spin_box.value()
        elif self.label_str == 'Maximum Threshold Y [m]':
            self.ui.config.PARM_PC['MaxThresholdY_m'] = self.double_spin_box.value()
        elif self.label_str == 'Minimum Threshold Z [m]':
            self.ui.config.PARM_PC['MinThresholdZ_m'] = self.double_spin_box.value()
        elif self.label_str == 'Maximum Threshold Z [m]':
            self.ui.config.PARM_PC['MaxThresholdZ_m'] = self.double_spin_box.value()

        elif self.label_str == 'Tolerance':
            self.ui.config.PARM_HE['Tolerance'] = self.double_spin_box.value()
        elif self.label_str == 'Outlier Distance [m]':
            self.ui.config.PARM_HE['OutlierDistance_m'] = self.double_spin_box.value()
        elif self.label_str == 'Heading Threshold (filter)':
            self.ui.config.PARM_HE['filter_HeadingThreshold'] = self.double_spin_box.value()
        elif self.label_str == 'Distance Threshold (filter)':
            self.ui.config.PARM_HE['filter_DistanceThreshold'] = self.double_spin_box.value()

        elif self.label_str == 'Pose Sampling Ratio':
            self.ui.config.PARM_MO['PoseSamplingRatio'] = self.double_spin_box.value()
        elif self.label_str == 'Point Sampling Ratio':
            self.ui.config.PARM_MO['PointSamplingRatio'] = self.double_spin_box.value()
        elif self.label_str == 'Outlier Distance  [m]':
            self.ui.config.PARM_MO['OutlierDistance_m'] = self.double_spin_box.value()

class SlideLabelLayout(QGridLayout):
    def __init__(self, label_str, ui):
        super().__init__()
        self.label_str = label_str
        self.ui = ui
        self.end_editing_finished = False
        self.start_editing_finished = False
        self.InitUi()

    def InitUi(self):
        self.slider = DoubleSlider(Qt.Horizontal)
        self.slider.valueChanged.connect(self.SetLabelEdit)
        self.addWidget(self.slider, 0, 0)

        self.label = QLabel(self.label_str)
        self.addWidget(self.label, 1, 0)

        self.double_spin_box = QDoubleSpinBox()
        self.double_spin_box.setSingleStep(0.01)
        self.double_spin_box.setMaximum(10000.0)
        self.double_spin_box.setMinimum(0.0)
        self.double_spin_box.setValue(self.slider.value())
        self.double_spin_box.editingFinished.connect(self.DoubleSpinBoxChanged)
        self.addWidget(self.double_spin_box, 1, 1)

    def SetLabelEdit(self):
        if self.label_str == 'Start Time [s]:':
            if self.start_editing_finished:
                self.start_editing_finished = False
            else:
                end_time = self.ui.importing_tab.end_time_layout.double_spin_box.value()
                if self.slider.value() > end_time:
                    self.double_spin_box.setValue(end_time)
                    self.slider.setValue(end_time)
                else:
                    self.double_spin_box.setValue(self.slider.value())

            self.ui.importing.start_time = self.double_spin_box.value()

        elif self.label_str == 'End Time [s]:':
            if self.end_editing_finished:
                self.end_editing_finished = False
            else:
                start_time = self.ui.importing_tab.start_time_layout.double_spin_box.value()
                if self.slider.value() < start_time:
                    self.double_spin_box.setValue(start_time)
                    self.slider.setValue(start_time)
                else:
                    self.double_spin_box.setValue(self.slider.value())

            self.ui.importing.end_time = self.double_spin_box.value()

    def DoubleSpinBoxChanged(self):
        if self.label_str == 'Start Time [s]:':
            self.start_editing_finished = True

            end_time = self.ui.importing_tab.end_time_layout.double_spin_box.value()
            if self.double_spin_box.value() > end_time:
                self.double_spin_box.setValue(end_time)
                self.slider.setValue(end_time)
                self.ErrorPopUp('Start time cannot be higher than end time')
            else:
                self.slider.setValue(self.double_spin_box.value())
            self.ui.importing.start_time = self.double_spin_box.value()

        elif self.label_str == 'End Time [s]:':
            self.end_editing_finished = True

            start_time = self.ui.importing_tab.start_time_layout.double_spin_box.value()
            if self.double_spin_box.value() < start_time:
                self.double_spin_box.setValue(start_time)
                self.slider.setValue(start_time)
                self.ErrorPopUp('End time cannot be lower than start time')
            else:
                self.slider.setValue(self.double_spin_box.value())
            self.ui.importing.end_time = self.double_spin_box.value()

    def ErrorPopUp(self, error_message):
        widget = QWidget()

        qr = widget.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        widget.move(qr.topLeft())

        QMessageBox.information(widget, 'Information', error_message)

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

        if status == CONST_DISPLAY_HANDEYE:
            df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.ui.config, self.ui.importing, self.ui.handeye.df_info, self.ui.handeye.CalibrationParam)
            calib_x, calib_y, calib_yaw = self.ui.handeye.calib_x, self.ui.handeye.calib_y, self.ui.handeye.calib_yaw
            method = 'HandEye'
        elif status == CONST_DISPLAY_OPTIMIZATION:
            df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.ui.config, self.ui.importing, self.ui.optimization.df_info, self.ui.optimization.CalibrationParam)
            calib_x, calib_y, calib_yaw = self.ui.optimization.calib_x, self.ui.optimization.calib_y, self.ui.optimization.calib_yaw
            method = 'Optimization'

        print('x')
        print(calib_x)
        print('y')
        print(calib_y)
        print('yaw')
        print(calib_yaw)



        color_list = []
        color_list.append('r')
        color_list.append('b')
        color_list.append('g')
        color_list.append('c')
        color_list.append('m')
        color_list.append('y')

        self.calibration_param[self.idxSensor][3] = self.double_spin_box_x.value()
        self.calibration_param[self.idxSensor][4] = self.double_spin_box_y.value()
        self.calibration_param[self.idxSensor][2] = self.double_spin_box_yaw.value()
        df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.ui.config,
                                                                                           self.ui.importing,
                                                                                           self.ui.handeye.df_info,
                                                                                           self.calibration_param)
        self.ui.evaluation_tab.result_before_graph_ax.clear()
        self.ui.evaluation_tab.result_before_graph_ax.plot(df_info['east_m'].values, df_info['north_m'].values, 'k.', label='trajectory')
        num = 0
        for idxSensor in PARM_LIDAR:
            num = num + 1
            strColIndex = 'PointCloud_' + str(idxSensor)
            self.ui.evaluation_tab.result_before_graph_ax.plot(accum_pointcloud_[idxSensor][:, 0], accum_pointcloud_[idxSensor][:, 1],
                                             ',', color = color_list[num-1],
                                             label=strColIndex)

        self.ui.evaluation_tab.result_before_graph_ax.axis('equal')
        self.ui.evaluation_tab.result_before_graph_ax.legend()
        self.ui.evaluation_tab.result_before_graph_ax.grid()
        self.ui.evaluation_tab.result_before_graph_canvas.draw()

        self.ui.evaluation_tab.result_after_graph_ax.clear()
        self.ui.evaluation_tab.result_after_graph_ax.plot(df_info['east_m'].values, df_info['north_m'].values, 'k.', label='trajectory')
        num = 0
        for idxSensor in PARM_LIDAR:
            num = num + 1
            strColIndex = 'PointCloud_' + str(idxSensor)
            self.ui.evaluation_tab.result_after_graph_ax.plot(accum_pointcloud[idxSensor][:, 0], accum_pointcloud[idxSensor][:, 1], ',', color = color_list[num-1],
                                            label=strColIndex)

        self.ui.evaluation_tab.result_after_graph_ax.axis('equal')
        self.ui.evaluation_tab.result_after_graph_ax.legend()
        self.ui.evaluation_tab.result_after_graph_ax.grid()
        self.ui.evaluation_tab.result_after_graph_canvas.draw()
        color_list = []
        color_list.append('r')
        color_list.append('b')
        color_list.append('g')
        color_list.append('c')
        color_list.append('m')
        color_list.append('y')
        veh_path = self.ui.config.PATH['Image_path'] + 'vehicle1.png'
        veh = plt.imread(veh_path)
        self.ui.evaluation_tab.result_data_pose_ax.clear()
        num = 0
        for i in self.ui.config.PARM_LIDAR['CheckedSensorList']:
            num = num + 1

            self.ui.evaluation_tab.result_data_pose_ax.imshow(veh)

            x = int(calib_x[num-1])*200 + 500
            y = 1000 -1*int(calib_y[num-1])*200 - 500

            car_length = 1.75
            lidar_num = 'lidar'+str(i)
            
            self.ui.evaluation_tab.result_data_pose_ax.scatter(x, y, s = 300, label = lidar_num, color = color_list[num-1],edgecolor = 'none', alpha = 0.5)
            self.ui.evaluation_tab.result_data_pose_ax.arrow(x, y, 100*np.cos(calib_yaw[num-1]*np.pi/180), -100*np.sin(calib_yaw[num-1]*np.pi/180), head_width=10, head_length=10, fc='k', ec='k')
            self.ui.evaluation_tab.result_data_pose_ax.plot(np.linspace(500,x,100), np.linspace(500,y,100), color_list[num-1]+'--')

            #ax.axes.xaxis.set_visible(False)
            #ax.axes.yaxis.set_visible(False)
        self.ui.evaluation_tab.result_data_pose_ax.axes.xaxis.set_visible(False)
        self.ui.evaluation_tab.result_data_pose_ax.axes.yaxis.set_visible(False)

        self.ui.evaluation_tab.result_data_pose_ax.set_xlim([-500,1500])
        self.ui.evaluation_tab.result_data_pose_ax.set_ylim([1000,0])
        self.ui.evaluation_tab.result_data_pose_ax.legend()
        self.ui.evaluation_tab.result_data_pose_canvas.draw()
        self.ui.evaluation_tab.result_data_pose_ax.set_title('Result of calibration - '+method)

        print('change btn')

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
        self.label_edit_x.setText('0.0')
        self.hbox.addWidget(self.label_edit_x)

        self.label_y = QLabel('y [m]')
        self.hbox.addWidget(self.label_y)
        self.label_edit_y = QLineEdit()
        self.label_edit_y.setText('0.0')
        self.hbox.addWidget(self.label_edit_y)

        self.label_yaw = QLabel('yaw [deg]')
        self.hbox.addWidget(self.label_yaw)
        self.label_edit_yaw = QLineEdit()
        self.label_edit_yaw.setText('0.0')
        self.hbox.addWidget(self.label_edit_yaw)

        self.addLayout(self.hbox)

class CalibrationResultEditLabel2(QVBoxLayout):
    def __init__(self, idxSensor, calibration_param, ui):
        super().__init__()
        self.idxSensor = idxSensor
        self.calibration_param = calibration_param
        self.ui = ui

        self.InitUi()

    def InitUi(self):
        self.addLayout(self.layer1())
        self.addLayout(self.layer2())
        self.addLayout(self.layer3())

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
        label = QLabel('Roll [deg]')
        hbox.addWidget(label)

        self.double_spin_box_roll = QDoubleSpinBox()
        self.double_spin_box_roll.setSingleStep(0.01)
        self.double_spin_box_roll.setMaximum(10000.0)
        self.double_spin_box_roll.setMinimum(-10000.0)
        hbox.addWidget(self.double_spin_box_roll)

        label = QLabel('Pitch [deg]')
        hbox.addWidget(label)

        self.double_spin_box_pitch = QDoubleSpinBox()
        self.double_spin_box_pitch.setSingleStep(0.01)
        self.double_spin_box_pitch.setMaximum(10000.0)
        self.double_spin_box_pitch.setMinimum(-10000.0)
        hbox.addWidget(self.double_spin_box_pitch)

        label = QLabel('Yaw [deg]')
        hbox.addWidget(label)

        self.double_spin_box_yaw = QDoubleSpinBox()
        self.double_spin_box_yaw.setSingleStep(0.01)
        self.double_spin_box_yaw.setMaximum(10000.0)
        self.double_spin_box_yaw.setMinimum(-10000.0)
        hbox.addWidget(self.double_spin_box_yaw)

        return hbox

    def layer3(self):
        hbox = QHBoxLayout()
        label = QLabel('X [m]')
        hbox.addWidget(label)

        self.double_spin_box_x = QDoubleSpinBox()
        self.double_spin_box_x.setSingleStep(0.01)
        self.double_spin_box_x.setMaximum(10000.0)
        self.double_spin_box_x.setMinimum(-10000.0)
        hbox.addWidget(self.double_spin_box_x)

        label = QLabel('Y [m]')
        hbox.addWidget(label)

        self.double_spin_box_y = QDoubleSpinBox()
        self.double_spin_box_y.setSingleStep(0.01)
        self.double_spin_box_y.setMaximum(10000.0)
        self.double_spin_box_y.setMinimum(-10000.0)
        hbox.addWidget(self.double_spin_box_y)

        label = QLabel('Z [m]')
        hbox.addWidget(label)

        self.double_spin_box_z = QDoubleSpinBox()
        self.double_spin_box_z.setSingleStep(0.01)
        self.double_spin_box_z.setMaximum(10000.0)
        self.double_spin_box_z.setMinimum(-10000.0)
        hbox.addWidget(self.double_spin_box_z)
        return hbox

    def SetCaliPARM(self):
        if self.calibration_param.get(self.idxSensor) == None:
            return False

        roll_deg = self.double_spin_box_roll.value()
        pitch_deg = self.double_spin_box_pitch.value()
        yaw_deg = self.double_spin_box_yaw.value()
        x = self.double_spin_box_x.value()
        y = self.double_spin_box_y.value()
        z = self.double_spin_box_z.value()
        self.calibration_param[self.idxSensor][0] = roll_deg * math.pi / 180
        self.calibration_param[self.idxSensor][1] = pitch_deg * math.pi / 180
        self.calibration_param[self.idxSensor][2] = yaw_deg * math.pi / 180
        self.calibration_param[self.idxSensor][3] = x
        self.calibration_param[self.idxSensor][4] = y
        self.calibration_param[self.idxSensor][5] = z

        self.ui.optimization.initial_calibration_param[self.idxSensor] = self.calibration_param[self.idxSensor]

        lidar = 'Set Lidar {} initial value\n'.format(self.idxSensor)
        roll_pitch_yaw = 'Roll: ' + str(round(roll_deg, 2)) + ' [Deg], Pitch: ' + str(round(pitch_deg, 2)) + ' [Deg], Yaw: ' + str(round(yaw_deg, 2)) + ' [Deg]\n'
        x_y_z = 'X: ' + str(round(x, 2)) + ' [m], Y: ' + str(round(y, 2)) + ' [m], Z: ' + str(round(z, 2)) + ' [m]\n'
        message = lidar + roll_pitch_yaw + x_y_z

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

        tabs = QTabWidget()
        tabs.addTab(self.handeye_init_value, 'Handeye')
        tabs.addTab(self.user_define_init_value, 'User Define')
        self.addWidget(tabs)

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

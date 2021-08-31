# -*- coding: utf-8 -*-
"""
@author: kimkys768@gmail.com, yondoo20@gmail.com
@date: 2020-09-22
@version: 0.0.2
"""

import os
import math
import copy

from gui_partition.widgets.DoubleSlider import DoubleSlider
from gui_partition.widgets.QScrollarea import *
from gui_partition.widgets.QButton import *
from gui_partition.widgets.QUniqDoubleSpinBox import *
from gui_partition.layouts.layout_id import *

from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap

import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

from hmi.visualization import vtk_lidar_calib

class FileInputWithCheckBtnLayout(QVBoxLayout):
    def __init__(self, instance_id, label_str, form_widget):
        super().__init__()
        self.id = instance_id
        self.label_str = label_str
        self.path_file_str = ''
        self.form_widget = form_widget
        self.parsed_bin = ''

        self.InitUi()

    def InitUi(self):
        hbox = QHBoxLayout()

        self.label_edit = QLineEdit()
        self.label_edit.editingFinished.connect(self.LabelTextChange)
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

    def LabelTextChange(self):
        self.path_file_str = self.label_edit.text()

    def GetFileBtn(self):
        '''
        Read the path of the Import File location through the dialog
        And put the path to 'label_edit'
        '''
        # Before import, get import file path by QFileDialog.
        widget = QWidget()
        # export_file_path is string of import file path directory
        export_file_path = str(QFileDialog.getExistingDirectory(widget, 'Select Directory'))

        if export_file_path: # if user set directory as blank. It need to cancel to set directory
            self.path_file_str = export_file_path
            self.label_edit.setText(self.path_file_str)

    def ImportFile(self):
        '''
        Check the file being and Import file data
        '''

        if len(self.form_widget.config.PARM_LIDAR['CheckedSensorList']) == 0:
            self.form_widget.ErrorPopUp('Please import more than 1 lidar')
            return
        self.form_widget.importing_tab.next_btn.setEnabled(False)

        self.form_widget.tabs.setTabEnabled(CONST_CONFIG_TAB, False)
        self.form_widget.tabs.setTabEnabled(CONST_ZROLLPITCH_TAB, False)
        self.form_widget.tabs.setTabEnabled(CONST_VALIDATION_TAB, False)
        self.form_widget.tabs.setTabEnabled(CONST_HANDEYE_TAB, False)
        self.form_widget.tabs.setTabEnabled(CONST_UNSUPERVISED_TAB, False)
        self.form_widget.tabs.setTabEnabled(CONST_EVALUATION_TAB, False)

        self.form_widget.importing.Clear()
        self.CheckGnssFile()
        has_pointcloud_file = self.CheckPointCloudFile()

        # Check file
        if not self.form_widget.importing.has_gnss_file:
            self.form_widget.importing_tab.gnss_initial_pose_layout.SetEnable(False)
            self.form_widget.ErrorPopUp('Gnss.csv is missing\n Please set initial value')
        else:
            self.form_widget.importing_tab.gnss_initial_pose_layout.SetEnable(True)

        if not self.form_widget.importing.has_motion_file:
            self.form_widget.importing_tab.motion_initial_pose_layout.SetEnable(False)
            self.form_widget.ErrorPopUp('Motion.csv is missing\n [Warning] Vehicle Minimum Speed is disabled')
        else:
            self.form_widget.importing_tab.motion_initial_pose_layout.SetEnable(True)

        if not has_pointcloud_file:
            self.form_widget.tabs.setTabEnabled(CONST_CONFIG_TAB, True)
            self.form_widget.ErrorPopUp('XYZRGB.bin is missing')

        # Import file
        self.GenerateCSVBtn()
        if self.form_widget.importing.has_gnss_file:
            self.gnss_button.setText('Gnss.csv 100%')
        else:
            self.gnss_button.setText('Gnss.csv 0%')

        if self.form_widget.importing.has_motion_file:
            self.motion_button.setText('Motion.csv 100%')
        else:
            self.motion_button.setText('Motion.csv 0%')

        self.GeneratePointCloudBtn()

        if (self.form_widget.importing.has_gnss_file or self.form_widget.importing.has_motion_file) and has_pointcloud_file:
            if self.form_widget.importing.has_gnss_file:
                self.form_widget.importing.ParseGnss()
            if self.form_widget.importing.has_motion_file:
                self.form_widget.importing.ParseMotion()

            self.form_widget.thread.SetFunc(self.form_widget.importing.ParsePointCloud)
            self.form_widget.thread._status = True
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

            self.form_widget.thread.change_value.connect(self.pbar.setValue)
            self.form_widget.thread.iteration_percentage.connect(self.form_widget.importing_tab.IterationPercentage)
            self.form_widget.thread.end.connect(self.form_widget.importing_tab.EndImport)

            self.form_widget.thread.start()

        self.form_widget.config_tab.is_lidar_num_changed = False
        self.form_widget.config_tab.is_config_parm_changed = False

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

    def CheckGnssFile(self):
        """
        Checking the GNSS file and Moiton file
        GNSS file is 'file_path/Gnss.csv'
        Motion file is 'file_path/Motion.csv'
        If this files are being, then each flag(has_gnss_file or has_motion_file) be true
        """
        self.form_widget.importing.gnss_logging_file = self.path_file_str
        self.form_widget.RemoveLayout(self.form_widget.importing_tab.gnss_scroll_box.layout)
        if os.path.isfile(self.form_widget.importing.gnss_logging_file + '/Gnss.csv'):
            self.form_widget.importing.has_gnss_file = True
        else:
            self.form_widget.importing.has_gnss_file = False

        if os.path.isfile(self.form_widget.importing.gnss_logging_file + '/Motion.csv'):
            self.form_widget.importing.has_motion_file = True
        else:
            self.form_widget.importing.has_motion_file = False

    def CheckPointCloudFile(self):
        '''
        Checking the PointCloud file
        The path of PointCloud file is 'file_path/XYZRGB_NumOfSensor.bin'
        Check all number of LiDAR, over one of LiDAR PointCloud file is not being then pointcloud's flag will be false.
        '''
        self.form_widget.importing.point_cloud_logging_path = self.path_file_str
        self.form_widget.RemoveLayout(self.form_widget.importing_tab.lidar_scroll_box.layout)
        has_pointcloud_file = True
        for idxSensor in self.form_widget.config.PARM_LIDAR['CheckedSensorList']:
            if not os.path.isfile(self.form_widget.importing.point_cloud_logging_path + '/XYZRGB_' + str(
                    idxSensor) + '.bin') == True:
                has_pointcloud_file = False

        return has_pointcloud_file

    def GenerateCSVBtn(self):
        '''
        This function generate the Buttons(LiDAR and GNSS).
        That buttons display using state of each data by image and loading percent text.
        '''
        self.form_widget.importing.gnss_logging_file = self.path_file_str
        # Clear current buttons
        self.form_widget.RemoveLayout(self.form_widget.importing_tab.gnss_scroll_box.layout)
        if os.path.isfile(self.form_widget.importing.gnss_logging_file + '/Gnss.csv') == True:
            self.gnss_button = Button('Gnss.csv', CONST_GREEN, CONST_GNSS, self.form_widget.config.PATH['Image'])
            self.form_widget.importing_tab.gnss_scroll_box.layout.addWidget(self.gnss_button)
        else:
            self.gnss_button = Button('Gnss.csv', CONST_RED, CONST_GNSS, self.form_widget.config.PATH['Image'])
            self.form_widget.importing_tab.gnss_scroll_box.layout.addWidget(self.gnss_button)

        if os.path.isfile(self.form_widget.importing.gnss_logging_file + '/Motion.csv') == True:
            self.motion_button = Button('Motion.csv', CONST_GREEN, CONST_GNSS, self.form_widget.config.PATH['Image'])
            self.form_widget.importing_tab.gnss_scroll_box.layout.addWidget(self.motion_button)
        else:
            self.motion_button = Button('Motion.csv', CONST_RED, CONST_GNSS, self.form_widget.config.PATH['Image'])
            self.form_widget.importing_tab.gnss_scroll_box.layout.addWidget(self.motion_button)

    def GeneratePointCloudBtn(self):
        '''
        This function generate the LiDAR button in Importng_tap's LiDAR [bin File List] scrollArea.
        '''
        self.form_widget.importing.point_cloud_logging_path = self.path_file_str
        self.form_widget.RemoveLayout(self.form_widget.importing_tab.lidar_scroll_box.layout)
        self.lidar_buttons = {}
        for idxSensor in self.form_widget.config.PARM_LIDAR['CheckedSensorList']:
            if os.path.isfile(self.form_widget.importing.point_cloud_logging_path + '/XYZRGB_' + str(
                    idxSensor) + '.bin') == True:
                ## Add Green Button
                btn = Button('XYZRGB {}'.format(idxSensor), CONST_GREEN, CONST_LIDAR,
                             self.form_widget.config.PATH['Image'])
                self.lidar_buttons[idxSensor] = btn
                self.form_widget.importing_tab.lidar_scroll_box.layout.addWidget(btn)
            else:
                ## Add Red Button
                btn = Button('XYZRGB {}'.format(idxSensor), CONST_RED, CONST_LIDAR,
                             self.form_widget.config.PATH['Image'])
                self.lidar_buttons[idxSensor] = btn
                self.form_widget.importing_tab.lidar_scroll_box.layout.addWidget(btn)

class CheckBoxListLayout(QVBoxLayout):
    def __init__(self, instance_id, form_widget, label_str=None):
        super().__init__()
        self.id = instance_id

        self.label_str = label_str
        self.form_widget = form_widget
        self.LiDAR_list = []
        self.lidar_buttons = {}

        self.InitUi()

    def InitUi(self):
        if self.label_str is not None:
            self.label = QLabel(self.label_str)
            self.addWidget(self.label)

        if self.id == CONST_SELECT_USING_SENSOR_LIST: # Instance name is 'select_using_sensor_list_layout'
            self.config_scroll_box = ScrollAreaH()
            self.addWidget(self.config_scroll_box)

    def AddWidgetItem(self, PARM_LIDAR_SENSOR_LIST, PARM_LIDAR_CHECKED_SENSOR_LIST):
        self.LiDAR_list.clear()
        self.lidar_buttons.clear()
        if self.id == CONST_SELECT_USING_SENSOR_LIST:    # instance name is 'select_using_sensor_list_layout'
            self.form_widget.RemoveLayout(self.config_scroll_box.layout)

        self.button_group = QButtonGroup()

        ## Adding configuration tab sensor list
        if self.id == CONST_SELECT_USING_SENSOR_LIST:    # instance name is 'select_using_sensor_list_layout'
            for sensor_index in PARM_LIDAR_SENSOR_LIST:
                label = 'XYZRGB ' + str(sensor_index)
                ## Add button
                if sensor_index in PARM_LIDAR_CHECKED_SENSOR_LIST:
                    check_btn = CheckButton(click_status=True,
                                            label_str=label,
                                            color=CONST_GREEN,
                                            btn_type=CONST_LIDAR,
                                            image_path=self.form_widget.config.PATH['Image'],
                                            callback=self.ItemChanged)

                else:
                    check_btn = CheckButton(click_status=False,
                                            label_str=label,
                                            color=CONST_RED,
                                            btn_type=CONST_LIDAR,
                                            image_path=self.form_widget.config.PATH['Image'],
                                            callback=self.ItemChanged)
                self.lidar_buttons[sensor_index] = check_btn
                self.config_scroll_box.layout.addLayout(check_btn)

    def ItemChanged(self):
        items = []
        if self.id == CONST_SELECT_USING_SENSOR_LIST:     # Instance name is 'select_using_sensor_list_layout'
            for key in self.lidar_buttons.keys():
                status = self.lidar_buttons[key].btn.status
                if status == 'green':
                    items.append(key)

            self.form_widget.config.PARM_LIDAR['CheckedSensorList'] = copy.deepcopy(items)
            self.form_widget.evaluation_tab.eval_lidar['CheckedSensorList'] = copy.deepcopy(items)

            zrp_calib_result = {}
            for pc in self.form_widget.config.PARM_LIDAR['CheckedSensorList']:
                if self.form_widget.zrollpitch_tab.calib_result.get(pc) is None:
                    zrp_calib_result[pc] = [0.0, 0.0, 0.0]
                else:
                    zrp_calib_result[pc] = self.form_widget.zrollpitch_tab.calib_result[pc]

            self.form_widget.zrollpitch_tab.select_lidar_to_calib_layout.AddWidgetItem(self.form_widget.config.PARM_LIDAR['CheckedSensorList'])
            self.form_widget.unsupervised_tab.select_lidar_combobox_layout.AddWidgetItem(self.form_widget.config.PARM_LIDAR['CheckedSensorList'])
            self.form_widget.ResetResultsLabels(self.form_widget.config.PARM_LIDAR, zrp_calib_result)

            if not len(items) == 0:
                configuration_first_checked_sensor = items[0]

                text = self.form_widget.unsupervised_tab.select_lidar_combobox_layout.cb.currentText()
                words = text.split(' ')
                checked_sensor = int(words[-1])

                if checked_sensor in items:
                    self.form_widget.config.PARM_LIDAR['SingleSensor'] = checked_sensor
                    self.form_widget.config.PARM_LIDAR['PrincipalSensor'] = checked_sensor
                else:
                    self.form_widget.config.PARM_LIDAR['SingleSensor'] = configuration_first_checked_sensor
                    self.form_widget.config.PARM_LIDAR['PrincipalSensor'] = configuration_first_checked_sensor
            else:
                self.form_widget.config.PARM_LIDAR['PrincipalSensor'] = None

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

class ComboBoxLabelLayout(QHBoxLayout):
    def __init__(self, intance_id, label_str, form_widget):
        super().__init__()
        self.id = intance_id
        self.label_str = label_str
        self.form_widget = form_widget

        self.InitUi()

    def InitUi(self):
        hbox = QHBoxLayout()

        self.label = QLabel(self.label_str)
        hbox.addWidget(self.label)

        self.cb = QComboBox()
        self.cb.currentTextChanged.connect(self.TextChanged)
        # valueChanged
        pal = self.cb.palette()
        pal.setColor(QPalette.Button, QColor(255,255,255))
        self.cb.setPalette(pal)
        hbox.addWidget(self.cb)

        self.addLayout(hbox)

    def AddWidgetItem(self, checked_sensor_list):
        if self.id == CONST_ZRP_SELECT_LIDAR_TO_CALIB or CONST_UNSUPERVISED_SELECT_LIDAR_TO_CALIB:    # instance name is 'select_lidar_to_calib_layout', 'select_lidar_combobox_layout'
            self.cb.clear()

            labels = []
            for sensor_index in checked_sensor_list:
                label = 'XYZRGB ' + str(sensor_index)
                labels.append(label)
            self.cb.addItems(labels)

        if self.id == CONST_ZRP_SELECT_LIDAR_TO_CALIB:
            if len(checked_sensor_list) == 0:
                return

            is_default = False
            if self.form_widget.config.PARM_ROI_DICT.get(checked_sensor_list[0]) is None:
                keys = list(self.form_widget.config.PARM_ROI_DICT.keys())
                if len(keys) == 0:
                    is_default = True
                else:
                    idx = keys[0]
            else:
                idx = int(checked_sensor_list[0])

            if is_default:
                self.form_widget.zrollpitch_tab.maximum_x_layout.double_spin_box.setValue(self.form_widget.config.PARM_ZRP['MaxDistanceX_m'])
                self.form_widget.zrollpitch_tab.minimum_x_layout.double_spin_box.setValue(self.form_widget.config.PARM_ZRP['MinDistanceX_m'])
                self.form_widget.zrollpitch_tab.maximum_y_layout.double_spin_box.setValue(self.form_widget.config.PARM_ZRP['MaxDistanceY_m'])
                self.form_widget.zrollpitch_tab.minimum_y_layout.double_spin_box.setValue(self.form_widget.config.PARM_ZRP['MinDistanceY_m'])
                self.form_widget.zrollpitch_tab.maximum_z_layout.double_spin_box.setValue(self.form_widget.config.PARM_ZRP['MaxDistanceZ_m'])
                self.form_widget.zrollpitch_tab.minimum_z_layout.double_spin_box.setValue(self.form_widget.config.PARM_ZRP['MinDistanceZ_m'])
            else:
                self.form_widget.zrollpitch_tab.maximum_x_layout.double_spin_box.setValue(self.form_widget.config.PARM_ROI_DICT[idx]['MaxDistanceX_m'])
                self.form_widget.zrollpitch_tab.minimum_x_layout.double_spin_box.setValue(self.form_widget.config.PARM_ROI_DICT[idx]['MinDistanceX_m'])
                self.form_widget.zrollpitch_tab.maximum_y_layout.double_spin_box.setValue(self.form_widget.config.PARM_ROI_DICT[idx]['MaxDistanceY_m'])
                self.form_widget.zrollpitch_tab.minimum_y_layout.double_spin_box.setValue(self.form_widget.config.PARM_ROI_DICT[idx]['MinDistanceY_m'])
                self.form_widget.zrollpitch_tab.maximum_z_layout.double_spin_box.setValue(self.form_widget.config.PARM_ROI_DICT[idx]['MaxDistanceZ_m'])
                self.form_widget.zrollpitch_tab.minimum_z_layout.double_spin_box.setValue(self.form_widget.config.PARM_ROI_DICT[idx]['MinDistanceZ_m'])

        elif self.id == CONST_UNSUPERVISED_SELECT_LIDAR_TO_CALIB:
            if len(self.form_widget.config.PARM_LIDAR['CheckedSensorList']) < 2:
                self.form_widget.unsupervised_tab.select_lidar_num_layout.button_group.button(1).setChecked(True)
                self.form_widget.unsupervised_tab.select_lidar_num_layout.RadioButton()
                self.form_widget.unsupervised_tab.select_lidar_num_layout.button_group.button(2).setEnabled(False)
            else:
                self.form_widget.unsupervised_tab.select_lidar_num_layout.button_group.button(2).setEnabled(True)

    def Clear(self):
        if self.id == CONST_ZRP_SELECT_LIDAR_TO_CALIB:    # instance name is 'select_lidar_to_calib_layout'
            # Reset ZRP tab result label
            PARM_LIDAR = self.form_widget.config.PARM_LIDAR
            calib_result = {}
            for pc in PARM_LIDAR['CheckedSensorList']:
                if self.form_widget.zrollpitch_tab.calib_result.get(pc) is None:
                    calib_result[pc] = [0.0, 0.0, 0.0]
                else:
                    calib_result[pc] = self.form_widget.zrollpitch_tab.calib_result[pc]

            self.form_widget.ResetResultsLabel(CONST_UNEDITABLE_LABEL2, PARM_LIDAR,
                                               self.form_widget.zrollpitch_tab.scroll_box.layout,
                                               self.form_widget.zrollpitch_tab.result_labels,
                                               calib_result)

            self.form_widget.ResetResultsLabel(CONST_ZRP_LABEL, PARM_LIDAR,
                                               self.form_widget.datavalidation_tab.zrp_scroll_box.layout,
                                               self.form_widget.datavalidation_tab.zrp_result_labels,
                                               calib_result)

            self.form_widget.ResetResultsLabel(CONST_ZRP_LABEL, PARM_LIDAR,
                                               self.form_widget.handeye_tab.zrp_scroll_box.layout,
                                               self.form_widget.handeye_tab.zrp_result_labels,
                                               calib_result)

            self.form_widget.ResetResultsLabel(CONST_ZRP_LABEL, PARM_LIDAR,
                                               self.form_widget.unsupervised_tab.zrp_scroll_box.layout,
                                               self.form_widget.unsupervised_tab.zrp_result_labels,
                                               calib_result)

    def TextChanged(self, text):
        if self.id == CONST_ZRP_SELECT_LIDAR_TO_CALIB:    # instance name is 'select_lidar_to_calib_layout'
            if text == '':
                return

            # Set zrollpitch_tab sensor index
            words = text.split(' ')
            self.form_widget.zrollpitch_tab.idxSensor = int(words[-1])

            # Reset ZRP tab result label
            PARM_LIDAR = self.form_widget.config.PARM_LIDAR
            calib_result = {}
            for pc in PARM_LIDAR['CheckedSensorList']:
                if self.form_widget.zrollpitch_tab.calib_result.get(pc) is None:
                    calib_result[pc] = [0.0, 0.0, 0.0]
                else:
                    calib_result[pc] = self.form_widget.zrollpitch_tab.calib_result[pc]

            self.form_widget.ResetResultsLabel(CONST_UNEDITABLE_LABEL2, PARM_LIDAR,
                                               self.form_widget.zrollpitch_tab.scroll_box.layout,
                                               self.form_widget.zrollpitch_tab.result_labels,
                                               calib_result)

            self.form_widget.ResetResultsLabel(CONST_ZRP_LABEL, PARM_LIDAR,
                                               self.form_widget.datavalidation_tab.zrp_scroll_box.layout,
                                               self.form_widget.datavalidation_tab.zrp_result_labels,
                                               calib_result)

            self.form_widget.ResetResultsLabel(CONST_ZRP_LABEL, PARM_LIDAR,
                                               self.form_widget.handeye_tab.zrp_scroll_box.layout,
                                               self.form_widget.handeye_tab.zrp_result_labels,
                                               calib_result)

            self.form_widget.ResetResultsLabel(CONST_ZRP_LABEL, PARM_LIDAR,
                                               self.form_widget.unsupervised_tab.zrp_scroll_box.layout,
                                               self.form_widget.unsupervised_tab.zrp_result_labels,
                                               calib_result)

            self.form_widget.ResetResultsLabel(CONST_ZRP_LABEL, PARM_LIDAR,
                                               self.form_widget.evaluation_tab.zrp_scroll_box.layout,
                                               self.form_widget.evaluation_tab.zrp_result_labels,
                                               calib_result)

            # reset roi configuration
            is_default = False
            if self.form_widget.config.PARM_ROI_DICT.get(int(words[-1])) is None:
                keys = list(self.form_widget.config.PARM_ROI_DICT.keys())
                if len(keys) == 0:
                    is_default = True
                else:
                    idx = keys[0]
            else:
                idx = int(words[-1])

            if is_default:
                self.form_widget.zrollpitch_tab.maximum_x_layout.double_spin_box.setValue(self.form_widget.config.PARM_ZRP['MaxDistanceX_m'])
                self.form_widget.zrollpitch_tab.minimum_x_layout.double_spin_box.setValue(self.form_widget.config.PARM_ZRP['MinDistanceX_m'])
                self.form_widget.zrollpitch_tab.maximum_y_layout.double_spin_box.setValue(self.form_widget.config.PARM_ZRP['MaxDistanceY_m'])
                self.form_widget.zrollpitch_tab.minimum_y_layout.double_spin_box.setValue(self.form_widget.config.PARM_ZRP['MinDistanceY_m'])
                self.form_widget.zrollpitch_tab.maximum_z_layout.double_spin_box.setValue(self.form_widget.config.PARM_ZRP['MaxDistanceZ_m'])
                self.form_widget.zrollpitch_tab.minimum_z_layout.double_spin_box.setValue(self.form_widget.config.PARM_ZRP['MinDistanceZ_m'])
            else:
                self.form_widget.zrollpitch_tab.maximum_x_layout.double_spin_box.setValue(self.form_widget.config.PARM_ROI_DICT[idx]['MaxDistanceX_m'])
                self.form_widget.zrollpitch_tab.minimum_x_layout.double_spin_box.setValue(self.form_widget.config.PARM_ROI_DICT[idx]['MinDistanceX_m'])
                self.form_widget.zrollpitch_tab.maximum_y_layout.double_spin_box.setValue(self.form_widget.config.PARM_ROI_DICT[idx]['MaxDistanceY_m'])
                self.form_widget.zrollpitch_tab.minimum_y_layout.double_spin_box.setValue(self.form_widget.config.PARM_ROI_DICT[idx]['MinDistanceY_m'])
                self.form_widget.zrollpitch_tab.maximum_z_layout.double_spin_box.setValue(self.form_widget.config.PARM_ROI_DICT[idx]['MaxDistanceZ_m'])
                self.form_widget.zrollpitch_tab.minimum_z_layout.double_spin_box.setValue(self.form_widget.config.PARM_ROI_DICT[idx]['MinDistanceZ_m'])

        elif self.id == CONST_UNSUPERVISED_SELECT_LIDAR_TO_CALIB:  # instance name is 'select_lidar_combobox_layout'
            if text == '':
                return

            words = text.split(' ')
            if self.form_widget.unsupervised_tab.is_single:
                self.form_widget.config.PARM_LIDAR['SingleSensor'] = int(words[-1])
            else:
                self.form_widget.config.PARM_LIDAR['PrincipalSensor'] = int(words[-1])

class SpinBoxLabelLayout(QVBoxLayout):
    def __init__(self, instance_id, label_str, form_widget):
        super().__init__()
        self.id = instance_id
        self.label_str = label_str
        self.form_widget = form_widget

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
        status = self.form_widget.unsupervised_tab.select_lidar_num_layout.button_group.checkedId()
        if status == CONST_SINGLE_LIDAR:
            PARM_OPTI = self.form_widget.config.PARM_SO
        elif status == CONST_MULTI_LIDAR:
            PARM_OPTI = self.form_widget.config.PARM_MO

        if self.id == CONST_CONFIG_LIDAR_NUM:
            ## Check PARM_LIDAR is empty
            if self.form_widget.config.PARM_LIDAR.get('SensorList') == None:
                return False
            if len(self.form_widget.config.PARM_LIDAR['SensorList']) <= 0:
                return False

            self.form_widget.datavalidation.complete_calibration = False
            self.form_widget.handeye.complete_calibration = False
            self.form_widget.unsupervised.complete_calibration = False
            self.form_widget.config_tab.is_lidar_num_changed = True

            is_minus = False
            last_lidar_num = self.form_widget.config.PARM_LIDAR['SensorList'][-1]
            spin_box_value = self.spin_box.value()
            PARM_LIDAR_num = len(self.form_widget.config.PARM_LIDAR['SensorList'])
            add_lidar_num = spin_box_value - PARM_LIDAR_num

            ## Determine adding or delete lidar list
            if add_lidar_num < 0:
                is_minus = True
                add_lidar_num = add_lidar_num * -1

            if is_minus:
                for i in range(add_lidar_num):
                    if len(self.form_widget.config.PARM_LIDAR['SensorList']) <= 1:
                        self.spin_box.setValue(1)
                        return False

                    sensor_index = self.form_widget.config.PARM_LIDAR['SensorList'][-1]
                    if sensor_index in self.form_widget.config.PARM_LIDAR['CheckedSensorList']:
                        list_index = self.form_widget.config.PARM_LIDAR['CheckedSensorList'].index(sensor_index)
                        del self.form_widget.config.PARM_LIDAR['CheckedSensorList'][list_index]
                    if sensor_index in self.form_widget.evaluation_tab.eval_lidar['CheckedSensorList']:
                        list_index = self.form_widget.evaluation_tab.eval_lidar['CheckedSensorList'].index(sensor_index)
                        del self.form_widget.evaluation_tab.eval_lidar['CheckedSensorList'][list_index]
                    del self.form_widget.config.PARM_LIDAR['SensorList'][-1]
            else: # is plus
                for i in range(add_lidar_num):
                    self.form_widget.config.PARM_LIDAR['SensorList'].append(last_lidar_num + 1)
                    self.form_widget.config.PARM_LIDAR['CheckedSensorList'].append(last_lidar_num + 1)
                    self.form_widget.evaluation_tab.eval_lidar['CheckedSensorList'].append(last_lidar_num + 1)
                    last_lidar_num = last_lidar_num + 1

            for idxSensor in self.form_widget.config.PARM_LIDAR['SensorList']:
                if self.form_widget.config.CalibrationParam.get(idxSensor):
                    continue
                calib = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.form_widget.config.CalibrationParam[idxSensor] = calib

            for idxSensor in self.form_widget.config.PARM_LIDAR['SensorList']:
                if self.form_widget.config.PARM_ROI_DICT.get(idxSensor):
                    continue
                self.form_widget.config.PARM_ROI_DICT[idxSensor] = {}
                self.form_widget.config.PARM_ROI_DICT[idxSensor]['MaxDistanceX_m'] = self.form_widget.config.PARM_ZRP['MaxDistanceX_m']
                self.form_widget.config.PARM_ROI_DICT[idxSensor]['MinDistanceX_m'] = self.form_widget.config.PARM_ZRP['MinDistanceX_m']
                self.form_widget.config.PARM_ROI_DICT[idxSensor]['MaxDistanceY_m'] = self.form_widget.config.PARM_ZRP['MaxDistanceY_m']
                self.form_widget.config.PARM_ROI_DICT[idxSensor]['MinDistanceY_m'] = self.form_widget.config.PARM_ZRP['MinDistanceY_m']
                self.form_widget.config.PARM_ROI_DICT[idxSensor]['MaxDistanceZ_m'] = self.form_widget.config.PARM_ZRP['MaxDistanceZ_m']
                self.form_widget.config.PARM_ROI_DICT[idxSensor]['MinDistanceZ_m'] = self.form_widget.config.PARM_ZRP['MinDistanceZ_m']

            ## Add widget item of lidar list in configuration tab and unsupervised tab
            self.form_widget.config_tab.select_using_sensor_list_layout.AddWidgetItem(self.form_widget.config.PARM_LIDAR['SensorList'], self.form_widget.config.PARM_LIDAR['CheckedSensorList'])
            self.form_widget.zrollpitch_tab.select_lidar_to_calib_layout.AddWidgetItem(self.form_widget.config.PARM_LIDAR['CheckedSensorList'])
            self.form_widget.unsupervised_tab.select_lidar_combobox_layout.AddWidgetItem(self.form_widget.config.PARM_LIDAR['CheckedSensorList'])

            ## Add Reset result label in rph tab, handeye tab, unsupervised tab and evaulation tab
            self.form_widget.ResetResultsLabels(self.form_widget.config.PARM_LIDAR)


        elif self.id == CONST_IMPORT_DATA_SAMPLING_INTERVER:
            self.form_widget.config.PARM_IM['SamplingInterval'] = self.spin_box.value()
        elif self.id == CONST_VALIDATION_MAXIMUM_ITERATION:
            self.form_widget.config.PARM_DV['MaximumIteration'] = self.spin_box.value()
        elif self.id == CONST_HANDEYE_MAXIMUM_ITERATION:
            self.form_widget.config.PARM_HE['MaximumIteration'] = self.spin_box.value()
        elif self.id == CONST_UNSUPERVISED_NUM_POINTS_PLANE_MODELING:
            PARM_OPTI['NumPointsPlaneModeling'] = self.spin_box.value()
        elif self.id == CONST_EVAL_SAMPLING_INTERVAL:
            self.form_widget.config.PARM_EV['SamplingInterval'] = self.spin_box.value()

        self.form_widget.config.IsParmChanged()

class DoubleSpinBoxLabelLayout(QHBoxLayout):
    def __init__(self, intance_id, label_str, form_widget, decimals=None):
        super().__init__()
        self.id = intance_id
        self.label_str = label_str
        self.form_widget = form_widget
        self.decimals = decimals

        self.InitUi()

    def InitUi(self):
        self.label = QLabel(self.label_str)
        self.addWidget(self.label)

        self.double_spin_box = QDoubleSpinBox()
        self.double_spin_box.setSingleStep(0.01)
        self.double_spin_box.setMaximum(1000.0)
        if self.decimals is not None:
            self.double_spin_box.setDecimals(self.decimals)
        self.double_spin_box.setMinimum(-1000.0)
        self.double_spin_box.valueChanged.connect(self.DoubleSpinBoxChanged)
        self.addWidget(self.double_spin_box)

    def DoubleSpinBoxChanged(self):
        status = self.form_widget.unsupervised_tab.select_lidar_num_layout.button_group.checkedId()
        if status == CONST_SINGLE_LIDAR:
            PARM_OPTI = self.form_widget.config.PARM_SO
        elif status == CONST_MULTI_LIDAR:
            PARM_OPTI = self.form_widget.config.PARM_MO

        if self.id == CONST_CONFIG_MINIMUM_THRESHOLD_DISTANCE: # Configuration tab Minimum Threshold Distance [m]
            self.form_widget.config.PARM_PC['MinThresholdDist_m'] = self.double_spin_box.value()
            self.form_widget.config_tab.is_config_parm_changed = True
        elif self.id == CONST_CONFIG_MAXIMUM_THRESHOLD_DISTANCE:  # Configuration tab Maximum Threshold Distance [m]
            self.form_widget.config.PARM_PC['MaxThresholdDist_m'] = self.double_spin_box.value()
            self.form_widget.config_tab.is_config_parm_changed = True
        elif self.id == CONST_CONFIG_MINIMUM_THRESHOLD_X:  # Configuration tab Minimum Threshold X [m]
            self.form_widget.config.PARM_PC['MinThresholdX_m'] = self.double_spin_box.value()
            self.form_widget.config_tab.is_config_parm_changed = True
        elif self.id == CONST_CONFIG_MAXIMUM_THRESHOLD_X:  # Configuration tab Maximum Threshold X [m]
            self.form_widget.config.PARM_PC['MaxThresholdX_m'] = self.double_spin_box.value()
            self.form_widget.config_tab.is_config_parm_changed = True
        elif self.id == CONST_CONFIG_MINIMUM_THRESHOLD_Y:  # Configuration tab Minimum Threshold Y [m]
            self.form_widget.config.PARM_PC['MinThresholdY_m'] = self.double_spin_box.value()
            self.form_widget.config_tab.is_config_parm_changed = True
        elif self.id == CONST_CONFIG_MAXIMUM_THRESHOLD_Y:  # Configuration tab Maximum Threshold Y [m]
            self.form_widget.config.PARM_PC['MaxThresholdY_m'] = self.double_spin_box.value()
            self.form_widget.config_tab.is_config_parm_changed = True
        elif self.id == CONST_CONFIG_MINIMUM_THRESHOLD_Z:  # Configuration tab Minimum Threshold Z [m]
            self.form_widget.config.PARM_PC['MinThresholdZ_m'] = self.double_spin_box.value()
            self.form_widget.config_tab.is_config_parm_changed = True
        elif self.id == CONST_CONFIG_MAXIMUM_THRESHOLD_Z:  # Configuration tab Minimum Threshold Distance [m]
            self.form_widget.config.PARM_PC['MaxThresholdZ_m'] = self.double_spin_box.value()
            self.form_widget.config_tab.is_config_parm_changed = True

        elif self.id == CONST_IMPORT_VEHICLE_MINIMUM_SPEED :  # Import tab Vehicle Minimum Speed [km/h]
            self.form_widget.config.PARM_IM['VehicleSpeedThreshold'] = self.double_spin_box.value()

        elif self.id == CONST_ZRP_MAXIMUM_X_DISTANCE:  # Z, Roll, Pitch: Calibration tab Maximum X Distance [m]
            self.form_widget.config.PARM_ROI_DICT[self.form_widget.zrollpitch_tab.idxSensor]['MaxDistanceX_m'] = self.double_spin_box.value()
        elif self.id == CONST_ZRP_MINIMUM_X_DISTANCE:  # Z, Roll, Pitch: Calibration tab Minimum X Distance [m]
            self.form_widget.config.PARM_ROI_DICT[self.form_widget.zrollpitch_tab.idxSensor]['MinDistanceX_m'] = self.double_spin_box.value()
        elif self.id == CONST_ZRP_MAXIMUM_Y_DISTANCE:  # Z, Roll, Pitch: Calibration tab Maximum Y Distance [m]
            self.form_widget.config.PARM_ROI_DICT[self.form_widget.zrollpitch_tab.idxSensor]['MaxDistanceY_m'] = self.double_spin_box.value()
        elif self.id == CONST_ZRP_MINIMUM_Y_DISTANCE:  # Z, Roll, Pitch: Calibration tab Minimum Y Distance [m]
            self.form_widget.config.PARM_ROI_DICT[self.form_widget.zrollpitch_tab.idxSensor]['MinDistanceY_m'] = self.double_spin_box.value()
        elif self.id == CONST_ZRP_MAXIMUM_Z_DISTANCE:  # Z, Roll, Pitch: Calibration tab Maximum Z Distance [m]
            self.form_widget.config.PARM_ROI_DICT[self.form_widget.zrollpitch_tab.idxSensor]['MaxDistanceZ_m'] = self.double_spin_box.value()
        elif self.id == CONST_ZRP_MINIMUM_Z_DISTANCE:  # Z, Roll, Pitch: Calibration tab Minimum Z Distance [m]
            self.form_widget.config.PARM_ROI_DICT[self.form_widget.zrollpitch_tab.idxSensor]['MinDistanceZ_m'] = self.double_spin_box.value()

        elif self.id == CONST_DATAVALIDATION_TOLERANCE:  # Datavalidation tab Tolerance
            self.form_widget.config.PARM_DV['Tolerance'] = self.double_spin_box.value()
        elif self.id == CONST_DATAVALIDATION_OUTLIER_DISTANCE:  # Datavalidation tab Outlier Distance [m]
            self.form_widget.config.PARM_DV['OutlierDistance_m'] = self.double_spin_box.value()
        elif self.id == CONST_DATAVALIDATION_HEADING_THRESHOLD:  # Datavalidation tab Heading Threshold (filter)
            self.form_widget.config.PARM_DV['FilterHeadingThreshold'] = self.double_spin_box.value()
        elif self.id == CONST_DATAVALIDATION_DISTANCE_THRESHOLD:  # Datavalidation tab Distance Threshold (filter)
            self.form_widget.config.PARM_DV['FilterDistanceThreshold'] = self.double_spin_box.value()
        elif self.id == CONST_DATAVALIDATION_MAXIMUM_Z_DISTANCE:  # Datavalidation tab Minimum Threshold Z (filter)
            self.form_widget.config.PARM_DV['MaxThresholdZ_m'] = self.double_spin_box.value()
        elif self.id == CONST_DATAVALIDATION_MINIMUM_Z_DISTANCE:  # Datavalidation tab Minimum Threshold Z (filter)
            self.form_widget.config.PARM_DV['MinThresholdZ_m'] = self.double_spin_box.value()

        elif self.id == CONST_HANDEYE_TOLERANCE:  # Handeye tab Tolerance
            self.form_widget.config.PARM_HE['Tolerance'] = self.double_spin_box.value()
        elif self.id == CONST_HANDEYE_OUTLIER_DISTANCE:  # Handeye tab Outlier Distance [m]
            self.form_widget.config.PARM_HE['OutlierDistance_m'] = self.double_spin_box.value()
        elif self.id == CONST_HANDEYE_HEADING_THRESHOLD:  # Handeye tab Heading Threshold (filter)
            self.form_widget.config.PARM_HE['FilterHeadingThreshold'] = self.double_spin_box.value()
        elif self.id == CONST_HANDEYE_DISTANCE_THRESHOLD:  # Handeye tab Distance Threshold (filter)
            self.form_widget.config.PARM_HE['FilterDistanceThreshold'] = self.double_spin_box.value()
        elif self.id == CONST_HANDEYE_MAXIMUM_Z_DISTANCE:  # Handeye tab Minimum Threshold Z (filter)
            self.form_widget.config.PARM_HE['MaxThresholdZ_m'] = self.double_spin_box.value()
        elif self.id == CONST_HANDEYE_MINIMUM_Z_DISTANCE:  # Handeye tab Minimum Threshold Z (filter)
            self.form_widget.config.PARM_HE['MinThresholdZ_m'] = self.double_spin_box.value()

        elif self.id == CONST_OPTI_POINT_SAMPLING_RATIO:  # unsupervised tab Point Sampling Ratio
            PARM_OPTI['PointSamplingRatio'] = self.double_spin_box.value()
        elif self.id == CONST_OPTI_OUTLIER_DISTANCE:  # unsupervised tab Outlier Distance [m]
            PARM_OPTI['OutlierDistance_m'] = self.double_spin_box.value()
        elif self.id == CONST_OPTI_MAXIMUM_Z_DISTANCE:  # unsupervised tab Minimum Threshold Z (filter)
            PARM_OPTI['MaxThresholdZ_m'] = self.double_spin_box.value()
        elif self.id == CONST_OPTI_MINIMUM_Z_DISTANCE:  # unsupervised tab Minimum Threshold Z (filter)
            PARM_OPTI['MinThresholdZ_m'] = self.double_spin_box.value()

        elif self.id == CONST_EVAL_VEHICLE_MINIMUM_SPEED:  # Evaluation tab Eval Vehicle Minimum Speed [km/h]
            self.form_widget.config.PARM_EV['VehicleSpeedThreshold'] = self.double_spin_box.value()
        elif self.id == CONST_EVAL_DISTANCE_INTERVAL:  # Evaluation tab Eval Distance Interval [m]
            self.form_widget.config.PARM_EV['DistanceInterval'] = self.double_spin_box.value()
        elif self.id == CONST_EVAL_MAXIMUM_Z_DISTANCE:  # Evaluation tab Minimum Threshold Z (filter)
            self.form_widget.config.PARM_EV['MaxThresholdZ_m'] = self.double_spin_box.value()
        elif self.id == CONST_EVAL_MINIMUM_Z_DISTANCE:  # Evaluation tab Minimum Threshold Z (filter)
            self.form_widget.config.PARM_EV['MinThresholdZ_m'] = self.double_spin_box.value()

        elif self.id == CONST_CONFIG_REF_X:  # Configuration tab Reference Origin X [m]
            vtk_lidar_calib.reference_origin_[0] = self.double_spin_box.value()
        elif self.id == CONST_CONFIG_REF_Y:  # Configuration tab Reference Origin Y [m]
            vtk_lidar_calib.reference_origin_[1] = self.double_spin_box.value()
        elif self.id == CONST_CONFIG_REF_Z:  # Configuration tab Reference Origin Z [m]
            vtk_lidar_calib.reference_origin_[2] = self.double_spin_box.value()
        elif self.id == CONST_CONFIG_REF_ROLL:  # Configuration tab Reference Origin Roll [deg]
            vtk_lidar_calib.reference_origin_[3] = self.double_spin_box.value()
        elif self.id == CONST_CONFIG_REF_PITCH:  # Configuration tab Reference Origin Pitch [deg]
            vtk_lidar_calib.reference_origin_[4] = self.double_spin_box.value()
        elif self.id == CONST_CONFIG_REF_YAW:  # Configuration tab Reference Origin Yaw [deg]
            vtk_lidar_calib.reference_origin_[5] = self.double_spin_box.value()

        self.form_widget.config.IsParmChanged()

class SlideLabelLayouts(QVBoxLayout):
    def __init__(self, instance_id, form_widget, label_str=None):
        super().__init__()
        self.id = instance_id

        self.form_widget = form_widget
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
                self.form_widget.ErrorPopUp('Start time cannot be higher than end time')
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
                self.form_widget.ErrorPopUp('End time cannot be lower than start time')
            else:
                self.end_time_layout.slider.setValue(self.end_time_layout.double_spin_box.value())

            self.end_time_layout.prev_slider_value = self.end_time_layout.slider.value()
            self.end_time_layout.prev_double_spin_box_value = self.end_time_layout.double_spin_box.value()
            self.end_time = self.end_time_layout.double_spin_box.value()

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

class RadioLabelLayout(QHBoxLayout):
    def __init__(self, instance_id, label_str, buttons, form_widget):
        super().__init__()
        self.id = instance_id
        self.buttons = buttons
        self.label_str = label_str
        self.form_widget = form_widget

        self.using_gnss_motion = False
        if self.id == CONST_DATAVALIDATION_USED_DATA or self.id == CONST_HANDEYE_USED_DATA or self.id == CONST_UNSUPERVISED_USED_DATA or self.id == CONST_EVAL_SELECT_LIDAR:
            self.prev_checkID = 1
        elif self.id == CONST_UNSUPERVISED_SELECT_LIDAR:
            self.prev_checkID = CONST_SINGLE_LIDAR

        self.InitUi()

    def InitUi(self):
        label = QLabel(self.label_str)
        self.addWidget(label)

        # button for Handeye calibration
        self.button_group = QButtonGroup()
        for key in self.buttons:
            rbn = QRadioButton(self.buttons[key])
            if key == 0:
                rbn.setChecked(True)
            else:
                rbn.setChecked(False)
            rbn.clicked.connect(self.RadioButton)
            self.addWidget(rbn)
            self.button_group.addButton(rbn, key+1)

    def RadioButton(self):
        '''
        only one button can selected
        '''
        status = self.button_group.checkedId()
        if self.id == CONST_DATAVALIDATION_USED_DATA or self.id == CONST_HANDEYE_USED_DATA or self.id == CONST_UNSUPERVISED_USED_DATA or self.id == CONST_EVAL_SELECT_LIDAR:
            if status == 1:  # GNSS Data
                if self.form_widget.importing.has_gnss_file == False:
                    self.button_group.button(self.prev_checkID).setChecked(True)
                    self.form_widget.ErrorPopUp('Please import Gnss.csv')
                    return False
                self.using_gnss_motion = False
            elif status == 2:  # Motion Data
                if self.form_widget.importing.has_motion_file == False:
                    self.button_group.button(self.prev_checkID).setChecked(True)
                    self.form_widget.ErrorPopUp('Please import Motion.csv')
                    return False
                self.using_gnss_motion = True

        if self.id == CONST_UNSUPERVISED_SELECT_LIDAR:
            if status == CONST_SINGLE_LIDAR:
                self.form_widget.unsupervised_tab.select_lidar_combobox_layout.label.setText("Select Single Lidar To Calibration")
                self.form_widget.unsupervised_tab.is_single = True

                PARM_SO = self.form_widget.config.PARM_SO
                self.form_widget.unsupervised_tab.point_sampling_ratio_layout.double_spin_box.setValue(PARM_SO['PointSamplingRatio'])
                self.form_widget.unsupervised_tab.num_points_plane_modeling_layout.spin_box.setValue(PARM_SO['NumPointsPlaneModeling'])
                self.form_widget.unsupervised_tab.outlier_distance_layout.double_spin_box.setValue(PARM_SO['OutlierDistance_m'])
                self.form_widget.unsupervised_tab.maximum_threshold_z_layout.double_spin_box.setValue(PARM_SO['MaxThresholdZ_m'])
                self.form_widget.unsupervised_tab.minimum_threshold_z_layout.double_spin_box.setValue(PARM_SO['MinThresholdZ_m'])

                for i, idxSensor in enumerate(self.form_widget.config.PARM_LIDAR['CheckedSensorList']):
                    if idxSensor == self.form_widget.config.PARM_LIDAR['SingleSensor']:
                        self.form_widget.unsupervised_tab.select_lidar_combobox_layout.cb.setCurrentIndex(i)
                        try:
                            self.form_widget.evaluation_tab.xyyaw_result_labels[idxSensor].button_group.button(3).setEnabled(True)
                        except:
                            pass
                    else:
                        self.form_widget.evaluation_tab.xyyaw_result_labels[idxSensor].button_group.button(3).setEnabled(False)

            elif status == CONST_MULTI_LIDAR:
                self.form_widget.unsupervised_tab.select_lidar_combobox_layout.label.setText("Select Principal Lidar To Calibration")
                self.form_widget.unsupervised_tab.is_single = False

                PARM_MO = self.form_widget.config.PARM_MO
                self.form_widget.unsupervised_tab.point_sampling_ratio_layout.double_spin_box.setValue(PARM_MO['PointSamplingRatio'])
                self.form_widget.unsupervised_tab.num_points_plane_modeling_layout.spin_box.setValue(PARM_MO['NumPointsPlaneModeling'])
                self.form_widget.unsupervised_tab.outlier_distance_layout.double_spin_box.setValue(PARM_MO['OutlierDistance_m'])
                self.form_widget.unsupervised_tab.maximum_threshold_z_layout.double_spin_box.setValue(PARM_MO['MaxThresholdZ_m'])
                self.form_widget.unsupervised_tab.minimum_threshold_z_layout.double_spin_box.setValue(PARM_MO['MinThresholdZ_m'])

                for i, idxSensor in enumerate(self.form_widget.config.PARM_LIDAR['CheckedSensorList']):
                    if idxSensor == self.form_widget.config.PARM_LIDAR['PrincipalSensor']:
                        self.form_widget.unsupervised_tab.select_lidar_combobox_layout.cb.setCurrentIndex(i)
                        break

        self.prev_checkID = self.button_group.checkedId()

class EditLabelWithButtonLayout(QHBoxLayout):
    def __init__(self, instance_id, id_to_strs1, id_to_strs2, form_widget):
        super().__init__()
        self.id = instance_id
        self.id_to_strs1 = id_to_strs1
        self.id_to_strs2 = id_to_strs2
        self.form_widget = form_widget
        self.vehicle_info = [0.0, 0.0, 0.0, '', '']

        self.InitUi()

    def InitUi(self):
        self.double_spin_list = []
        for i, key in enumerate(self.id_to_strs1):
            self.double_spin_list.append(DoubleSpinBoxLabelLayout(key, self.id_to_strs1[key], self.form_widget))
            self.addLayout(self.double_spin_list[-1])
            self.addStretch(round(0.5))

        self.cb_list = []
        for i, key in enumerate(self.id_to_strs2):
            self.cb_list.append(ComboBoxLabelLayout(key, self.id_to_strs2[key], self.form_widget))
            self.addLayout(self.cb_list[-1])
            self.addStretch(round(0.5))

        self.btn = QPushButton('Import')
        self.btn.clicked.connect(self.Import)
        self.addWidget(self.btn)

    def Import(self):
        if self.id == CONST_CONFIG_VEHICLE_INFO:
            if self.cb_list[0].cb.currentIndex() == self.cb_list[1].cb.currentIndex():
                self.form_widget.ErrorPopUp('Please select an appropriate axis')
                return

            file_name_stl = self.form_widget.config_tab.cb.currentText()
            words = file_name_stl.split('.')
            file_name = words[0]
            # if self.form_widget.config_tab.added_stl_dict.get(file_name) is not None:
            self.vehicle_info[0] = self.double_spin_list[0].double_spin_box.value()
            self.vehicle_info[1] = self.double_spin_list[1].double_spin_box.value()
            self.vehicle_info[2] = self.double_spin_list[2].double_spin_box.value()
            self.vehicle_info[3] = self.cb_list[0].cb.currentText()
            self.vehicle_info[4] = self.cb_list[1].cb.currentText()
            self.form_widget.config.VEHICLE_INFO[file_name] = copy.deepcopy(self.vehicle_info)
            self.form_widget.config_tab.VTKInit(file_name_stl)

    def Clear(self, target=None):
        if target is not None:
            last_idx = 0
            for i, double_spin in enumerate(self.double_spin_list):
                double_spin.double_spin_box.setValue(target[i])
                last_idx = i
            last_idx += 1

            for i, cb in enumerate(self.cb_list):
                idx = cb.cb.findText(target[last_idx+i], Qt.MatchFixedString)
                cb.cb.setCurrentIndex(idx)
        else:
            for double_spin in self.double_spin_list:
                double_spin.double_spin_box.setValue(0.0)
            self.cb_list[0].cb.setCurrentIndex(0)
            self.cb_list[1].cb.setCurrentIndex(1)

    def BtnEnable(self, enable):
        for double_spin in self.double_spin_list:
            double_spin.double_spin_box.setEnabled(enable)
        for cb in self.cb_list:
            cb.cb.setEnabled(enable)

        self.btn.setEnabled(enable)

class LabelWithDoubleSpinBoxsLayout(QHBoxLayout):
    def __init__(self, spinbox_ids, label_str, form_widget):
        super().__init__()
        self.spinbox_ids = spinbox_ids
        self.label_str = label_str
        self.form_widget = form_widget

        self.InitUi()

    def InitUi(self):
        self.lb = QLabel(self.label_str)
        self.addWidget(self.lb)

        self.addStretch()

        self.spinbox_dict = {}
        for spin_box_id in self.spinbox_ids:
            uniq_double_spin_box = QUniqDoubleSpinBox(spin_box_id, self.form_widget)
            uniq_double_spin_box.setSingleStep(0.01)
            uniq_double_spin_box.setMaximum(1000.0)
            uniq_double_spin_box.setMinimum(-1000.0)
            self.addWidget(uniq_double_spin_box)
            self.spinbox_dict[spin_box_id] = uniq_double_spin_box

    def SetEnable(self, enable=True):
        self.lb.setEnabled(enable)
        for key in self.spinbox_dict:
            self.spinbox_dict[key].setEnabled(enable)

class LabelWithLabelDoubleSpinBoxsLayout(QHBoxLayout):
    def __init__(self, id_to_strs, label_str, form_widget):
        super().__init__()
        self.id_to_strs = id_to_strs
        self.label_str = label_str
        self.form_widget = form_widget

        self.InitUi()

    def InitUi(self):
        self.lb = QLabel(self.label_str)
        self.addWidget(self.lb)

        self.addStretch(round(0.5))

        self.double_spin_list = []
        for i, key in enumerate(self.id_to_strs):
            self.double_spin_list.append(DoubleSpinBoxLabelLayout(key, self.id_to_strs[key], self.form_widget))
            self.addLayout(self.double_spin_list[-1])
            self.addStretch(round(0.5))

class LabelWithSliderLayout(QVBoxLayout):
    def __init__(self, instance_id, label_str, form_widget):
        super().__init__()
        self.id = instance_id
        self.label_str = label_str
        self.form_widget = form_widget

        self.InitUi()

    def InitUi(self):
        hbox = QHBoxLayout()
        self.slider = QSlider(Qt.Vertical)
        self.slider.valueChanged.connect(self.SliderValueChanged)
        hbox.addWidget(self.slider, 50)
        self.addLayout(hbox)

        label = QLabel(self.label_str)
        self.addWidget(label)

    def SliderValueChanged(self):
        if self.id == CONST_CONFIG_TRANSPARENT or self.id == CONST_XYYAW_TRANSPARENT or self.id == CONST_EVAL_TRANSPARENT:
            current_tab = self.form_widget.GetCurrentTab()
            if current_tab == -1:
                return

            current_tab.vehicle_actor.GetProperty().SetOpacity(self.ValueMapping(self.slider.value()))
            current_tab.renWin.Render()
        elif self.id == CONST_VIEW_TRANSPARENT:
            current_tab = self.form_widget.GetCurrentTab()
            if current_tab == -1:
                return

            try:
                current_tab.dialog.vehicle_actor.GetProperty().SetOpacity(self.ValueMapping(self.slider.value()))
                current_tab.dialog.renWin.Render()
            except:
                pass

    def Clear(self):
        if self.id == CONST_CONFIG_TRANSPARENT or self.id == CONST_XYYAW_TRANSPARENT or self.id == CONST_EVAL_TRANSPARENT:
            current_tab = self.form_widget.GetCurrentTab()

            vehicle_actor_opacity = current_tab.vehicle_actor.GetProperty().GetOpacity()
            current_tab.transparent_layout.slider.setValue(vehicle_actor_opacity * 100)

    def ValueMapping(self, val, reverse=False):
        OLD_MIN = 0; OLD_MAX = 99
        NEW_MIN = 0.05; NEW_MAX = 1

        if not reverse:
            old_range = OLD_MAX - OLD_MIN
            new_range = NEW_MAX - NEW_MIN
            new_value = (((val - OLD_MIN) * new_range) / old_range) + NEW_MIN
            new_value = NEW_MAX - new_value + NEW_MIN
        else:
            old_range = OLD_MAX - OLD_MIN
            new_range = NEW_MAX - NEW_MIN
            new_value = (((val - NEW_MIN) * old_range) / new_range) + OLD_MIN
            new_value = OLD_MAX - new_value + OLD_MIN

        return new_value

    def InitSlider(self, actor=None):
        if self.id == CONST_CONFIG_TRANSPARENT or self.id == CONST_XYYAW_TRANSPARENT or self.id == CONST_EVAL_TRANSPARENT:
            opacity = actor.GetProperty().GetOpacity()
            print('value mapping: {}'.format(self.ValueMapping(opacity, reverse=True)))
            self.slider.setValue(self.ValueMapping(opacity, reverse=True))

class CalibrationResultEditLabel(QVBoxLayout):
    def __init__(self, intance_id, idxSensor, calibration_param, form_widget):
        super().__init__()
        self.id = intance_id
        self.idxSensor = idxSensor
        self.calibration_param = calibration_param
        self.form_widget = form_widget

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
        status = self.form_widget.evaluation_tab.button_group.checkedId()
        if not self.id == status:
            return False

        self.calibration_param[self.idxSensor][3] = self.double_spin_box_x.value() * math.pi / 180.0
        self.calibration_param[self.idxSensor][4] = self.double_spin_box_y.value() * math.pi / 180.0
        self.calibration_param[self.idxSensor][2] = self.double_spin_box_yaw.value() * math.pi / 180.0

        self.form_widget.evaluation_tab.DisplayCalibrationGraph()

class CalibrationResultLabel(QVBoxLayout):
    def __init__(self, idxSensor, str1, str2, str3):
        super().__init__()
        self.idxSensor = idxSensor

        self.str1 = str1
        self.str2 = str2
        self.str3 = str3

        self.InitUi()

    def InitUi(self):
        self.label = QLabel('LiDAR {}'.format(self.idxSensor))
        self.addWidget(self.label)
        self.hbox = QHBoxLayout()
        self.label1 = QLabel(self.str1)
        self.hbox.addWidget(self.label1)
        self.label_edit1 = QLineEdit()
        self.label_edit1.setReadOnly(True)
        self.label_edit1.setText('0.0')
        self.label_edit1.setStyleSheet("background-color: #F0F0F0;")
        self.hbox.addWidget(self.label_edit1)

        self.label2 = QLabel(self.str2)
        self.hbox.addWidget(self.label2)
        self.label_edit2 = QLineEdit()
        self.label_edit2.setReadOnly(True)
        self.label_edit2.setText('0.0')
        self.label_edit2.setStyleSheet("background-color: #F0F0F0;")
        self.hbox.addWidget(self.label_edit2)

        self.label3 = QLabel(self.str3)
        self.hbox.addWidget(self.label3)
        self.label_edit3 = QLineEdit()
        self.label_edit3.setReadOnly(True)
        self.label_edit3.setText('0.0')
        self.label_edit3.setStyleSheet("background-color: #F0F0F0;")
        self.hbox.addWidget(self.label_edit3)

        self.addLayout(self.hbox)

class ValidationResultLabel(QVBoxLayout):
    def __init__(self, idxSensor):
        super().__init__()
        self.idxSensor = idxSensor

        self.InitUi()

    def InitUi(self):
    #    try:
        #self.label_heading_threshold = QLabel('Heading Threshold [deg]: {}'.format(self.form_widget.config.PARM_DV['FilterHeadingThreshold']))
        #self.addWidget(self.label_heading_threshold)
        #self.hbox = QHBoxLayout()
        #self.label_distance_threshold = QLabel('Distance Threshold [m]: {}'.format(self.form_widget.config.PARM_DV['FilterDistanceThreshold']))
        #self.addWidget(self.label_heading_threshold)
        #self.hbox = QHBoxLayout()


        self.label = QLabel('LiDAR {}'.format(self.idxSensor))
        self.addWidget(self.label)
        self.hbox = QHBoxLayout()

        self.label_translation_error = QLabel('Translation Error RMSE [m]')
        self.hbox.addWidget(self.label_translation_error)
        self.label_edit_translation_error = QLineEdit()
        self.label_edit_translation_error.setReadOnly(True)
        self.label_edit_translation_error.setText('0.0')
        self.label_edit_translation_error.setStyleSheet("background-color: #F0F0F0;")
        self.hbox.addWidget(self.label_edit_translation_error)

        self.label_rotation_error = QLabel('Rotation Error RMSE [deg]')
        self.hbox.addWidget(self.label_rotation_error)
        self.label_edit_rotation_error = QLineEdit()
        self.label_edit_rotation_error.setReadOnly(True)
        self.label_edit_rotation_error.setText('0.0')
        self.label_edit_rotation_error.setStyleSheet("background-color: #F0F0F0;")
        self.hbox.addWidget(self.label_edit_rotation_error)


        self.addLayout(self.hbox)

class CalibrationResultEditLabel2(QVBoxLayout):
    def __init__(self, idxSensor, calibration_param, form_widget):
        super().__init__()
        self.idxSensor = idxSensor
        self.calibration_param = copy.deepcopy(calibration_param)
        self.form_widget = form_widget

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

        self.form_widget.unsupervised.CalibrationParam[self.idxSensor] = copy.deepcopy(self.calibration_param[self.idxSensor])

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
    def __init__(self, str1, str2, form_widget):
        super().__init__()
        self.form_widget = form_widget

        self.str1 = str1
        self.str2 = str2

        self.initUI()

    def initUI(self):
        self.scroll_box1 = ScrollAreaV()
        self.scroll_box2 = ScrollAreaV()

        self.tabs = QTabWidget()
        self.tabs.addTab(self.scroll_box1, self.str1)
        self.tabs.addTab(self.scroll_box2, self.str2)
        self.addWidget(self.tabs)

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

class RadioWithEditLabel(QHBoxLayout):
    def __init__(self, instance_id, idxSensor, buttons, form_widget, editable=True, using_checkbox=False):
        super().__init__()
        self.id = instance_id
        if self.id == CONST_EVALUATION_RESULT_LABEL:
            self.prev_checkID = CONST_HANDEYE
        elif self.id == CONST_EVALUATION_ZRP_RESULT_LABEL:
            self.prev_checkID = CONST_AUTOMATIC

        self.idxSensor = idxSensor
        self.buttons = buttons
        self.form_widget = form_widget
        self.editable = editable
        self.using_checkbox = using_checkbox

        self.InitUi()

    def InitUi(self):
        if self.using_checkbox:
            self.checkbox = QCheckBox('LiDAR {}'.format(str(self.idxSensor)))
            self.checkbox.setChecked(True)
            self.checkbox.stateChanged.connect(self.CheckBox)
            self.addWidget(self.checkbox)
        else:
            label = QLabel('LiDAR {}'.format(str(self.idxSensor)))
            self.addWidget(label)

        self.button_group = QButtonGroup()
        for i, key in enumerate(self.buttons):
            rbn = QRadioButton()
            rbn.setChecked(self.buttons[key])
            rbn.clicked.connect(self.RadioButton)
            self.addWidget(rbn)
            self.button_group.addButton(rbn, i+1)

        # spinbox for x
        self.spinbox1 = QDoubleSpinBox()
        self.spinbox1.setReadOnly(self.editable)
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
        self.spinbox2.setReadOnly(self.editable)
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
        self.spinbox3.setReadOnly(self.editable)
        self.spinbox3.setSingleStep(0.01)
        self.spinbox3.setMaximum(1000.0)
        self.spinbox3.setMinimum(-1000.0)
        self.spinbox3.setDecimals(4)
        self.spinbox3.setStyleSheet("background-color: #F0F0F0;")
        self.spinbox3.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spinbox3.valueChanged.connect(self.DoubleSpinBoxChanged3)
        self.addWidget(self.spinbox3)

    ## Callback functions
    def CheckBox(self):
        if self.id == CONST_EVALUATION_RESULT_LABEL:
            if self.checkbox.checkState() == 0:
                if self.idxSensor in self.form_widget.evaluation_tab.eval_lidar['CheckedSensorList']:
                    self.form_widget.evaluation_tab.eval_lidar['CheckedSensorList'].remove(self.idxSensor)
            else:
                if not self.idxSensor in self.form_widget.evaluation_tab.eval_lidar['CheckedSensorList']:
                    self.form_widget.evaluation_tab.eval_lidar['CheckedSensorList'].append(self.idxSensor)
                    self.form_widget.evaluation_tab.eval_lidar['CheckedSensorList'].sort()

    def RadioButton(self):
        if self.form_widget.config_tab.is_lidar_num_changed == True:
            self.button_group.button(self.prev_checkID).setChecked(True)
            self.form_widget.ErrorPopUp('Please import after changing lidar number')
            return False
        elif self.form_widget.config_tab.is_config_parm_changed == True:
            self.button_group.button(self.prev_checkID).setChecked(True)
            self.form_widget.ErrorPopUp('Please import after changing PointCloud Configuration')
            return False

        if self.id == CONST_EVALUATION_RESULT_LABEL:
            status = self.button_group.checkedId()
            if status == CONST_HANDEYE:
                if not self.form_widget.handeye.complete_calibration:
                    self.button_group.button(self.prev_checkID).setChecked(True)
                    self.form_widget.ErrorPopUp('Please complete the HandEye calibration')
                    return False
            elif status == CONST_UNSUPERVISED:
                if not self.form_widget.unsupervised.complete_calibration:
                    self.button_group.button(self.prev_checkID).setChecked(True)
                    self.form_widget.ErrorPopUp('Please complete the unsupervised calibration')
                    return False
            elif status == CONST_CUSTOM:
                if self.form_widget.evaluation_tab.custom_calibration_param.get(self.idxSensor) == None:
                    self.form_widget.evaluation_tab.custom_calibration_param[self.idxSensor] = [0., 0., 0., 0., 0., 0.]

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
                self.spinbox1.setValue(self.form_widget.handeye.CalibrationParam[self.idxSensor][3])
                self.spinbox2.setValue(self.form_widget.handeye.CalibrationParam[self.idxSensor][4])
                self.spinbox3.setValue(self.form_widget.handeye.CalibrationParam[self.idxSensor][2] * 180.0 / math.pi)
            elif status == CONST_UNSUPERVISED:
                self.spinbox1.setValue(self.form_widget.unsupervised.CalibrationParam[self.idxSensor][3])
                self.spinbox2.setValue(self.form_widget.unsupervised.CalibrationParam[self.idxSensor][4])
                self.spinbox3.setValue(self.form_widget.unsupervised.CalibrationParam[self.idxSensor][2] * 180.0 / math.pi)
            elif status == CONST_CUSTOM:
                self.spinbox1.setValue(self.form_widget.evaluation_tab.custom_calibration_param[self.idxSensor][3])
                self.spinbox2.setValue(self.form_widget.evaluation_tab.custom_calibration_param[self.idxSensor][4])
                self.spinbox3.setValue(self.form_widget.evaluation_tab.custom_calibration_param[self.idxSensor][2] * 180.0 / math.pi)

        if self.id == CONST_EVALUATION_ZRP_RESULT_LABEL:
            status = self.button_group.checkedId()
            if status == CONST_AUTOMATIC:
                # Reset spinbox format
                self.spinbox1.setReadOnly(True)
                self.spinbox1.setStyleSheet("background-color: #F0F0F0;")
                self.spinbox1.setButtonSymbols(QAbstractSpinBox.NoButtons)
                self.spinbox2.setReadOnly(True)
                self.spinbox2.setStyleSheet("background-color: #F0F0F0;")
                self.spinbox2.setButtonSymbols(QAbstractSpinBox.NoButtons)
                self.spinbox3.setReadOnly(True)
                self.spinbox3.setStyleSheet("background-color: #F0F0F0;")
                self.spinbox3.setButtonSymbols(QAbstractSpinBox.NoButtons)

                # Set value to spinbox
                if self.form_widget.zrollpitch_tab.calib_result.get(self.idxSensor) is None:
                    self.form_widget.zrollpitch_tab.calib_result[self.idxSensor] = [0.0, 0.0, 0.0]

                calib_result = self.form_widget.zrollpitch_tab.calib_result[self.idxSensor]

                self.spinbox1.setValue(calib_result[0])
                self.spinbox2.setValue(calib_result[1])
                self.spinbox3.setValue(calib_result[2])
            elif status == CONST_MANUAL:
                # Reset spinbox format
                self.spinbox1.setReadOnly(False)
                self.spinbox1.setStyleSheet("background-color: #FFFFFF;")
                self.spinbox1.setButtonSymbols(QAbstractSpinBox.UpDownArrows)
                self.spinbox2.setReadOnly(False)
                self.spinbox2.setStyleSheet("background-color: #FFFFFF;")
                self.spinbox2.setButtonSymbols(QAbstractSpinBox.UpDownArrows)
                self.spinbox3.setReadOnly(False)
                self.spinbox3.setStyleSheet("background-color: #FFFFFF;")
                self.spinbox3.setButtonSymbols(QAbstractSpinBox.UpDownArrows)

                # Set value to spinbox
                current_tab = self.form_widget.GetCurrentTab()

                if current_tab.manual_zrp_calib_result.get(self.idxSensor) is None:
                    current_tab.manual_zrp_calib_result[self.idxSensor] = [0.0, 0.0, 0.0]

                calib_result = current_tab.manual_zrp_calib_result[self.idxSensor]

                self.spinbox1.setValue(calib_result[0])
                self.spinbox2.setValue(calib_result[1])
                self.spinbox3.setValue(calib_result[2])

        self.prev_checkID = self.button_group.checkedId()

    def DoubleSpinBoxChanged1(self):
        if self.id == CONST_EVALUATION_RESULT_LABEL:
            if self.button_group.checkedId() == CONST_CUSTOM:
                self.form_widget.evaluation_tab.custom_calibration_param[self.idxSensor][3] = self.spinbox1.value()

        if self.id == CONST_EVALUATION_ZRP_RESULT_LABEL:
            status = self.button_group.checkedId()
            if status == CONST_AUTOMATIC:
                return

            current_tab = self.form_widget.GetCurrentTab()
            current_tab.manual_zrp_calib_result[self.idxSensor][0] = self.spinbox1.value()

    def DoubleSpinBoxChanged2(self):
        if self.id == CONST_EVALUATION_RESULT_LABEL:
            if self.button_group.checkedId() == CONST_CUSTOM:
                self.form_widget.evaluation_tab.custom_calibration_param[self.idxSensor][4] = self.spinbox2.value()
        if self.id == CONST_EVALUATION_ZRP_RESULT_LABEL:
            status = self.button_group.checkedId()
            if status == CONST_AUTOMATIC:
                return

            current_tab = self.form_widget.GetCurrentTab()
            current_tab.manual_zrp_calib_result[self.idxSensor][1] = self.spinbox2.value()

    def DoubleSpinBoxChanged3(self):
        if self.id == CONST_EVALUATION_RESULT_LABEL:
            if self.button_group.checkedId() == CONST_CUSTOM:
                self.form_widget.evaluation_tab.custom_calibration_param[self.idxSensor][2] = self.spinbox3.value() * math.pi / 180.0
        if self.id == CONST_EVALUATION_ZRP_RESULT_LABEL:
            status = self.button_group.checkedId()
            if status == CONST_AUTOMATIC:
                return

            current_tab = self.form_widget.GetCurrentTab()
            current_tab.manual_zrp_calib_result[self.idxSensor][2] = self.spinbox3.value()

    ## Util functions
    def SetValue(self, value):
        self.spinbox1.setValue(round(value[0], 4))
        self.spinbox2.setValue(round(value[1], 4))
        self.spinbox3.setValue(round(value[2], 4))

class NewWindow(QMainWindow):
    def __init__(self, title, calib_result, form_widget, parent=None):
        super(NewWindow, self).__init__(parent)

        self.setWindowTitle(title)
        
        self.form_widget = form_widget
        self.calib_result = calib_result

        self.InitUi()

    def InitUi(self):
        self.frame = QFrame()
        self.vbox = QVBoxLayout()

        # vtk vehicle
        hbox_vtk = QHBoxLayout()
        self.vtkWidget = QVTKRenderWindowInteractor(self.frame)
        hbox_vtk.addWidget(self.vtkWidget)
        self.VTKInit()

        self.transparent_layout = LabelWithSliderLayout(CONST_VIEW_TRANSPARENT, 'Transparent', self.form_widget)
        current_tab = self.form_widget.GetCurrentTab()
        transparent = current_tab.transparent_layout.slider.value()
        self.vehicle_actor.GetProperty().SetOpacity(self.transparent_layout.ValueMapping(transparent))
        self.transparent_layout.slider.setValue(transparent)

        hbox_vtk.addLayout(self.transparent_layout)
        self.vbox.addLayout(hbox_vtk)

        hbox_2d_plane = QHBoxLayout()
        btn = QPushButton('xy plane')
        btn.clicked.connect(lambda: self.form_widget.XY(self.ren, self.renWin))
        hbox_2d_plane.addWidget(btn)

        btn = QPushButton('yz plane')
        btn.clicked.connect(lambda: self.form_widget.YZ(self.ren, self.renWin))
        hbox_2d_plane.addWidget(btn)

        btn = QPushButton('zx plane')
        btn.clicked.connect(lambda: self.form_widget.ZX(self.ren, self.renWin))
        hbox_2d_plane.addWidget(btn)
        self.vbox.addLayout(hbox_2d_plane)

    def VTKInit(self):
        vtk.vtkObject.GlobalWarningDisplayOff()
        colors = vtk.vtkNamedColors()

        self.ren = vtk.vtkRenderer()
        self.renWin = self.vtkWidget.GetRenderWindow()
        self.renWin.AddRenderer(self.ren)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        self.iren.SetRenderWindow(self.renWin)
        self.widget = vtk.vtkOrientationMarkerWidget()

        vtk_lidar_calib.GetAxis(self.iren, self.widget)

        lidar_info_dict = {}
        for i in range(len(self.form_widget.config.PARM_LIDAR['CheckedSensorList'])):
            idxSensors = list(self.form_widget.config.PARM_LIDAR['CheckedSensorList'])

            lidar_info = [self.calib_result[0][i], self.calib_result[1][i], self.calib_result[2][i],
                          self.calib_result[3][i], self.calib_result[4][i], self.calib_result[5][i]]
            lidar_info_dict[idxSensors[i]] = lidar_info
        actors = vtk_lidar_calib.GetActors(lidar_info_dict)
        self.vehicle_actor = vtk_lidar_calib.GetVehicleActor()
        actors.append(self.vehicle_actor)

        for actor in actors:
            self.ren.AddActor(actor)

        self.ren.SetBackground(colors.GetColor3d("white"))

        self.ren.ResetCamera()
        self.frame.setLayout(self.vbox)
        self.setCentralWidget(self.frame)

        self.iren.Initialize()
        self.iren.Start()

        self.resize(720, 720)
        self.show()
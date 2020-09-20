from PyQt5.QtWidgets import *
from PyQt5.QtCore import QDate, QSize, Qt
from PyQt5.QtGui import QPixmap
from widget import DoubleSlider
from widget import QThread
from PyQt5.QtCore import pyqtSignal
from process import get_result
import matplotlib.pyplot as plt
import numpy as np

CONST_DISPLAY_HANDEYE = 0
CONST_DISPLAY_OPTIMIZATION = 1
## A widget which has a edit label need the common function
# Get text
# Set Text
# Check extension
# Check label is empty
##
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
        try:
            self.ParseGnss()
            self.label_edit.setText(self.path_file_str)
        except:
            self.ErrorPopUp('Fail Open Gnss.csv File')

        self.ui.importing.point_cloud_logging_path = self.path_file_str
        self.ui.thread.SetFunc(self.ui.importing.ParsePointCloud)
        self.ui.thread.change_value.connect(self.pbar.setValue)
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

    def ParseGnss(self):
        self.ui.importing.gnss_logging_file = self.path_file_str
        self.ui.importing.ParseGnss()

    def ParsePointCloud(self):
        non_error = True
        for i in self.ui.importing.exist_arr:
            non_error = non_error * i

        if non_error:
            self.ui.importing_tab.logging_file_text_edit.clear()

            self.ui.importing_tab.logging_file_text_edit.append('[ Gnss File List ]')
            self.ui.importing_tab.logging_file_text_edit.append('Gnss.csv')
            self.ui.importing_tab.logging_file_text_edit.append('\n')
            self.ui.importing_tab.logging_file_text_edit.append('[ PointCloud File List ]')
            for idxSensor in self.ui.config.PARM_LIDAR['SensorList']:
                self.ui.importing_tab.logging_file_text_edit.append('PointCloud_' + str(idxSensor) + '.bin')

            parsed_pandas_dataframe = self.ui.importing.text_pointcloud
            self.parsed_bin = parsed_pandas_dataframe.to_string()

            default_start_time = self.ui.importing.DefaultStartTime
            default_end_time = self.ui.importing.DefaultEndTime

            # set slider default time
            self.ui.importing_tab.start_time_layout.slider.setMinimum(default_start_time)
            self.ui.importing_tab.start_time_layout.slider.setMaximum(default_end_time)
            self.ui.importing_tab.end_time_layout.slider.setMinimum(default_start_time)
            self.ui.importing_tab.end_time_layout.slider.setMaximum(default_end_time)

            # set slider and double_spin_box value
            self.ui.importing_tab.end_time_layout.slider.setValue(self.ui.importing_tab.end_time_layout.slider.maximum())
            self.ui.importing_tab.end_time_layout.double_spin_box.setValue(default_end_time)
            self.ui.importing_tab.start_time_layout.slider.setValue(self.ui.importing_tab.end_time_layout.slider.minimum())
            self.ui.importing_tab.start_time_layout.double_spin_box.setValue(default_start_time)
        else:
            index = 0
            for idxSensor in self.ui.config.PARM_LIDAR['SensorList']:
                if self.ui.importing.exist_arr[index] == 0:
                    self.ErrorPopUp('There are no PointCloud' + str(idxSensor) + '.bin file')
                index = index + 1

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
        self.listWidget.resize(20,20)
        self.listWidget.itemChanged.connect(self.ItemChanged)
        self.addWidget(self.listWidget)

    def AddWidgetItem(self, PARM_LIDAR):
        self.LiDAR_list.clear()
        listItems = self.listWidget.count()
        if not listItems == 0:
            for item_index in reversed(range(listItems)):
                self.listWidget.takeItem(item_index)
                self.listWidget.removeItemWidget(self.listWidget.takeItem(item_index))

        self.button_group = QButtonGroup()
        is_first = True
        for sensor_index in PARM_LIDAR:
            if self.label_str == 'Select Using Sensor List':
                item = QListWidgetItem('LiDAR %i' % sensor_index)
                item.setCheckState(Qt.Checked)
                self.listWidget.addItem(item)
                self.LiDAR_list.append(item)
            if self.label_str == 'Select Principle Sensor List':
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
            self.ui.config.PARM_LIDAR['SensorList'] = items

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
            if self.ui.config.PARM_LIDAR.get('SensorList') == None:
                return False
            if len(self.ui.config.PARM_LIDAR['SensorList']) <= 0:
                return False

            is_minus = False
            last_lidar_num = self.ui.config.PARM_LIDAR['SensorList'][-1]
            spin_box_value = self.spin_box.value()
            PARM_LIDAR_num = len(self.ui.config.PARM_LIDAR['SensorList'])
            add_lidar_num = spin_box_value - PARM_LIDAR_num

            if add_lidar_num < 0:
                is_minus = True
                add_lidar_num = add_lidar_num * -1

            if not is_minus:
                for i in range(add_lidar_num):
                    self.ui.config.PARM_LIDAR['SensorList'].append(last_lidar_num + 1)
                    last_lidar_num = last_lidar_num + 1
            else:
                for i in range(add_lidar_num):
                    if len(self.ui.config.PARM_LIDAR['SensorList']) <= 1:
                        self.spin_box.setValue(1)
                        return False
                    del self.ui.config.PARM_LIDAR['SensorList'][-1]

            self.ui.config_tab.select_using_sensor_list_layout.AddWidgetItem(self.ui.config.PARM_LIDAR['SensorList'])
            self.ui.optimization_tab.select_principle_sensor_list_layout.AddWidgetItem(self.ui.config.PARM_LIDAR['SensorList'])

        elif self.label_str == 'Sampling Interval':
            self.ui.config.PARM_HE['SamplingInterval'] = self.spin_box.value()
        elif self.label_str == 'Maximum Iteration':
            self.ui.config.PARM_HE['MaximumIteration'] = self.spin_box.value()
        elif self.label_str == 'Num Points Plane Modeling':
            self.ui.config.PARM_MO['NumPointsPlaneModeling'] = self.spin_box.value()

        self.ui.evaluation_tab.AddResultLabel()

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
        self.is_first_end_time_change = True
        self.slider_call_count = 0
        self.double_call_count = 0
        self.start_que = [0, 0]
        self.end_que = [0, 0]

        self.InitUi()

    def InitUi(self):
        self.slider = DoubleSlider.DoubleSlider(Qt.Horizontal)
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
        self.slider_call_count = self.slider_call_count + 1
        print('--label edit--')

        if self.label_str == 'Start Time [s]:':
            self.start_que[1] = self.start_que[0]
            self.start_que[0] = 0

            end_time = self.ui.importing_tab.end_time_layout.double_spin_box.value()
            if self.slider.value() > end_time:
                self.double_spin_box.setValue(end_time)
                self.slider.setValue(end_time)
                print('start up')
            else:
                self.double_spin_box.setValue(self.slider.value())
                print('start down')

            self.ui.importing.start_time = self.double_spin_box.value()

        elif self.label_str == 'End Time [s]:':
            start_time = self.ui.importing_tab.start_time_layout.double_spin_box.value()
            if self.slider.value() < start_time:
                self.double_spin_box.setValue(start_time)
                self.slider.setValue(start_time)
                print('end up')
            else:
                self.double_spin_box.setValue(self.slider.value())
                print('end down')

            self.ui.importing.end_time = self.double_spin_box.value()

    def DoubleSpinBoxChanged(self):
        self.double_call_count = self.double_call_count + 1
        print('--double--')

        if self.label_str == 'Start Time [s]:':
            self.start_que[1] = self.start_que[0]
            self.start_que[0] = 1
            end_time = self.ui.importing_tab.end_time_layout.double_spin_box.value()
            if self.double_spin_box.value() > end_time:
                self.double_spin_box.setValue(end_time)
                self.slider.setValue(end_time)
                print('start up 22')
                self.ErrorPopUp('Start time cannot be higher than end time')
            else:
                self.slider.setValue(self.double_spin_box.value())
                print('start down 22')

            self.ui.importing.start_time = self.double_spin_box.value()

        elif self.label_str == 'End Time [s]:':
            start_time = self.ui.importing_tab.start_time_layout.double_spin_box.value()
            if self.double_spin_box.value() < start_time:
                self.double_spin_box.setValue(start_time)
                self.slider.setValue(start_time)
                print('end up 22')
                self.ErrorPopUp('End time cannot be lower than start time')
            else:
                self.slider.setValue(self.double_spin_box.value())
                print('end down 22')

            self.ui.importing.end_time = self.double_spin_box.value()

    def ErrorPopUp(self, error_message):
        widget = QWidget()

        qr = widget.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        widget.move(qr.topLeft())

        QMessageBox.information(widget, 'Information', error_message)

class CalibrationResultEditLabel(QVBoxLayout):
    def __init__(self, id, pointcloud_num, calibration_param, ui):
        super().__init__()
        self.id = id
        self.pointcloud_num = pointcloud_num
        self.calibration_param = calibration_param
        self.ui = ui

        self.InitUi()

    def InitUi(self):
        self.label = QLabel('LiDAR {}'.format(self.pointcloud_num))
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
        color_list.append('salmon')
        color_list.append('darkorange')
        color_list.append('tan')
        color_list.append('yellowgreen')
        color_list.append('mediumseagreen')
        color_list.append('cadetiblue')
        color_list.append('lightskyblue')
        color_list.append('slategray')
        color_list.append('slateblue')
        color_list.append('mediumpurple')
        color_list.append('orchid')
        color_list.append('palevioletred')

        self.calibration_param[self.pointcloud_num][3] = self.double_spin_box_x.value()
        self.calibration_param[self.pointcloud_num][4] = self.double_spin_box_y.value()
        self.calibration_param[self.pointcloud_num][2] = self.double_spin_box_yaw.value()
        df_info, PARM_LIDAR, accum_pointcloud, accum_pointcloud_ = get_result.GetPlotParam(self.ui.config,
                                                                                           self.ui.importing,
                                                                                           self.ui.handeye.df_info,
                                                                                           self.calibration_param)
        self.ui.evaluation_tab.result_before_graph_ax.clear()
        '''
        self.ui.evaluation_tab.result_before_graph_ax.plot(df_info['east_m'].values, df_info['north_m'].values, 'gray', '.', label='trajectory')
        num = 0
        for idxSensor in PARM_LIDAR:
            num = num + 1
            strColIndex = 'PointCloud_' + str(idxSensor)
            self.ui.evaluation_tab.result_before_graph_ax.plot(accum_pointcloud_[idxSensor][:, 0], accum_pointcloud_[idxSensor][:, 1],color_list[num-1],',',label=strColIndex)

        self.ui.evaluation_tab.result_before_graph_ax.axis('equal')
        self.ui.evaluation_tab.result_before_graph_ax.legend()
        self.ui.evaluation_tab.result_before_graph_ax.grid()
        self.ui.evaluation_tab.result_before_graph_canvas.draw()
        '''

        self.ui.evaluation_tab.result_after_graph_ax.clear()
        '''
        self.ui.evaluation_tab.result_after_graph_ax.plot(df_info['east_m'].values, df_info['north_m'].values, 'gray','.', label='trajectory')
        num = 0
        for idxSensor in PARM_LIDAR:
            num = num + 1
            strColIndex = 'PointCloud_' + str(idxSensor)
            self.ui.evaluation_tab.result_after_graph_ax.plot(accum_pointcloud[idxSensor][:, 0], accum_pointcloud[idxSensor][:, 1], color_list[num-1],',', label=strColIndex)

        self.ui.evaluation_tab.result_after_graph_ax.axis('equal')
        self.ui.evaluation_tab.result_after_graph_ax.legend()
        self.ui.evaluation_tab.result_after_graph_ax.grid()
        self.ui.evaluation_tab.result_after_graph_canvas.draw()
        '''
        
        ###
        color_list = []
        color_list.append('salmon')
        color_list.append('darkorange')
        color_list.append('tan')
        color_list.append('yellowgreen')
        color_list.append('mediumseagreen')
        color_list.append('cadetiblue')
        color_list.append('lightskyblue')
        color_list.append('slategray')
        color_list.append('slateblue')
        color_list.append('mediumpurple')
        color_list.append('orchid')
        color_list.append('palevioletred')
        veh_path = 'image/vehicle1.png'
        veh = plt.imread(veh_path)
        self.ui.evaluation_tab.result_data_pose_ax.clear()
        num = 0
        for i in self.ui.config.PARM_LIDAR['SensorList']:
            num = num + 1

            self.ui.evaluation_tab.result_data_pose_ax.imshow(veh)
            
            x = int(calib_x[num-1])*200 + 500
            y = 1000 -1*int(calib_y[num-1])*200 - 500
                        
            car_length = 1.75
            lidar_num = 'lidar'+str(i)
            
            self.ui.evaluation_tab.result_data_pose_ax.scatter(x, y, s = 300, label = lidar_num, color = color_list[num-1],edgecolor = 'none')
            self.ui.evaluation_tab.result_data_pose_ax.arrow(x, y, 100*np.cos(calib_yaw[num-1]*np.pi/180), -100*np.sin(calib_yaw[num-1]*np.pi/180), head_width=10, head_length=10, fc='k', ec='k')
            self.ui.evaluation_tab.result_data_pose_ax.plot(np.linspace(500,x,100), np.linspace(500,y,100),color_list[num-1], '--')

         
            #ax.axes.xaxis.set_visible(False)
            #ax.axes.yaxis.set_visible(False)
        self.ui.evaluation_tab.result_data_pose_ax.axes.xaxis.set_visible(False)
        self.ui.evaluation_tab.result_data_pose_ax.axes.yaxis.set_visible(False)

        self.ui.evaluation_tab.result_data_pose_ax.set_xlim([-500,1500])
        self.ui.evaluation_tab.result_data_pose_ax.set_ylim([1000,0])
        self.ui.evaluation_tab.result_data_pose_ax.legend()
        self.ui.evaluation_tab.result_data_pose_ax.set_title('Result of calibration - '+method)
        self.ui.evaluation_tab.result_data_pose_canvas.draw()

        print('change btn')

class CalibrationResultLabel(QVBoxLayout):
    def __init__(self, pointcloud_num):
        super().__init__()
        self.pointcloud_num = pointcloud_num

        self.InitUi()

    def InitUi(self):
        self.label = QLabel('LiDAR {}'.format(self.pointcloud_num))
        self.addWidget(self.label)

        self.hbox = QHBoxLayout()
        self.label_x = QLabel('x [m]')
        self.hbox.addWidget(self.label_x)
        self.label_edit_x = QLineEdit()
        self.hbox.addWidget(self.label_edit_x)

        self.label_y = QLabel('y [m]')
        self.hbox.addWidget(self.label_y)
        self.label_edit_y = QLineEdit()
        self.hbox.addWidget(self.label_edit_y)

        self.label_yaw = QLabel('yaw [deg]')
        self.hbox.addWidget(self.label_yaw)
        self.label_edit_yaw = QLineEdit()
        self.hbox.addWidget(self.label_edit_yaw)

        self.addLayout(self.hbox)

class ResultTab(QVBoxLayout):
    def __init__(self, ui):
        super().__init__()
        self.ui = ui
        # print(self.ui.config.PARM_LIDAR)
    #
    #     self.initUI()
    #
    # def initUI(self):
    #     self.default_init_value = self.DefaultInit()
    #     self.handeye_init_value = QWidget()
    #     self.DefaultInit()
    #
    #     tabs = QTabWidget()
    #     tabs.addTab(self.default_init_value, 'Default')
    #     tabs.addTab(self.handeye_init_value, 'Handeye')
    #     self.addWidget(tabs)
    #
    def DefaultInit(self):
        self.widget = QWidget()
        self.scrollarea = QScrollArea(self.widget)
        self.scrollarea.setWidgetResizable(True)

        widget = QWidget()
        self.scrollarea.setWidget(widget)
        self.layout_SArea = QVBoxLayout(widget)

class ImageDisplay(QWidget):
    def __init__(self):
        super().__init__()
        self.InitUi()
    def InitUi(self):
        self.im = QPixmap('image/config.png')
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

import time
import threading


# class DataValidation(QWidget):
#     def __init__(self, form_widget):
#         super().__init__()
#         self.using_gnss_motion = False
#         self.form_widget = form_widget
#
#         self.progress_status = CONST_STOP
#         self.result_labels = {}
#         self.label_heading_threshold = self.form_widget.config.PARM_DV['FilterHeadingThreshold']
#         self.label_distance_threshold = self.form_widget.config.PARM_DV['FilterDistanceThreshold']
#         self.initUi()
#
#     def initUi(self):
#         main_vbox = QVBoxLayout()
#
#         main_hbox = QHBoxLayout()
#
#         self.config_widget = QWidget()
#         self.config_widget.setLayout(self.Configuration_Layout())
#         main_hbox.addWidget(self.config_widget, 25)
#
#         self.result_widget = QWidget()
#         self.result_widget.setLayout(self.Result_Layout())
#         main_hbox.addWidget(self.result_widget, 75)
#         # self.setLayout(main_hbox)
#
#         main_vbox.addLayout(main_hbox, 50)
#         self.next_btn = QPushButton('Skip')
#         self.next_btn.clicked.connect(self.NextBtn)
#         main_vbox.addWidget(self.next_btn, 50)
#
#         self.setLayout(main_vbox)
#
#     def Configuration_Layout(self):
#         self.configuration_vbox = QVBoxLayout()
#
#         self.configuration_vbox.addWidget(self.Configuration_DataInfo_Groupbox())
#
#         hbox = QHBoxLayout()
#         label = QLabel('Used Data')
#         hbox.addWidget(label)
#
#         # button for Handeye calibration
#         self.button_group = QButtonGroup()
#         rbn1 = QRadioButton('GNSS Data')
#         rbn1.setChecked(True)
#         rbn1.clicked.connect(self.RadioButton)
#         hbox.addWidget(rbn1)
#         self.button_group.addButton(rbn1, 1)
#
#         # button for unsupervised calibration
#         rbn2 = QRadioButton('Motion Data')
#         rbn2.clicked.connect(self.RadioButton)
#         hbox.addWidget(rbn2)
#         self.button_group.addButton(rbn2, 2)
#         self.configuration_vbox.addLayout(hbox)
#
#         self.configuration_vbox.addWidget(self.Configuration_Validation_Groupbox())
#
#         return self.configuration_vbox
#
#     def Result_Layout(self):
#         vbox = QVBoxLayout()
#
#         vbox.addWidget(self.Result_ResultData_Groupbox())
#         vbox.addWidget(self.Result_ResultGraph_Groupbox())
#
#         return vbox
#
#     def Result_ResultData_Groupbox(self):
#         groupbox = QGroupBox('Translation RMSE')
#         vbox = QVBoxLayout()
#
#         self.result_data_pose_fig = plt.figure()
#         self.result_data_pose_canvas = FigureCanvas(self.result_data_pose_fig)
#         self.result_data_pose_ax = self.result_data_pose_fig.add_subplot(1, 1, 1)
#         self.result_data_pose_ax.grid()
#         self.result_data_pose_canvas.draw()
#         vbox.addWidget(self.result_data_pose_canvas)
#
#         btn = QPushButton('View')
#         btn.clicked.connect(self.ViewTranslationError)
#         vbox.addWidget(btn)
#
#         groupbox.setLayout(vbox)
#         return groupbox
#
#     def Result_ResultGraph_Groupbox(self):
#         groupbox = QGroupBox('Rotation RMSE')
#         vbox = QVBoxLayout()
#
#         self.result_graph_fig = plt.figure()
#         self.result_graph_canvas = FigureCanvas(self.result_graph_fig)
#         self.result_graph_ax = self.result_graph_fig.add_subplot(1, 1, 1)
#         self.result_graph_ax.grid()
#         self.result_graph_canvas.draw()
#         vbox.addWidget(self.result_graph_canvas)
#
#         btn = QPushButton('View')
#         btn.clicked.connect(self.ViewRotationError)
#         vbox.addWidget(btn)
#
#         groupbox.setLayout(vbox)
#         return groupbox
#
#     def Configuration_DataInfo_Groupbox(self):
#         groupbox = QGroupBox('Configuration Info')
#         vbox = QVBoxLayout()
#
#         ICP_configuration_label = QLabel('[ ICP Configuration ]', self)
#         vbox.addWidget(ICP_configuration_label)
#
#         self.maximum_interation_layout = element.SpinBoxLabelLayout('Maximum Iteration [Count]', self.form_widget)
#         vbox.addLayout(self.maximum_interation_layout)
#
#         self.tolerance_layout = element.DoubleSpinBoxLabelLayout('Tolerance', self.form_widget)
#         vbox.addLayout(self.tolerance_layout)
#
#         self.outlier_distance_layout = element.DoubleSpinBoxLabelLayout('Outlier Distance [m]', self.form_widget)
#         vbox.addLayout(self.outlier_distance_layout)
#
#         Threshold_configuration_label = QLabel('[ Threshold ]', self)
#         vbox.addWidget(Threshold_configuration_label)
#         self.heading_threshold_layout = element.DoubleSpinBoxLabelLayout('Heading Threshold (filter)', self.form_widget)
#         vbox.addLayout(self.heading_threshold_layout)
#
#         self.distance_threshold_layout = element.DoubleSpinBoxLabelLayout('Distance Threshold (filter)',
#                                                                           self.form_widget)
#         vbox.addLayout(self.distance_threshold_layout)
#
#         groupbox.setLayout(vbox)
#
#         print(self.maximum_interation_layout)
#
#         return groupbox
#
#     def Configuration_Validation_Groupbox(self):
#         groupbox = QGroupBox('Data Validation')
#         vbox = QVBoxLayout(self)
#
#         hbox = QHBoxLayout()
#         btn = QPushButton('Start')
#         btn.clicked.connect(lambda: self.StartValidation(CONST_HANDEYE,
#                                                          self.form_widget.datavalidation.Validation,
#                                                          self.form_widget.config.PARM_IM['VehicleSpeedThreshold'],
#                                                          self.form_widget.importing_tab.limit_time_layout.start_time,
#                                                          self.form_widget.importing_tab.limit_time_layout.end_time,
#                                                          self.form_widget.config.PARM_LIDAR,
#                                                          [self.text_edit.clear, self.calibration_pbar.reset],
#                                                          [self.text_edit.append, self.calibration_pbar.setValue],
#                                                          self.EndValibration))
#
#         hbox.addWidget(btn)
#
#         self.pause_btn = QPushButton('Pause')
#         self.pause_btn.clicked.connect(self.Pause)
#         hbox.addWidget(self.pause_btn)
#         vbox.addLayout(hbox)
#
#         stop_btn = QPushButton('Stop')
#         stop_btn.clicked.connect(self.Cancel)
#         hbox.addWidget(stop_btn)
#
#         vbox.addLayout(hbox)
#
#         self.label = QLabel('[ Data Validation Progress ]')
#         vbox.addWidget(self.label)
#
#         self.calibration_pbar = QProgressBar(self)
#         vbox.addWidget(self.calibration_pbar)
#
#         self.text_edit = QTextEdit()
#         vbox.addWidget(self.text_edit)
#
#         self.scroll_box = ScrollAreaV()
#         vbox.addWidget(self.scroll_box)
#
#         groupbox.setLayout(vbox)
#         return groupbox
#
#     def NextBtn(self):
#         self.form_widget.tabs.setCurrentIndex(CONST_HANDEYE_TAB)
#
#     def StartValidation(self, validation_exist, validation, vehicle_speed_threshold, start_time, end_time, sensor_list,
#                         targets_clear, progress_callbacks, end_callback):
#         if self.form_widget.config_tab.is_lidar_num_changed == True:
#             self.form_widget.ErrorPopUp('Please import after changing lidar number')
#             return False
#         if self.progress_status is not CONST_STOP:
#             return False
#         self.progress_status = CONST_PLAY
#
#         for idxSensor in sensor_list['CheckedSensorList']:
#             if self.form_widget.importing.PointCloudSensorList.get(idxSensor) is None:
#                 self.form_widget.ErrorPopUp('Import pointcloud {}'.format(idxSensor))
#                 return False
#
#         for target_clear in targets_clear:
#             target_clear()
#
#         try:
#             self.result_data_pose_ax.clear()
#             self.result_data_pose_canvas.draw()
#             self.result_graph_ax.clear()
#             self.result_graph_canvas.draw()
#             # print("clear results in unsupervised")
#         except:
#             pass
#         self.form_widget.validation_thread._status = True
#         self.form_widget.validation_thread.SetFunc(validation,
#                                                    [start_time, end_time, sensor_list, self.using_gnss_motion,
#                                                     vehicle_speed_threshold])
#         try:
#             self.form_widget.validation_thread.change_value.disconnect()
#         except:
#             pass
#         try:
#             self.form_widget.validation_thread.iteration_percentage.disconnect()
#         except:
#             pass
#         try:
#             self.form_widget.validation_thread.end.disconnect()
#         except:
#             pass
#         try:
#             self.form_widget.validation_thread.emit_string.disconnect()
#         except:
#             pass
#
#         self.form_widget.validation_thread.emit_string.connect(progress_callbacks[0])  # text_edit_callback
#         self.form_widget.validation_thread.change_value.connect(progress_callbacks[1])  # progress_callback
#
#         self.form_widget.validation_thread.end.connect(end_callback)
#         self.form_widget.validation_thread.start()
#
#     def Pause(self):
#         if self.progress_status is CONST_PLAY:
#             self.progress_status = CONST_PAUSE
#
#             self.form_widget.validation_thread.pause = True
#             self.pause_btn.setText("Resume")
#
#         elif self.progress_status is CONST_PAUSE:
#             self.progress_status = CONST_PLAY
#
#             self.form_widget.validation_thread.pause = False
#             self.pause_btn.setText("Pause")
#
#         else:
#             return False
#
#     def Cancel(self):
#         self.progress_status = CONST_STOP
#         self.form_widget.validation_thread.pause = False
#         self.pause_btn.setText("Pause")
#         self.form_widget.validation_thread.toggle_status()
#
#     def EndValibration(self):
#         self.progress_status = CONST_STOP
#         self.form_widget.datavalidation.complete_validation = True
#
#         # print(self.form_widget.config.PARM_DV['FilterHeadingThreshold'])
#
#         # element.ValidationConfigLabel.label_edit_heading_threshold.setText(format(self.form_widget.config.PARM_DV['FilterHeadingThreshold'],".4f"))
#         # element.ValidationConfigLabel.label_edit_heading_threshold.setText(format(self.form_widget.config.PARM_DV['FilterDistanceThreshold'],".4f"))
#
#         print(self.form_widget.config.PARM_DV['FilterHeadingThreshold'])
#
#         # self.label_heading_threshold = QLabel('Heading Threshold [deg]: {}'.format(self.form_widget.config.PARM_DV['FilterHeadingThreshold']))
#
#         # self.label_distance_threshold = QLabel('Distance Threshold [m]: {}'.format(self.form_widget.config.PARM_DV['FilterDistanceThreshold']))
#
#         # self.label_heading_threshold.label_edit_heading_threshold.setText(format(self.label_heading_threshold, ".4f"))
#         # self.label_distance_threshold.label_edit_distance_threshold.setText(format(self.label_distance_threshold, ".4f"))
#         # self.result_labels[0].label_edit_heading_threshold.setText(format(self.label_heading_threshold, ".4f"))
#         # self.result_labels[0].label_edit_distance_threshold.setText(format(self.label_distance_threshold, ".4f"))
#
#         # validation tab
#         ## Set 'Result Calibration Data'
#
#         for idxSensor in self.form_widget.datavalidation.PARM_LIDAR['CheckedSensorList']:
#             self.result_labels[idxSensor].label_edit_translation_error.setText(
#                 format(self.form_widget.datavalidation.RMSETranslationErrorDict[idxSensor], ".4f"))
#             self.result_labels[idxSensor].label_edit_rotation_error.setText(
#                 format(self.form_widget.datavalidation.RMSERotationErrorDict[idxSensor], ".4f"))
#
#         ## Plot 'Result Data'
#         self.result_data_pose_ax.clear()
#         self.form_widget.ViewTranslationError(self.form_widget.datavalidation.TranslationError,
#                                               self.form_widget.datavalidation.PARM_LIDAR, self.result_data_pose_ax,
#                                               self.result_data_pose_canvas)
#
#         ## Plot 'Result Graph'
#         self.result_graph_ax.clear()
#         self.form_widget.ViewRotationError(self.form_widget.datavalidation.RotationError,
#                                            self.form_widget.datavalidation.PARM_LIDAR, self.result_graph_ax,
#                                            self.result_graph_canvas)
#
#     def ViewTranslationError(self):
#         if self.progress_status is not CONST_STOP:
#             return False
#         self.form_widget.ViewTranslationError(self.form_widget.datavalidation.TranslationError,
#                                               self.form_widget.config.PARM_LIDAR)
#
#     def ViewRotationError(self):
#         if self.progress_status is not CONST_STOP:
#             return False
#         self.form_widget.ViewRotationError(self.form_widget.datavalidation.RotationError,
#                                            self.form_widget.config.PARM_LIDAR)
#
#     def RadioButton(self):
#         '''
#         only one button can selected
#         '''
#         status = self.button_group.checkedId()
#         if status == 1:  # GNSS Data
#             if self.form_widget.importing.has_gnss_file == False:
#                 self.button_group.button(self.prev_checkID).setChecked(True)
#                 self.form_widget.ErrorPopUp('Please import Gnss.csv')
#                 return False
#             self.using_gnss_motion = False
#         elif status == 2:  # Motion Data
#             if self.form_widget.importing.has_motion_file == False:
#                 self.button_group.button(self.prev_checkID).setChecked(True)
#                 self.form_widget.ErrorPopUp('Please import Motion.csv')
#                 return False
#             self.using_gnss_motion = True
#         self.prev_checkID = self.button_group.checkedId()


#------------------------------------------------------------------------------------------------------------------------


# def ViewLiDAR(self, calib_x, calib_y, calib_yaw, PARM_LIDAR, vtkWidget=None):
#     colors = vtk.vtkNamedColors()
#
#     ren = vtk.vtkRenderer()
#     vtkWidget.GetRenderWindow().AddRenderer(ren)
#     iren = vtkWidget.GetRenderWindow().GetInteractor()
#
#     lidar_info_list = [[0, 0, 0, 0, 0, 0],
#                        [3.01, -0.03, 0.65, 0.42, 0.99, 0.48],
#                        [3.05, -0.19, 0.69, 0.75, -1.71, -0.52],
#                        [0.74, 0.56, 1.56, -25.64, 0.82, -0.13],
#                        [0.75, -0.06, 1.7, -0.12, 0.08, 1.3],
#                        [0.72, -0.83, 1.57, 4.58, 0.46, -0.31]]
#     vehicle_info = [4371, 1904, 1649, 90, 90, 0]
#
#     # -------------------------------------------create coordinate axes instances
#
#     # Get axis
#     axes = vtk.vtkAxesActor()
#
#     widget = vtk.vtkOrientationMarkerWidget()
#     rgba = [0] * 4
#     colors.GetColor('Carrot', rgba)
#     widget.SetOutlineColor(rgba[0], rgba[1], rgba[2])
#     widget.SetOrientationMarker(axes)
#     widget.SetInteractor(iren)
#     widget.SetViewport(0, 0, 0.2, 0.2)
#     widget.SetEnabled(1)
#     widget.InteractiveOn()
#
#     actors = GetActors(lidar_info_list, vehicle_info)
#
#     # Assign actor to the renderer
#     for actor in actors:
#         ren.AddActor(actor)
#     ren.SetBackground(colors.GetColor3d('white'))
#
#     # ren.AddActor(actor)
#     ren.ResetCamera()
#
#     # frame.setLayout(vbox)
#
#     iren.Initialize()
#     iren.Start()


#--------------------------------------------------------------------------------------------------------------------------------------------


# colors = vtk.vtkNamedColors()
# # frame = QFrame()
# # vtkWidget = QVTKRenderWindowInteractor(frame)
# vtkWidget = QVTKRenderWindowInteractor()
# vbox.addWidget(vtkWidget)
#
# ren = vtk.vtkRenderer()
# vtkWidget.GetRenderWindow().AddRenderer(ren)
# iren = vtkWidget.GetRenderWindow().GetInteractor()
#
# lidar_info_list = [[0, 0, 0, 0, 0, 0],
#                    [3.01, -0.03, 0.65, 0.42, 0.99, 0.48],
#                    [3.05, -0.19, 0.69, 0.75, -1.71, -0.52],
#                    [0.74, 0.56, 1.56, -25.64, 0.82, -0.13],
#                    [0.75, -0.06, 1.7, -0.12, 0.08, 1.3],
#                    [0.72, -0.83, 1.57, 4.58, 0.46, -0.31]]
# vehicle_info = [4371, 1904, 1649, 90, 90, 0]
#
# # -------------------------------------------create coordinate axes instances
#
# # Get axis
# axes = vtk.vtkAxesActor()
#
# widget = vtk.vtkOrientationMarkerWidget()
# rgba = [0] * 4
# colors.GetColor('Carrot', rgba)
# widget.SetOutlineColor(rgba[0], rgba[1], rgba[2])
# widget.SetOrientationMarker(axes)
# widget.SetInteractor(iren)
# widget.SetViewport(0, 0, 0.2, 0.2)
# widget.SetEnabled(1)
# widget.InteractiveOn()
#
#
# actors = calib_result.GetActors(lidar_info_list, vehicle_info)
#
# # Assign actor to the renderer
# for actor in actors:
#     ren.AddActor(actor)
# ren.SetBackground(colors.GetColor3d('white'))
#
# # ren.AddActor(actor)
# ren.ResetCamera()
#
# # frame.setLayout(vbox)
#
# iren.Initialize()
# iren.Start()
#
# vbox.addWidget(self.lidar_calib_pose.vtkWidget)

class AddDaemon(threading.Thread):
    def __init__(self):
        super().__init__()
        self.stuff = 'hi there this is AddDaemon'

    def run(self):
        while True:
            print(self.stuff)
            time.sleep(3)


class RemoveDaemon(threading.Thread):
    def __init__(self):
        super().__init__()
        self.stuff = 'hi this is RemoveDaemon'

    def run(self):
        while True:
            print(self.stuff)
            time.sleep(1)


a = AddDaemon()
b = RemoveDaemon()

a.start()
# b.start()

if a.is_alive():
    print("a alive")
else:
    print("a dead")

if b.is_alive():
    print("b alive")
else:
    print("b dead")

time.sleep(100)
print("end")

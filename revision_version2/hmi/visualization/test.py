import time
import threading

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

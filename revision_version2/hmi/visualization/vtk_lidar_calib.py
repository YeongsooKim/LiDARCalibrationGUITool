#!/usr/bin/env python
"""
@author : YoungRok Son
@author email: dudfhr3349@gmail.com
@author : Yeongsoo Kim
@author email: kimkys768@gmail.com
"""
import vtk
import numpy as np

'''
#TODO 
1. Make same coordinate with vehicle and Lidar
- Make size to meter unit > lidar and vehicle
- Align direction with car and lidar
2. make multiple lidar according to dictionary data which is come from other module
- use for sentence

'''

# Length, Width, Height, STL front axis, STL Top axis
reference_origin_ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
vehicle_info_ = [0.0, 0.0, 0.0, '', '']
vehicle_stl_path_ = ''
lidar_stl_path_ = ''
ground_stl_path_ = ''

X_PIX2METER = 1.0 / 0.0006887739372658
Y_PIX2METER = 1.0 / 0.0006887739372658
Z_PIX2METER = 100.0 / 0.07200000000000001

### Transformation
# Rotate vehicle according to the value of vehicle_info_[3,4,5]
def GetRotation(vehicleTransfromObject):
    global vehicle_info_
    front_axis = vehicle_info_[3]
    top_axis = vehicle_info_[4]

    if front_axis == 'X' and top_axis == 'Y':
        rot_x_deg = 90; rot_y_deg = 0; rot_z_deg = 0
    elif front_axis == 'X' and top_axis == 'Z':
        rot_x_deg = 0; rot_y_deg = 0; rot_z_deg = 0
    elif front_axis == 'Y' and top_axis == 'X':
        rot_x_deg = -90; rot_y_deg = 0; rot_z_deg = -90
    elif front_axis == 'Y' and top_axis == 'Z':
        rot_x_deg = 0; rot_y_deg = 0; rot_z_deg = -90
    elif front_axis == 'Z' and top_axis == 'X':
        rot_x_deg = -90; rot_y_deg = 90; rot_z_deg = -90
    elif front_axis == 'Z' and top_axis == 'Y':
        rot_x_deg = 90; rot_y_deg = 90; rot_z_deg = 0

    vehicleTransfromObject.RotateX(rot_x_deg)
    vehicleTransfromObject.RotateY(rot_y_deg)
    vehicleTransfromObject.RotateZ(rot_z_deg)

    return vehicleTransfromObject


def GetBoundarySize(boundary):
    if boundary[0] * boundary[1] > 0.0:
        x_length_pix = abs(boundary[0] + boundary[1])
    elif boundary[0] * boundary[1] < 0.0:
        x_length_pix = abs(boundary[0] - boundary[1])
    else:
        if boundary[0] == 0.0:
            x_length_pix = abs(boundary[1])
        elif boundary[1] == 0.0:
            x_length_pix = abs(boundary[0])

    # get Y-axis length
    if boundary[2] * boundary[3] > 0.0:
        y_length_pix = abs(boundary[2] + boundary[3])
    elif boundary[2] * boundary[3] < 0.0:
        y_length_pix = abs(boundary[2] - boundary[3])
    else:
        if boundary[2] == 0.0:
            y_length_pix = abs(boundary[3])
        elif boundary[3] == 0.0:
            y_length_pix = abs(boundary[2])

    # get Z-axis length
    if boundary[4] * boundary[5] > 0.0:
        z_length_pix = abs(boundary[4] + boundary[5])
    elif boundary[4] * boundary[5] < 0.0:
        z_length_pix = abs(boundary[4] - boundary[5])
    else:
        if boundary[4] == 0.0:
            z_length_pix = abs(boundary[5])
        elif boundary[5] == 0.0:
            z_length_pix = abs(boundary[4])

    boundary_size = [x_length_pix, y_length_pix, z_length_pix]

    return boundary_size


def GetVehicleRatio(boundary):
    boundary_size = GetBoundarySize(boundary)
    length_mm = max(boundary_size)

    scale_ratio = vehicle_info_[0]/length_mm

    return scale_ratio


def LidarSizeScaling():
    # the unit is in meter
    # make the car size to real size in meter
    width_m = 103.3 / 1000  # 75.6 mm to meter
    heigth_m = 75.6 / 1000
    length_m = 103.3 / 1000
    width_pix = 150
    heigth_pix = 105
    length_pix = 149.9766387939453

    lidar_size = [length_m / length_pix, width_m / width_pix, heigth_m / heigth_pix]

    return lidar_size


def LidarTranslatePixel2Meter(x, y, z):
    global reference_origin_
    translate_x = (x + reference_origin_[0]) * X_PIX2METER
    translate_y = (y + reference_origin_[1]) * Y_PIX2METER
    translate_z = (z + reference_origin_[2]) * Z_PIX2METER

    pos = [translate_x, translate_y, translate_z]

    return pos


def RPY2Rotation(r, p, y):
    global reference_origin_

    r_rotation = [r + reference_origin_[3], 1, 0, 0]
    p_rotation = [p + reference_origin_[4], 0, 1, 0]
    y_rotation = [y + reference_origin_[5], 0, 0, 1]

    return r_rotation, p_rotation, y_rotation


### Get actors
def GetActors(lidar_info_dict):
    actors = []

    ground_actor = GetGridActor()
    actors.append(ground_actor)

    lidar_actors = GetLiDARActors(lidar_info_dict)
    for actor in lidar_actors:
        actors.append(actor)

    return actors


def GetLiDARActors(lidar_info_dict):
    # -------------------------------------------create vehicle instances
    colors = vtk.vtkNamedColors()
    color_list = ['salmon', 'royalblue', 'lightblue', 'mediumorchid', 'mediumseagreen', 'gold', 'mediumslateblue',
                  'darkorange']

    # -------------------------------------------create LiDAR instances
    # LiDAR Source
    vtk_reader = vtk.vtkSTLReader()
    global lidar_stl_path_
    vtk_reader.SetFileName(lidar_stl_path_)

    # Mapper
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(vtk_reader.GetOutputPort())

    # add lidar actors
    names = []
    actors = []
    transforms = []

    for i, lidar_info_key in enumerate(lidar_info_dict):
        names.append("LiDAR " + str(lidar_info_key))

        # set actors
        actors.append(vtk.vtkActor())
        actors[i].SetMapper(mapper)
        actors[i].GetProperty().SetDiffuse(0.8)
        actors[i].GetProperty().SetDiffuseColor(colors.GetColor3d(color_list[i]))
        actors[i].GetProperty().SetSpecular(0.3)
        actors[i].GetProperty().SetSpecularPower(60.0)

        # set LiDAR Properties
        pose = lidar_info_dict[lidar_info_key]
        pos = pose[:3]

        orientation = pose[3:]
        transforms.append(vtk.vtkTransform())
        # set size
        size = LidarSizeScaling()
        transforms[i].Scale(size)

        # set orientation
        r_rotation, p_rotation, y_rotation = RPY2Rotation(orientation[0], orientation[1], orientation[2])

        # set position
        aligned_pos = LidarTranslatePixel2Meter(pose[0], pose[1], pose[2])
        transforms[i].Translate(aligned_pos)

        transforms[i].RotateWXYZ(r_rotation[0], r_rotation[1], r_rotation[2], r_rotation[3])
        transforms[i].RotateWXYZ(p_rotation[0], p_rotation[1], p_rotation[2], p_rotation[3])
        transforms[i].RotateWXYZ(y_rotation[0], y_rotation[1], y_rotation[2], y_rotation[3])

        actors[i].SetUserTransform(transforms[i])

    return actors


def GetVehicleActor():
    colors = vtk.vtkNamedColors()

    # Vehicle Source
    vtk_reader = vtk.vtkSTLReader()
    global vehicle_stl_path_
    vtk_reader.SetFileName(vehicle_stl_path_)

    # Mapper
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(vtk_reader.GetOutputPort())

    # Actor
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    # Set VehicleActor Property
    actor.GetProperty().SetDiffuse(0.8)
    actor.GetProperty().SetDiffuseColor(colors.GetColor3d('silver'))
    actor.GetProperty().SetSpecular(0.0)
    actor.GetProperty().SetSpecularPower(60.0)
    actor.GetProperty().SetOpacity(1.0)

    # Set Vehicle size and pose
    scale_ratio = GetVehicleRatio(actor.GetBounds())
    actor.SetScale(scale_ratio, scale_ratio, scale_ratio)

    transformation = vtk.vtkTransform()
    rotation = GetRotation(transformation)
    actor.SetUserTransform(rotation)

    return actor


def GetAxis(iren, widget):
    # Get axis
    axes = vtk.vtkAxesActor()

    rgba = [0] * 4
    colors = vtk.vtkNamedColors()
    colors.GetColor('Carrot', rgba)
    widget.SetOutlineColor(rgba[0], rgba[1], rgba[2])
    widget.SetOrientationMarker(axes)
    widget.SetInteractor(iren)
    widget.SetViewport(0, 0, 0.2, 0.2)
    widget.SetEnabled(1)
    widget.InteractiveOn()


def GetGridActor():
    colors = vtk.vtkNamedColors()

    # Vehicle Source
    vtk_reader = vtk.vtkSTLReader()
    global ground_stl_path_
    vtk_reader.SetFileName(ground_stl_path_)

    # Mapper
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(vtk_reader.GetOutputPort())

    # Actor
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    # Set VehicleActor Property
    actor.GetProperty().SetDiffuse(0.8)
    actor.GetProperty().SetDiffuseColor(colors.GetColor3d('black'))
    actor.GetProperty().SetSpecular(0.0)
    actor.GetProperty().SetSpecularPower(60.0)
    actor.GetProperty().SetOpacity(0.05)

    # Set Vehicle size and pose
    actor.SetScale(1 / 1000, 1 / 1000, 1 / 1000)

    return actor


if __name__ == '__main__':
    import sys

    vtk.vtkObject.GlobalWarningDisplayOff()

    # [x, y, z, roll, pitch, yaw]
    lidar_info_dict = {}
    lidar_info_dict[0] = [0, 0, 0, 0, 0, 0]
    lidar_info_dict[1] = [3.55, -0.19, 0.65, 0.42, 0.99, 0.48]
    lidar_info_dict[2] = [3.55, 0.19, 0.69, 0.75, -1.71, -0.52]
    lidar_info_dict[3] = [1.24, 0.56, 1.67, -25, .64, 0.82, -0.13]
    lidar_info_dict[4] = [1.25, -0.0, 1.7, -0.12, 0.08, 1.3]
    lidar_info_dict[5] = [1.22, -0.56, 1.67, 25, 0.46, -0.31]

    SetVehicleStlPath('D:/git_ws/calibration_guitool/revision_version2/common/meshes/vehicles/Evoque.stl')
    SetLidarStlPath('D:/git_ws/calibration_guitool/revision_version2/common/meshes/lidars/lidar.stl')
    SetGridStlPath('D:/git_ws/calibration_guitool/revision_version2/common/meshes/grid/grid.stl')
    SetVehicleInfo([4.371, 1.904, 1.649, 90, 90, 0, 0, 0, 0])

    ren = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)
    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    # Assign actor to the renderer
    actors = GetActors(lidar_info_dict)
    for actor in actors:
        ren.AddActor(actor)

    widget = vtk.vtkOrientationMarkerWidget()
    GetAxis(iren, widget)

    colors = vtk.vtkNamedColors()
    ren.SetBackground(colors.GetColor3d('white'))

    # Add the actors to the renderer, set the background and size
    renWin.SetSize(1000, 1000)
    renWin.SetWindowName('Result of LiDAR Calibration')

    iren.Initialize()

    ren.GetActiveCamera().SetPosition(0, 1, 0)
    # ren.GetActiveCamera().SetFocalPoint(0, 0, -90)
    # ren.GetActiveCamera().GetViewUp()

    ren.ResetCamera()
    ren.GetActiveCamera().Zoom(1.5)
    renWin.Render()

    iren.Start()

    # CalibResult(lidar_info_list, vehicle_info_)

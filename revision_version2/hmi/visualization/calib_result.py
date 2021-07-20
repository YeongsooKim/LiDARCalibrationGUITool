#!/usr/bin/env python
"""
@author : YoungRok Son
@author email: dudfhr3349@gmail.com
@author : Yeongsoo Kim
@author email: kimkys768@gmail.com
"""
import vtk
import numpy as np
import threading
import time

'''
#TODO 
1. Make same coordinate with vehicle and Lidar
- Make size to meter unit > lidar and vehicle
- Align direction with car and lidar
2. make multiple lidar according to dictionary data which is come from other module
- use for sentence

'''

# Length, Width, Height, x rotation, y rotation, z rotation, x translation, y translation, z translation
 # youngrok add 2021/07/20
 # Add vehicle rotation parameters at the end of list.
vehicle_info_ = [4.371, 1.904, 1.649, 90, 90, 0,0,0,0]

test_global_value_ = 10

X_PIX2METER = 1.0/0.0006887739372658
Y_PIX2METER = 1.0/0.0006887739372658
Z_PIX2METER = 100.0/0.07200000000000001

# youngrok add 2021/07/20
# Parameters for tranlsation in meter.
X_TRANSLATE_PIX2METER = 1.0/1.008755986325648 
Y_TRANSLATE_PIX2METER = 1.0/0.89637191416309
Z_TRANSLATE_PIX2METER = 1.0/1.007424376155878

 # youngrok add 2021/07/20
 # move vehicle to center
def moveCenterVehicleTranslation(boundary):
    centerlize_yaxis = -(boundary[2]+boundary[3])/2
    height2zero = -boundary[4]
    return [0, centerlize_yaxis, height2zero]
 # youngrok add 2021/07/20
 # Rotate vehicle according to the value of vehicle_info_[6,7,8]
def getRotation(vehicleTransfromObject, vehicleInfo):
    vehicleTransfromObject.RotateX(vehicle_info_[6])
    vehicleTransfromObject.RotateY(vehicle_info_[7])
    vehicleTransfromObject.RotateZ(vehicle_info_[8])
    
    return vehicleTransfromObject

 # youngrok add 2021/07/20
 # Move vehicle in Meter.
def translateVehicle(x,y,z): # Pixel to Meter
    
    aligned_poision = [x * X_TRANSLATE_PIX2METER, y * Y_TRANSLATE_PIX2METER, z * Z_TRANSLATE_PIX2METER]
    
    return aligned_poision


def SetGlobalValue(value):
    global test_global_value_
    test_global_value_ += value
    print(test_global_value_)

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


def GetOrientation(boundary):
    boundary_size = GetBoundarySize(boundary)
    
    pass

def VehicleSizeScaling(vehicle_info, boundary):
    boundary_size = GetBoundarySize(boundary)

    # pass the number from User
    length = vehicle_info[0]
    width = vehicle_info[1]
    height = vehicle_info[2]

    sorted_boundary_size = sorted(boundary_size)

    length_m = sorted_boundary_size[2]
    width_m = sorted_boundary_size[1]
    height_m = sorted_boundary_size[0]

    # the unit is in meter
    # make the car size to real size in meter
    vehicle_size = [length/length_m, width/width_m, height/height_m]

    return vehicle_size

def LidarSizeScaling():
    # the unit is in meter
    # make the car size to real size in meter
    width_m = 103.3/1000  # 75.6 mm to meter
    heigth_m = 75.6/1000
    length_m = 103.3/1000
    width_pix = 150
    heigth_pix = 105
    length_pix = 149.9766387939453

    lidar_size = [length_m/length_pix, width_m/width_pix, heigth_m/heigth_pix]

    return lidar_size

def LidarTranslatePixel2Meter(x, y, z):
    translate_x = x * X_PIX2METER
    translate_y = y * Y_PIX2METER
    translate_z = z * Z_PIX2METER

    pos = [translate_x, translate_y, translate_z]

    return pos

def TranslateCoordinateVehicle2origin(x, y, z):
    alignedPoision = [y, x, z]

    return alignedPoision


def RPY2Rotation(r, p, y):
    r_rotation = [r, 1, 0, 0]
    p_rotation = [p, 0, 1, 0]
    y_rotation = [y, 0, 0, 1]

    return r_rotation, p_rotation, y_rotation


def GetActors(lidar_info_list, vehicle_info):
    # -------------------------------------------create vehicle instances
    colors = vtk.vtkNamedColors()
    color_list = ['salmon', 'royalblue', 'lightblue', 'mediumorchid', 'mediumseagreen', 'gold', 'mediumslateblue', 'darkorange']

    # Vehicle Source
    vehicle_stl_reader = vtk.vtkSTLReader()
    vehicle_stl_reader.SetFileName('evoque_old.stl')
    # objreader = vtk.vtkOBJReader('evoque_old.obj')

    # Mapper
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(vehicle_stl_reader.GetOutputPort())
    # mapper.SetInputConnection(objreader.GetOutputPort())

    # Actor
    vehicle_actor = vtk.vtkActor()
    vehicle_actor.SetMapper(mapper)

    # Set VehicleActor Property
    vehicle_actor.GetProperty().SetDiffuse(0.8)
    vehicle_actor.GetProperty().SetDiffuseColor(colors.GetColor3d('silver'))
    vehicle_actor.GetProperty().SetSpecular(0.0)
    vehicle_actor.GetProperty().SetSpecularPower(60.0)
    vehicle_actor.GetProperty().SetOpacity(0.5)

    # Set Vehicle size and pose
    vehicle_transform = vtk.vtkTransform()

    # make real size unit
    boundary = vehicle_actor.GetBounds()
    scaled_vehicle_size = VehicleSizeScaling(vehicle_info, boundary)
    vehicle_transform.Scale(scaled_vehicle_size)

    # align coordinate
    # vehicle_transform.RotateWXYZ(-90,0,0,1)

    # move position to origin with center of gravity
    # alignedPoision = TranslateCoordinateVehicle2origin(3.75,0,0)
    # vehicle_transform.Translate(alignedPoision)
    # vehicle_actor.SetUserTransform(vehicle_transform)

    # -------------------------------------------create LiDAR instances
    # LiDAR Source
    lidar_vtk_reader = vtk.vtkSTLReader()
    lidar_vtk_reader.SetFileName('lidar.stl')

    # Mapper
    lidar_mapper = vtk.vtkPolyDataMapper()
    lidar_mapper.SetInputConnection(lidar_vtk_reader.GetOutputPort())

    # add lidar actors
    lidar_num = len(lidar_info_list)
    # print("number of LiDAR : " + str(lidar_num))
    lidar_names = []
    lidar_actors = []
    lidar_transforms = []

    for i in range(lidar_num):
        lidar_names.append("LiDAR " + str(i))

        # set actors
        lidar_actors.append(vtk.vtkActor())
        lidar_actors[i].SetMapper(lidar_mapper)
        lidar_actors[i].GetProperty().SetDiffuse(0.8)
        lidar_actors[i].GetProperty().SetDiffuseColor(colors.GetColor3d(color_list[i]))
        lidar_actors[i].GetProperty().SetSpecular(0.3)
        lidar_actors[i].GetProperty().SetSpecularPower(60.0)

        # set LiDAR Properties
        pose = lidar_info_list[i]
        pos = pose[:3]

        print("::::::::::Position of LiDAR::::::::::")
        print(pos)

        orientation = pose[3:]
        lidar_transforms.append(vtk.vtkTransform())
        # set size
        lidar_size = LidarSizeScaling()
        lidar_transforms[i].Scale(lidar_size)

        # set orientation
        r_rotation, p_rotation, y_rotation = RPY2Rotation(orientation[0], orientation[1], orientation[2])

        # set position
        aligned_pos = LidarTranslatePixel2Meter(pose[0], pose[1], pose[2])
        lidar_transforms[i].Translate(aligned_pos)

        lidar_transforms[i].RotateWXYZ(r_rotation[0], r_rotation[1], r_rotation[2], r_rotation[3])
        lidar_transforms[i].RotateWXYZ(p_rotation[0], p_rotation[1], p_rotation[2], p_rotation[3])
        lidar_transforms[i].RotateWXYZ(y_rotation[0], y_rotation[1], y_rotation[2], y_rotation[3])

        lidar_actors[i].SetUserTransform(lidar_transforms[i])

    # -------------------------------------------create renderer and renderer Windows

    # Create a rendering window and renderer
    ren = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)
    renWin.SetWindowName('ReadSTL')

    # Create a renderwindowinterasctor
    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    # -------------------------------------------create window instances for visualization

    actors = []
    actors.append(vehicle_actor)
    for actor in lidar_actors:
        actors.append(actor)

    return actors
    # # Assign actor to the renderer
    # ren.AddActor(vehicle_actor)
    # for i in range(lidar_num):
    #     ren.AddActor(lidar_actors[i])
    #     # print(lidar_actors[i].GetBounds())
    # ren.SetBackground(colors.GetColor3d('white'))


def CalibResult(lidar_info_list, vehicle_info):
    # -------------------------------------------create vehicle instances
    colors = vtk.vtkNamedColors()
    color_list = ['salmon', 'royalblue', 'lightblue', 'mediumorchid', 'mediumseagreen', 'gold', 'mediumslateblue', 'darkorange']

    # Vehicle Source
    vehicle_stl_reader = vtk.vtkSTLReader()
    vehicle_stl_reader.SetFileName('evoque_old_aligned.stl')
    # objreader = vtk.vtkOBJReader('evoque_old.obj')

    # Mapper
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(vehicle_stl_reader.GetOutputPort())
    # mapper.SetInputConnection(objreader.GetOutputPort())

    # Actor
    vehicle_actor = vtk.vtkActor()
    vehicle_actor.SetMapper(mapper)

    # Set VehicleActor Property
    vehicle_actor.GetProperty().SetDiffuse(0.8)
    vehicle_actor.GetProperty().SetDiffuseColor(colors.GetColor3d('silver'))
    vehicle_actor.GetProperty().SetSpecular(0.0)
    vehicle_actor.GetProperty().SetSpecularPower(60.0)
    vehicle_actor.GetProperty().SetOpacity(0.5)


    # youngrok add 2021/07/20
    # Change the order of trnasforming
    # rotation -> scaling -> translation
    # Added Vehicle rotation function
    # Added Vehicle Translation function in meter
    # Set Vehicle size and pose
    vehicle_transform = vtk.vtkTransform()

    vehicle_rotation = getRotation(vehicle_transform,vehicle_info_)
    vehicle_actor.SetUserTransform(vehicle_rotation)
    
    # make real size unit
    boundary = vehicle_actor.GetBounds()
    scaled_vehicle_size = VehicleSizeScaling(vehicle_info, boundary)
    vehicle_transform.Scale(scaled_vehicle_size)
    

    #move position to origin with center of gravity
    centerlize = moveCenterVehicleTranslation(vehicle_actor.GetBounds())
    aligned_poision = translateVehicle(centerlize[0],centerlize[1],centerlize[2])
    vehicle_transform.Translate(aligned_poision)
    #transformed_vehicle_boundary = vehicle_actor.GetBounds()
    #print("vehicle Size after Transformation : ", transformed_vehicle_boundary)


    # -------------------------------------------create LiDAR instances
    # LiDAR Source
    lidar_vtk_reader = vtk.vtkSTLReader()
    lidar_vtk_reader.SetFileName('lidar.stl')

    # Mapper
    lidar_mapper = vtk.vtkPolyDataMapper()
    lidar_mapper.SetInputConnection(lidar_vtk_reader.GetOutputPort())

    # add lidar actors
    lidar_num = len(lidar_info_list)
    # print("number of LiDAR : " + str(lidar_num))
    lidar_names = []
    lidar_actors = []
    lidar_transforms = []

    for i in range(lidar_num):
        lidar_names.append("LiDAR " + str(i))

        # set actors
        lidar_actors.append(vtk.vtkActor())
        lidar_actors[i].SetMapper(lidar_mapper)
        lidar_actors[i].GetProperty().SetDiffuse(0.8)
        lidar_actors[i].GetProperty().SetDiffuseColor(colors.GetColor3d(color_list[i]))
        lidar_actors[i].GetProperty().SetSpecular(0.3)
        lidar_actors[i].GetProperty().SetSpecularPower(60.0)

        # set LiDAR Properties
        pose = lidar_info_list[i]
        pos = pose[:3]

        print("::::::::::Position of LiDAR::::::::::")
        print(pos)

        orientation = pose[3:]
        lidar_transforms.append(vtk.vtkTransform())
        # set size
        lidar_size = LidarSizeScaling()
        lidar_transforms[i].Scale(lidar_size)

        # set orientation
        r_rotation, p_rotation, y_rotation = RPY2Rotation(orientation[0], orientation[1], orientation[2])

        # set position
        aligned_pos = LidarTranslatePixel2Meter(pose[0], pose[1], pose[2])
        lidar_transforms[i].Translate(aligned_pos)

        lidar_transforms[i].RotateWXYZ(r_rotation[0], r_rotation[1], r_rotation[2], r_rotation[3])
        lidar_transforms[i].RotateWXYZ(p_rotation[0], p_rotation[1], p_rotation[2], p_rotation[3])
        lidar_transforms[i].RotateWXYZ(y_rotation[0], y_rotation[1], y_rotation[2], y_rotation[3])

        lidar_actors[i].SetUserTransform(lidar_transforms[i])

    # -------------------------------------------create renderer and renderer Windows

    # Create a rendering window and renderer
    ren = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)
    renWin.SetWindowName('ReadSTL')

    # Create a renderwindowinterasctor
    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    # -------------------------------------------create coordinate axes instances

    # Get axis
    axes = vtk.vtkAxesActor()

    widget = vtk.vtkOrientationMarkerWidget()
    rgba = [0] * 4
    colors.GetColor('Carrot', rgba)
    widget.SetOutlineColor(rgba[0], rgba[1], rgba[2])
    widget.SetOrientationMarker(axes)
    widget.SetInteractor(iren)
    widget.SetViewport(0, 0, 0.2, 0.2)
    widget.SetEnabled(1)
    widget.InteractiveOn()

    # -------------------------------------------create window instances for visualization

    # Assign actor to the renderer
    ren.AddActor(vehicle_actor)
    for i in range(lidar_num):
        ren.AddActor(lidar_actors[i])
        # print(lidar_actors[i].GetBounds())
    ren.SetBackground(colors.GetColor3d('white'))

    # Enable user interface interactor
    iren.Initialize()
    renWin.Render()
    iren.Start()


if __name__ == '__main__':
    import sys

    # [x, y, z, roll, pitch, yaw]
    lidar_info_list = [[0,0,0,0,0,0],
                 [3.55,-0.19,0.65,0.42,0.99,0.48],
                 [3.55,0.19,0.69,0.75,-1.71,-0.52],
                 [1.24,0.56,1.67,-25,.64,0.82,-0.13],
                 [1.25,-0.0,1.7,-0.12,0.08,1.3],
                 [1.22,-0.56,1.67,25,0.46,-0.31]]
    

    CalibResult(lidar_info_list, vehicle_info_)
    CalibResult(lidar_info_list, vehicle_info_)

    '''
    sonatasize =        vehicleSizeWidth = 1.800
                        vehicleSizeHeigth = 1.440
                        vehicleSizeLength = 4.620

    evoque      =        vehicleSizeWidth = 1,904
                        vehicleSizeHeight = 1,649
                        vehicleSizeLength = 4,371


    [[1,0,0,0,0,0],
                 [0,0,0,0,0,0],
                 [1,0,0,0,0,0],
                 [0,0,0,0,0,0],
                 [1,0,0,0,0,0],
                 [0,0,0,0,0,0]]


                [[0,0,0,0,0,0],
                 [0,-0.03,0.65,0.42,0.99,0.48],
                 [0,-0.19,0.69,0.75,-1.71,-0.52],
                 [0,0.56,1.56,-25.64,0.82,-0.13],
                 [0,-0.06,1.7,-0.12,0.08,1.3],
                 [0,-0.83,1.57,4.58,0.46,-0.31]]
    '''
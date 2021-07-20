#!/usr/bin/env python
"""
@author : YoungRok Son
@author email: dudfhr3349@gmail.com
@author : Yeongsoo Kim
@author email: kimkys768@gmail.com
"""
import vtk
'''
#TODO 
1. Make same coordinate with vehicle and Lidar
- Make size to meter unit > lidar and vehicle
- Align direction with car and lidar
2. make multiple lidar according to dictionary data which is come from other module
- use for sentence

'''

# Length, Width, Height, x rotation, y rotation, z rotation
vehicle_info_ = [0., 0., 0., 0., 0., 0., 0., 0., 0.]
vehicle_stl_path_ = ''
lidar_stl_path_ = ''

X_PIX2METER = 1.0/0.0006887739372658
Y_PIX2METER = 1.0/0.0006887739372658
Z_PIX2METER = 100.0/0.07200000000000001

# Parameters for tranlsation in meter.
X_TRANSLATE_PIX2METER = 1.0/1.008755986325648 
Y_TRANSLATE_PIX2METER = 1.0/0.89637191416309
Z_TRANSLATE_PIX2METER = 1.0/1.007424376155878


# move vehicle to center
def moveCenterVehicleTranslation(boundary):
    centerlize_yaxis = -(boundary[2] + boundary[3]) / 2
    height2zero = -boundary[4]
    return [0, centerlize_yaxis, height2zero]

# Rotate vehicle according to the value of vehicle_info_[6,7,8]
def GetRotation(vehicleTransfromObject):
    global vehicle_info_
    vehicleTransfromObject.RotateX(vehicle_info_[6])
    vehicleTransfromObject.RotateY(vehicle_info_[7])
    vehicleTransfromObject.RotateZ(vehicle_info_[8])

    return vehicleTransfromObject

# Move vehicle in Meter.
def TranslateVehicle(x, y, z):  # Pixel to Meter
    aligned_poision = [x * X_TRANSLATE_PIX2METER, y * Y_TRANSLATE_PIX2METER, z * Z_TRANSLATE_PIX2METER]

    return aligned_poision

def SetVehicleStlPath(path):
    global vehicle_stl_path_
    vehicle_stl_path_ = path

def SetLidarStlPath(path):
    global lidar_stl_path_
    lidar_stl_path_ = path

def SetVehicleInfo(vehicle_info):
    global vehicle_info_
    vehicle_info_ = vehicle_info

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

def VehicleSizeScaling(boundary):
    boundary_size = GetBoundarySize(boundary)

    # pass the number from User
    global vehicle_info_
    length = vehicle_info_[0]
    width = vehicle_info_[1]
    height = vehicle_info_[2]

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


def GetActors(lidar_info_list):
    # -------------------------------------------create vehicle instances
    colors = vtk.vtkNamedColors()
    color_list = ['salmon', 'royalblue', 'lightblue', 'mediumorchid', 'mediumseagreen', 'gold', 'mediumslateblue', 'darkorange']

    # Vehicle Source
    vehicle_stl_reader = vtk.vtkSTLReader()
    global vehicle_stl_path_
    vehicle_stl_reader.SetFileName(vehicle_stl_path_)
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
    scaled_vehicle_size = VehicleSizeScaling(boundary)
    vehicle_transform.Scale(scaled_vehicle_size)

    #move position to origin with center of gravity
    #transformed_vehicle_boundary = vehicle_actor.GetBounds()

    # -------------------------------------------create LiDAR instances
    # LiDAR Source
    lidar_vtk_reader = vtk.vtkSTLReader()
    global lidar_stl_path_
    lidar_vtk_reader.SetFileName(lidar_stl_path_)

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

    actors = []
    actors.append(vehicle_actor)
    for actor in lidar_actors:
        actors.append(actor)

    return actors


def GetVehicleActor():
    colors = vtk.vtkNamedColors()

    # Vehicle Source
    vehicle_stl_reader = vtk.vtkSTLReader()
    global vehicle_stl_path_
    vehicle_stl_reader.SetFileName(vehicle_stl_path_)
    # objreader = vtk.vtkOBJReader('evoque_old.obj')

    # Mapper
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(vehicle_stl_reader.GetOutputPort())
    # mapper.SetInputConnection(objreader.GetOutputPort())

    # Actor
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    # Set VehicleActor Property
    actor.GetProperty().SetDiffuse(0.8)
    actor.GetProperty().SetDiffuseColor(colors.GetColor3d('silver'))
    actor.GetProperty().SetSpecular(0.0)
    actor.GetProperty().SetSpecularPower(60.0)
    actor.GetProperty().SetOpacity(0.5)

    # Set Vehicle size and pose
    transformation = vtk.vtkTransform()

    rotation = GetRotation(transformation)
    actor.SetUserTransform(rotation)

    # make real size unit
    boundary = actor.GetBounds()
    scaled_vehicle_size = VehicleSizeScaling(boundary)
    transformation.Scale(scaled_vehicle_size)

    #move position to origin with center of gravity
    centerlize = moveCenterVehicleTranslation(actor.GetBounds())
    aligned_poision = TranslateVehicle(centerlize[0], centerlize[1], centerlize[2])
    transformation.Translate(aligned_poision)
    #transformed_vehicle_boundary = actor.GetBounds()

    return actor

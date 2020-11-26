# -*- coding: utf-8 -*-
"""
@author: chansoo7857@gmail.com
"""

import os  # os에서 제공되는 여러 기능 제공
import pickle  # 텍스트 이외의 자료형을 파일로 저장할 수 있는 모듈 제공
import tkinter as tk  # GUI 툴킷에 대한 표준 파이썬 인터페이스
import tkinter.filedialog  # file 경로 설정
import struct  # 데이터 변환 시 사용
import numpy as np  # 수치계산 (벡터 및 행렬 연산)에 사용
import pandas as pd  # dataframe을 다


# %% Select path
def select_path(title, file_name='prev_logging_path.pickle'):
    # Remove tk windows
    root = tk.Tk()
    root.withdraw()

    # Load pickle
    pickle_logging_path = os.path.dirname(os.path.abspath(__file__)) + '/' + file_name
    try:
        with open(pickle_logging_path, 'rb') as f:
            pickle_for_load = pickle.load(f)

        # Get file path
        options = {}
        options['initialdir'] = pickle_for_load['logging_path']
        options['title'] = title
        logging_path = tkinter.filedialog.askdirectory(**options)

    except:
        print('Fail to read the previous logging path')

        # Get file path
        options = {}
        options['title'] = title
        logging_path = tkinter.filedialog.askdirectory(**options)

    # save pickle
    pickle_for_save = {'logging_path': logging_path}
    with open(pickle_logging_path, 'wb') as f:
        pickle.dump(pickle_for_save, f)

    # return
    return logging_path


def select_file(title, file_name='prev_selected_file.pickle'):
    # Remove tk windows
    root = tk.Tk()
    root.withdraw()

    # Load pickle
    pickle_selected_file = os.path.dirname(os.path.abspath(__file__)) + '/' + file_name
    try:
        with open(pickle_selected_file, 'rb') as f:
            pickle_for_load = pickle.load(f)

        # Get file path
        options = {}
        options['initialdir'] = pickle_for_load['selected_file']
        options['title'] = title
        selected_file = tkinter.filedialog.askopenfilename(**options)

    except:
        print('Fail to read the previous selected file')

        # Get file path
        options = {}
        options['title'] = title
        selected_file = tkinter.filedialog.askopenfilename(**options)

    # save pickle
    pickle_for_save = {'selected_file': selected_file}
    with open(pickle_selected_file, 'wb') as f:
        pickle.dump(pickle_for_save, f)

    # return
    return selected_file


# %% Parse csv file
def parse_csv(file, skip_line=1):
    List = []
    with open(file) as f:
        # Skip lines
        for i in range(1, skip_line + 1):
            line = f.readline()

            # Read the file until the end of the file
        while True:
            # Read one lines
            line = f.readline()

            # Check the end of the file
            if not line: break

            # Parse data
            strLine = line.split(",")

            # Convert data
            fLine = list(map(float, strLine))

            # Parse data
            List.append(fLine)

    # Return Gnss List
    return List

# %% Parse information
# Parse Motion.csv
def parse_motion_csv_np(file, skip_line=1):
    # Read data
    data_list = np.array(parse_csv(file, skip_line))

    # Convert timestamp from micro second to second
    data_list[:, 2] = data_list[:, 2] / 1e6

    # return
    return data_list

# Parse Gnss.csv
def parse_motion_csv_df(file, usecols):
    df_motion = pd.read_csv(file, usecols=usecols)
    if 'timestamp' in df_motion.columns:
        df_motion['timestamp'] = df_motion['timestamp'] / 1e6
    return df_motion


def parse_gnss_csv_np(file, skip_line=1):
    # Read data
    data_list = np.array(parse_csv(file, skip_line))

    # Convert timestamp from micro second to second
    data_list[:, 2] = data_list[:, 2] / 1e6

    # return
    return data_list

def parse_gnss_csv_df(file, usecols):
    df_gnss = pd.read_csv(file, usecols=usecols)
    if 'timestamp' in df_gnss.columns:
        df_gnss['timestamp'] = df_gnss['timestamp'] / 1e6
    return df_gnss

# Parse XYZRGB.bin
def parse_pointcloud_bin_np(file):
    data_list = []
    # Open file
    with open(file, "rb") as f:
        while True:
            # read
            tmp_timestamp = f.read(8)
            if tmp_timestamp == b'': break

            # Define row
            row = []
            # Get timestamp
            timestamp = struct.unpack("<Q", tmp_timestamp)[0]
            row.append(timestamp)

            # Get number of points
            num_point = struct.unpack("<i", f.read(4))[0]
            row.append(num_point)

            ## Get current pointer
            row.append(f.tell())

            # Add row
            data_list.append(row)

            ## Jump to next point
            f.seek(num_point * 8 * 6, 1)

    return data_list

# Parse XYZRGB.bin
def parse_pointcloud_bin_df(file):
    ArrayPointCloud = np.array(parse_pointcloud_bin_np(file))
    df_pointcloud = pd.DataFrame(ArrayPointCloud, columns=['timestamp', 'num_points', 'file_pointer'])
    df_pointcloud['timestamp'] = df_pointcloud['timestamp'] / 1e6
    return df_pointcloud

# Get point cloud from binary file
def get_point_cloud(file, num_point, file_pointer):
    # open binary file
    with open(file, "rb") as f:
        # go to file pointer
        f.seek(file_pointer)

        # parse point cloud data
        strFormat = "<" + str(num_point * 6) + "d"
        array1d = np.array(struct.unpack(strFormat, f.read(num_point * 8 * 6)))

        # format shape
        pointcloud = np.reshape(array1d, (num_point, -1))
        return pointcloud

def normalized_angle_rad(input_rad):
    tmp_input_rad = input_rad
    while (tmp_input_rad > np.pi):
        tmp_input_rad = tmp_input_rad - 2 * np.pi
    while (tmp_input_rad < -np.pi):
        tmp_input_rad = tmp_input_rad + 2 * np.pi

    normalized_rad = tmp_input_rad
    return normalized_rad

def normalized_angle_deg(input_deg):
    tmp_input_deg = input_deg
    while (tmp_input_deg > 180):
        tmp_input_deg = tmp_input_deg - 360
    while (tmp_input_deg < -180):
        tmp_input_deg = tmp_input_deg + 360

    normalized_deg = tmp_input_deg
    return normalized_deg
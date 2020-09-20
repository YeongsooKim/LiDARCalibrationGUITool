 # -*- coding: utf-8 -*-
"""
@author: chansoo7857@gmail.com
@date: 2018-07-12
@version: 0.0.1
"""
##############################################################################################################################
#%% Import Libraries
##############################################################################################################################

# Basic modules in Anaconda
import numpy as np    # 행렬이나 대규모 다차원 배열 처리
import pandas as pd   # 데이터 조작 및 분석
import matplotlib.pyplot as plt  # matlab과 유사한 그래프 표시 
from scipy.optimize import basinhopping
import configparser   # Configure file을 생성하고 생성된 configure file을 읽어내는데 사용
from sympy import *

# Additional modules
from tqdm import tqdm   # progress bar 생성, 진행상황 피드백
# import sys   # 시스템 경로
# sys.path.append("D:\\OneDrive\\konkuk.ac.kr\\자동차지능연구실 - AILabDrive\\02_Researcher\\2020_1학기_김가민\\2020_1학기\\[InnerProject]\\Online Calibration between LiDAR-Vehicle\\python")   # 시스템 경로 추가
#sys.path.append("C:\\Users\\gam\\konkuk.ac.kr\\자동차지능연구실 - AILabDrive\\02_Researcher\\2020_1학기_김가민\\2020_1학기\\[InnerProject]\\Online Calibration between LiDAR-Vehicle\python")
# User defined modules
#import numdifftools
#from numdifftools import Jacobian, Hessian
#import scipy
#from scipy import optimize


class Evaluation:
    def __init__(self, config, import_data, handeye, optimization):
        self.config = config
        self.import_data = import_data
        self.handeye = handeye
        self.optimization = optimization

        # Parameter
        self.handeye_calibration_param = {}
        self.optimization_calibration_param = {}

    def GetResult(self):

        pass

    def Plot(self):
        pass

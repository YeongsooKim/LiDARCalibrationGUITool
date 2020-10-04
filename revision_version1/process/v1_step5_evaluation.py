 # -*- coding: utf-8 -*-
"""
@author: chansoo7857@gmail.com
@date: 2018-07-12
@version: 0.0.1
"""
##############################################################################################################################
#%% Import Libraries
##############################################################################################################################

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

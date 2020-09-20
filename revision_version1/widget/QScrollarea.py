from PyQt5.QtWidgets import *
from PyQt5.QtCore import QDate, QSize, Qt
from PyQt5.QtGui import QPixmap
from widget import DoubleSlider
from widget import QThread
from PyQt5.QtCore import pyqtSignal
from process import get_result
import matplotlib.pyplot as plt
import numpy as np

# class ScrollArea(QWidget):
#     def __init__(self):
#         super().__init__()
#
#         self.InitUi()
#
#     def InitUi(self):
#         self.scroll_widget = QWidget()
#         self.scrollarea = QScrollArea(self.scroll_widget)
#         self.scrollarea.setWidgetResizable(True)
#
#         self.new_widget = QWidget()
#         self.scrollarea.setWidget(self.new_widget)
#         self.layout = self(self.scrollarea)
#
#
#         # self.layout()
#         # self.addWidget(self.scrollarea)


class ScrollArea(QScrollArea):
    def __init__(self, parent=None):
        super(ScrollArea, self).__init__(parent)
        self.setWidgetResizable(True)
        # self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        # self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOn)

        self.layout = QGridLayout()
        # self.gridbox = QGridLayout()
        # self.layout.setSpacing(30)

        scroll = QWidget()
        scroll.setLayout(self.layout)
        self.setWidget(scroll)
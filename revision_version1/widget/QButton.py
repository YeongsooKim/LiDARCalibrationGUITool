from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

CONST_ICON_WIDTH = 50
CONST_ICON_HEIGHT = 50
CONST_BUTTON_WIDTH = 110
CONST_BUTTON_HEIGHT = 80

CONST_GREEN = 0
CONST_RED = 1
CONST_LIDAR = 0
CONST_GNSS = 1

class Button(QToolButton):
    green_lidar_icon = 'green_lidar.ico'
    red_lidar_icon = 'red_lidar.ico'
    green_gnss_icon = 'green_gnss.ico'
    red_gnss_icon = 'red_gnss.ico'
    def __init__(self, text, color, icon, path, parent=None):
        super(Button, self).__init__(parent)
        if color is CONST_GREEN:
            if icon is CONST_LIDAR:
                icon = QIcon(path + self.green_lidar_icon)
            elif icon is CONST_GNSS:
                icon = QIcon(path + self.green_gnss_icon)
        elif color is CONST_RED:
            if icon is CONST_LIDAR:
                icon = QIcon(path + self.red_lidar_icon)
            elif icon is CONST_GNSS:
                icon = QIcon(path + self.red_gnss_icon)
        self.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        self.setText(text)
        self.setIcon(icon)
        self.setIconSize(QSize(CONST_ICON_WIDTH, CONST_ICON_HEIGHT))
        self.setFixedSize(CONST_BUTTON_WIDTH, CONST_BUTTON_HEIGHT)
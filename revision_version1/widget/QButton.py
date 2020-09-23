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
    def __init__(self, text, color, icon, parent=None):
        super(Button, self).__init__(parent)
        if color is CONST_GREEN:
            if icon is CONST_LIDAR:
                icon = QIcon('image/green_lidar.ico')
            elif icon is CONST_GNSS:
                icon = QIcon('image/green_gnss.ico')
        elif color is CONST_RED:
            if icon is CONST_LIDAR:
                icon = QIcon('image/red_lidar.ico')
            elif icon is CONST_GNSS:
                icon = QIcon('image/red_gnss.ico')
        self.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        self.setText(text)
        self.setIcon(icon)
        self.setIconSize(QSize(CONST_ICON_WIDTH, CONST_ICON_HEIGHT))
        self.setFixedSize(CONST_BUTTON_WIDTH, CONST_BUTTON_HEIGHT)
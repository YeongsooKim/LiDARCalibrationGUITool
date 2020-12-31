from PyQt5.QtWidgets import *

class ScrollAreaV(QScrollArea):
    def __init__(self, parent=None):
        super(ScrollAreaV, self).__init__(parent)
        self.setWidgetResizable(True)
        self.layout = QVBoxLayout()

        self.scroll = QWidget()
        self.scroll.setLayout(self.layout)
        self.setWidget(self.scroll)

class ScrollAreaH(QScrollArea):
    def __init__(self, parent=None):
        super(ScrollAreaH, self).__init__(parent)
        self.setWidgetResizable(True)
        self.layout = QHBoxLayout()

        self.scroll = QWidget()
        self.scroll.setLayout(self.layout)
        self.setWidget(self.scroll)
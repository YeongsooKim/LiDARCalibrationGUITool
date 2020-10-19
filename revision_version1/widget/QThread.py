from PyQt5.QtCore import Qt
import numpy as np
from process import utils_file

from PyQt5.QtCore import QThread
from PyQt5.QtCore import QWaitCondition
from PyQt5.QtCore import QMutex
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot

class Thread(QThread):
    emit_string = pyqtSignal(str)
    change_value = pyqtSignal(int)
    interation_percentage = pyqtSignal(dict)
    end = pyqtSignal()

    def __init__(self):
        QThread.__init__(self)
        self.cond = QWaitCondition()
        self.mutex = QMutex()
        self._status = True
        self.error = False

    def __del__(self):
        self.wait()

    def run(self):
        if self.args is None:
            self.target(self)
        else:
            self.target(self, self.args)
        self.end.emit()

    def SetFunc(self, target, args=None):
        self.target = target
        self.args = args

    def toggle_status(self):
        self._status = not self._status
        if self._status:
            self.cond.wakeAll()

    def Stop(self):
        self.terminate()

    @property
    def status(self):
        return self._status

    def Err(self, err):
        self.error = err
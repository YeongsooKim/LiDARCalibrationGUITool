from PyQt5.QtWidgets import QDoubleSpinBox
from gui_partition.widgets.widget_id import *

class QUniqDoubleSpinBox(QDoubleSpinBox):
    '''
    Class generate Layout included uniques id
    '''
    def __init__(self, id, form_widget, parent=None):
        super(QUniqDoubleSpinBox, self).__init__(parent)
        self.id = id
        self.form_widget = form_widget
        self.editingFinished.connect(self.ValueChanged)

    def ValueChanged(self):
        if self.id == CONST_IMPORT_GNSS_EAST:
            init = [self.value(), self.form_widget.importing.init[1], self.form_widget.importing.init[2]]
            self.form_widget.importing.ChangeInitGnss(init)
        elif self.id == CONST_IMPORT_GNSS_NORTH:
            init = [self.form_widget.importing.init[0], self.value(), self.form_widget.importing.init[2]]
            self.form_widget.importing.ChangeInitGnss(init)
        elif self.id == CONST_IMPORT_GNSS_HEADING:
            init = [self.form_widget.importing.init[0], self.form_widget.importing.init[1], self.value()]
            self.form_widget.importing.ChangeInitGnss(init)

        elif self.id == CONST_IMPORT_MOITION_EAST:
            init = [self.value(), self.form_widget.importing.init[1], self.form_widget.importing.init[2]]
            self.form_widget.importing.ChangeInitMotion(init)
        elif self.id == CONST_IMPORT_MOITION_NORTH:
            init = [self.form_widget.importing.init[0], self.value(), self.form_widget.importing.init[2]]
            self.form_widget.importing.ChangeInitMotion(init)
        elif self.id == CONST_IMPORT_MOITION_HEADING:
            init = [self.form_widget.importing.init[0], self.form_widget.importing.init[1], self.value()]
            self.form_widget.importing.ChangeInitMotion(init)
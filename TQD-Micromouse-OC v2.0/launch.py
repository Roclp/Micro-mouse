from PySide6.QtWidgets import QApplication, QWidget
from PySide6 import QtCore, QtGui
import VS
import sys


if __name__ == '__main__':
    args = sys.argv
    app = QApplication()
    win = VS.Mydemo(args[1], args[2])
    win.show()
    sys.exit(app.exec())

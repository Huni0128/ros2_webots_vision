# pyqt_gui/widgets/image_view.py

from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QImage

class ImageView(QLabel):
    def __init__(self, width=640, height=480):
        super().__init__('Waiting for image…')
        self.setAlignment(Qt.AlignCenter)
        self._w = width
        self._h = height

    def update_image(self, qimg: QImage):
        pix = QPixmap.fromImage(qimg).scaled(
            self._w, self._h, Qt.KeepAspectRatio)
        self.setPixmap(pix)

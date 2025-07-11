from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QImage

class ImageView(QLabel):
    # 카메라 이미지 표시용 QLabel 위젯

    def __init__(self, width=640, height=480):
        # width, height: 표시할 이미지의 최대 크기
        super().__init__('Waiting for image…')
        self.setAlignment(Qt.AlignCenter)  # 텍스트/이미지 중앙 정렬
        self._w = width
        self._h = height

    def update_image(self, qimg: QImage):
        # QImage를 받아 지정된 크기로 스케일한 후 QLabel에 표시
        pix = QPixmap.fromImage(qimg).scaled(
            self._w, self._h, Qt.KeepAspectRatio
        )
        self.setPixmap(pix)

#include <QApplication>
#include <QSlider>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QIntValidator>

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  QWidget window;
  window.setWindowTitle("Panda GUI (Standalone)");

  QVBoxLayout *mainLayout = new QVBoxLayout();

  // 조인트 슬라이더 7개 생성
  for (int i = 0; i < 7; ++i) {
    QHBoxLayout *hLayout = new QHBoxLayout();
    QLabel *label = new QLabel(QString("Joint %1").arg(i + 1));

    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, 180);
    slider->setValue(90);

    QLineEdit *valueEdit = new QLineEdit(QString::number(90));
    valueEdit->setFixedWidth(50);
    valueEdit->setValidator(new QIntValidator(0, 180));  // 숫자 범위 제한

    // 슬라이더 → 입력창 업데이트
    QObject::connect(slider, &QSlider::valueChanged, [valueEdit](int value) {
      valueEdit->setText(QString::number(value));
    });

    // 입력창 → 슬라이더 업데이트
    QObject::connect(valueEdit, &QLineEdit::editingFinished, [slider, valueEdit]() {
      int value = valueEdit->text().toInt();
      slider->setValue(value);
    });

    hLayout->addWidget(label);
    hLayout->addWidget(slider);
    hLayout->addWidget(valueEdit);
    mainLayout->addLayout(hLayout);
  }

  // 그리퍼 버튼
  QHBoxLayout *gLayout = new QHBoxLayout();
  QPushButton *openBtn = new QPushButton("Open Gripper");
  QPushButton *closeBtn = new QPushButton("Close Gripper");

  gLayout->addWidget(openBtn);
  gLayout->addWidget(closeBtn);
  mainLayout->addLayout(gLayout);

  window.setLayout(mainLayout);
  window.resize(500, 400);
  window.show();

  return app.exec();
}

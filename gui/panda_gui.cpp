#include <QApplication>
#include <QSlider>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QIntValidator>
#include <array>
#include <fstream>
#include <nlohmann/json.hpp>
#include <iostream>
#include <algorithm>

// 슬라이더 값 기준 offset (초기 자세 기준 각도 값)
std::array<int, 7> joint_offset_deg = {0, 0, 0, 0, 0, 0, 0};

// 슬라이더 현재 값 (0~180)
std::array<int, 7> joint_slider_values = {90, 90, 90, 90, 90, 90, 90};

// 그리퍼 상태 ("open" or "close")
std::string gripper_state = "open";

// [1] 초기 자세 JSON 파일에서 offset_deg 불러오기
void load_initial_offsets() {
  std::ifstream file("/tmp/panda_initial_pose.json");
  if (!file) {
    std::cerr << "[WARN] Failed to load /tmp/panda_initial_pose.json\n";
    return;
  }
  try {
    nlohmann::json j;
    file >> j;
    auto offsets = j["offset_deg"];
    for (int i = 0; i < 7; ++i) {
      joint_offset_deg[i] = offsets[i].get<int>();
      joint_slider_values[i] = 90;  // 초기 슬라이더는 중앙값 고정
    }
    std::cout << "[INFO] Loaded offset_deg from JSON.\n";
  } catch (...) {
    std::cerr << "[WARN] Failed to parse offset_deg.\n";
  }
}

// [2] 현재 슬라이더 값 및 그리퍼 상태를 panda_command.json에 저장
void save_command() {
  nlohmann::json j;
  j["joints"] = joint_slider_values;
  j["gripper"] = gripper_state;
  std::ofstream file("/tmp/panda_command.json");
  file << j.dump(2);
}

// [3] 메인 함수 - Qt GUI 실행
int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  QWidget window;
  window.setWindowTitle("Panda GUI (Standalone)");

  load_initial_offsets();  // 초기자세 offset 불러오기

  QVBoxLayout *mainLayout = new QVBoxLayout();

  // [4] 관절 제어 슬라이더 & 입력창 구성
  for (int i = 0; i < 7; ++i) {
    QHBoxLayout *hLayout = new QHBoxLayout();
    QLabel *label = new QLabel(QString("Joint %1").arg(i + 1));

    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, 180);
    slider->setValue(joint_slider_values[i]);  // 중앙값으로 초기화

    // 실제 각도 계산: 슬라이더값 - 90 + 오프셋
    int actual_angle = joint_slider_values[i] - 90 + joint_offset_deg[i];
    QLineEdit *valueEdit = new QLineEdit(QString::number(actual_angle));
    valueEdit->setFixedWidth(50);
    valueEdit->setValidator(new QIntValidator(-180, 180));  // 입력값 제한

    // [4-1] 슬라이더 → 입력창 반영
    QObject::connect(slider, &QSlider::valueChanged, [i, valueEdit](int value) {
      joint_slider_values[i] = value;
      int actual_angle = value - 90 + joint_offset_deg[i];
      valueEdit->setText(QString::number(actual_angle));
      save_command();
    });

    // [4-2] 입력창 → 슬라이더 반영
    QObject::connect(valueEdit, &QLineEdit::editingFinished, [i, slider, valueEdit]() {
      int input_angle = valueEdit->text().toInt();
      int slider_val = input_angle - joint_offset_deg[i] + 90;
      slider_val = std::clamp(slider_val, 0, 180);  // 유효 범위 제한
      joint_slider_values[i] = slider_val;
      slider->setValue(slider_val);
      save_command();
    });

    hLayout->addWidget(label);
    hLayout->addWidget(slider);
    hLayout->addWidget(valueEdit);
    mainLayout->addLayout(hLayout);
  }

  // [5] 그리퍼 제어 버튼
  QHBoxLayout *gLayout = new QHBoxLayout();
  QPushButton *openBtn = new QPushButton("Open Gripper");
  QPushButton *closeBtn = new QPushButton("Close Gripper");

  QObject::connect(openBtn, &QPushButton::clicked, [] {
    gripper_state = "open";
    save_command();
  });
  QObject::connect(closeBtn, &QPushButton::clicked, [] {
    gripper_state = "close";
    save_command();
  });

  gLayout->addWidget(openBtn);
  gLayout->addWidget(closeBtn);
  mainLayout->addLayout(gLayout);

  window.setLayout(mainLayout);
  window.resize(500, 400);
  window.show();

  save_command();  // GUI 실행 전 초기 커맨드 파일 작성
  return app.exec();
}

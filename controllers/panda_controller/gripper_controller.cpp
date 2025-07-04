#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <iostream>
#include <cmath>  // for std::abs (절댓값 계산용)

using namespace webots;

class GripperController {
public:
  // 생성자: 로봇에서 그리퍼 조인트 2개(finger1, finger2)를 가져옴
  GripperController(Robot* robot) {
    finger1 = robot->getMotor("panda_finger_joint1");
    finger2 = robot->getMotor("panda_finger_joint2");

    if (!finger1 || !finger2) {
      std::cerr << "[ERROR] Failed to get gripper motors" << std::endl;
    } else {
      std::cout << "[INFO] Gripper motors successfully initialized." << std::endl;
    }
  }

  // 그리퍼 모터가 모두 정상적으로 연결되었는지 확인
  bool isAvailable() const {
    return finger1 && finger2;
  }

  // 그리퍼를 열기 (기본 위치 0.035 rad)
  void open() {
    if (!isAvailable()) return;

    const double open_pos = 0.035;

    // 현재 위치가 다를 경우에만 명령 전송
    if (std::abs(finger1->getTargetPosition() - open_pos) > 1e-4) {
      finger1->setPosition(open_pos);
      finger2->setPosition(open_pos);
      std::cout << "[INFO] Gripper opened" << std::endl;
    }
  }

  // 그리퍼를 닫기 (기본 위치 0.005 rad)
  void close() {
    if (!isAvailable()) return;

    const double close_pos = 0.005;

    // 현재 위치가 다를 경우에만 명령 전송
    if (std::abs(finger1->getTargetPosition() - close_pos) > 1e-4) {
      finger1->setPosition(close_pos);
      finger2->setPosition(close_pos);
      std::cout << "[INFO] Gripper closed" << std::endl;
    }
  }

private:
  // 그리퍼를 구성하는 두 개의 모터 포인터
  Motor* finger1 = nullptr;
  Motor* finger2 = nullptr;
};

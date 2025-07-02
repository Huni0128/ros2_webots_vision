// gripper_controller.cpp
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <iostream>

using namespace webots;

class GripperController {
public:
  GripperController(Robot* robot) {
    finger1 = robot->getMotor("panda_finger_joint1");
    finger2 = robot->getMotor("panda_finger_joint2");

    if (!finger1 || !finger2) {
      std::cerr << "[ERROR] Failed to get gripper motors" << std::endl;
    } else {
      std::cout << "[INFO] Gripper motors successfully initialized." << std::endl;
    }
  }

  bool isAvailable() const {
    return finger1 && finger2;
  }

  void open() {
    if (!isAvailable()) return;
    const double open_pos = 0.035;
    finger1->setPosition(open_pos);
    finger2->setPosition(open_pos);
    std::cout << "[INFO] Gripper opened to " << open_pos << " rad." << std::endl;
  }

  void close() {
    if (!isAvailable()) return;
    const double close_pos = 0.005;
    finger1->setPosition(close_pos);
    finger2->setPosition(close_pos);
    std::cout << "[INFO] Gripper closed to " << close_pos << " rad." << std::endl;
  }

private:
  Motor* finger1 = nullptr;
  Motor* finger2 = nullptr;
};

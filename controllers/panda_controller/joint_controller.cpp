#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>  // for std::abs, M_PI (라디안-도 변환에 필요)

using namespace webots;
using namespace std;

class JointController {
public:
  // 생성자: panda 로봇의 7개 관절 모터를 초기화
  JointController(Robot* robot) {
    vector<string> names = {
      "panda_joint1", "panda_joint2", "panda_joint3",
      "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
    };
    for (const auto& name : names) {
      Motor* motor = robot->getMotor(name);
      if (motor)
        motors.push_back(motor);  // 정상적으로 모터를 찾은 경우
      else {
        cerr << "[WARN] Motor not found: " << name << endl;
        motors.push_back(nullptr);  // 모터를 찾지 못한 경우 nullptr로 채움
      }
    }
  }

  // 현재 각 조인트의 목표 위치를 라디안 단위로 반환
  vector<double> getPositions() const {
    vector<double> result;
    for (const auto& motor : motors) {
      result.push_back(motor ? motor->getTargetPosition() : 0.0);  // 유효한 모터일 경우 위치 반환
    }
    return result;
  }

  // 단일 조인트에 위치 설정 (값이 바뀐 경우에만 degree로 출력)
  void setPosition(int index, double radian) {
    if (index >= 0 && index < static_cast<int>(motors.size()) && motors[index]) {
      double current = motors[index]->getTargetPosition();
      if (std::abs(current - radian) > 1e-4) {  // 현재 위치와 다를 경우에만 설정
        motors[index]->setPosition(radian);
        double deg = radian * 180.0 / M_PI;  // 라디안 → 도 변환
        cout << "[INFO] Joint " << (index + 1) << " set to " << deg << " deg." << endl;
      }
    }
  }

  // 여러 조인트를 한 번에 설정 (각 관절마다 바뀐 경우에만 degree로 출력)
  void setPositions(const vector<double>& radians) {
    for (size_t i = 0; i < motors.size() && i < radians.size(); ++i) {
      if (motors[i]) {
        double current = motors[i]->getTargetPosition();
        if (std::abs(current - radians[i]) > 1e-4) {  // 기존 값과 다를 때만 설정
          motors[i]->setPosition(radians[i]);
          double deg = radians[i] * 180.0 / M_PI;  // 라디안 → 도 변환
          cout << "[INFO] Joint " << (i + 1) << " set to " << deg << " deg." << endl;
        }
      }
    }
  }

private:
  vector<Motor*> motors;  // 7개 panda_joint를 관리할 Motor 포인터 배열
};

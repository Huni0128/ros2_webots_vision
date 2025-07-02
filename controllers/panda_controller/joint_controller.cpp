#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <vector>
#include <string>

using namespace webots;
using namespace std;

class JointController {
public:
  JointController(Robot* robot) {
    vector<string> names = {
      "panda_joint1", "panda_joint2", "panda_joint3",
      "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
    };
    for (const auto& name : names)
      motors.push_back(robot->getMotor(name));
  }

  void setInitialPose() {
    motors[0]->setPosition(0.3);  // 예시
  }

private:
  vector<Motor*> motors;
};

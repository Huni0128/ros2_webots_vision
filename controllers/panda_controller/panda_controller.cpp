#include <webots/Robot.hpp>
#define TIME_STEP 32

#include "camera_viewer.cpp"
#include "joint_controller.cpp"
#include "gripper_controller.cpp"

int main() {
  webots::Robot robot;

  JointController joint(&robot);
  CameraViewer cam(&robot, TIME_STEP);
  GripperController gripper(&robot);

  joint.setInitialPose();

  if (gripper.isAvailable()) {
    gripper.open();  // 초기 상태: 열기
  }

  int stepCount = 0;

  while (robot.step(TIME_STEP) != -1) {
    cam.process();

    if (gripper.isAvailable()) {
      if (stepCount == 100) {
        gripper.close();  // 100번 째 step에 닫기
      }
      if (stepCount == 300) {
        gripper.open();   // 다시 열기
      }
    }

    stepCount++;
  }

  return 0;
}

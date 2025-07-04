#include <webots/Robot.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include <cmath>
#include <iostream>
#include "camera_viewer.cpp"
#include "joint_controller.cpp"
#include "gripper_controller.cpp"

#define TIME_STEP 32  // Webots 시뮬레이션 타임스텝 (ms 단위)

int main() {
  webots::Robot robot;

  // 관절, 카메라, 그리퍼 컨트롤러 초기화
  JointController joint(&robot);
  CameraViewer cam(&robot, TIME_STEP);
  GripperController gripper(&robot);

  // [1] 초기 관절 각도 측정 및 저장 (deg 단위로 변환 후 JSON 저장)
  auto positions = joint.getPositions();  // 현재 관절 위치(rad)
  nlohmann::json init_json;
  for (size_t i = 0; i < positions.size(); ++i) {
    double deg = positions[i] * 180.0 / M_PI;  // 라디안 → 도(degree) 변환
    init_json["offset_deg"].push_back(static_cast<int>(std::round(deg)));
  }

  // [2] 초기 자세를 /tmp/panda_initial_pose.json 파일에 저장
  const std::string json_path = "/tmp/panda_initial_pose.json";
  std::ofstream init_file(json_path);
  if (init_file) {
    init_file << init_json.dump(2);  // JSON pretty print
    init_file.flush();
    init_file.close();
    std::cout << "[INFO] Saved initial pose to " << json_path << std::endl;
  } else {
    std::cerr << "[ERROR] Failed to write " << json_path << std::endl;
  }

  // [3] 시작 시 그리퍼 열기
  if (gripper.isAvailable()) gripper.open();

  // [4] 시뮬레이션 루프
  while (robot.step(TIME_STEP) != -1) {
    cam.process();  // 카메라 프레임 처리

    // [5] GUI에서 작성된 명령 JSON(panda_command.json) 불러오기
    std::ifstream file("/tmp/panda_command.json");
    if (file && file.peek() != std::ifstream::traits_type::eof()) {
      try {
        nlohmann::json j;
        file >> j;

        // [5-1] 조인트 제어 처리
        if (j.contains("joints")) {
          auto joint_values = j["joints"];  // 슬라이더 값: 0~180 범위
          std::ifstream offset_file(json_path);  // 초기 offset 값 불러오기
          nlohmann::json offset_json;
          offset_file >> offset_json;
          auto offsets = offset_json["offset_deg"];

          for (int i = 0; i < 7; ++i) {
            int slider_val = joint_values[i].get<int>();  // 현재 슬라이더 값
            int actual_deg = slider_val - 90 + offsets[i].get<int>();  // 실제 조인트 각도 계산
            double rad = actual_deg * M_PI / 180.0;  // 도 → 라디안
            joint.setPosition(i, rad);  // 조인트에 적용
          }
        }

        // [5-2] 그리퍼 제어 처리
        if (j.contains("gripper")) {
          std::string state = j["gripper"];
          if (state == "open") gripper.open();
          else if (state == "close") gripper.close();
        }

      } catch (...) {
        std::cerr << "[WARN] Failed to parse panda_command.json\n";
      }
    }
  }

  return 0;
}

# ROS2 Webots Vision Simulation

본 프로젝트는 **Webots** 시뮬레이터와 **ROS 2**를 활용하여 로봇 비전 기반의 픽 앤 플레이스(pick & place) 작업을 수행하는 데모입니다.
YOLOv8을 이용한 색상 박스 인식, 깊이 카메라를 활용한 3D 위치 추정, 프랑카 판다(Franka Panda) 로봇 암 제어, 그리고 PyQt 기반의 모니터링 GUI까지 하나의 환경에서 경험할 수 있습니다.

<sim_img>

## 주요 기능

- **객체 검출(Object Detection)**
  - `object_detector` 패키지에서 YOLOv8 모델을 사용해 빨강/파랑/초록 박스를 인식합니다.
  - 인식된 결과는 `/object_detector/detections` 토픽으로 퍼블리시되며, 주석이 달린 이미지는 `/camera/detected_objects`로 전송됩니다.

- **깊이 기반 위치 추정(Depth Estimation)**
  - `object_depth_estimator` 패키지가 2D 검출 결과와 Webots RangeFinder 이미지를 동기화하여 박스의 3D 좌표를 계산합니다.
  - 결과는 `/object_depth/pose_array` 토픽을 통해 전달됩니다.

- **픽 앤 플레이스 제어(Pick & Place)**
  - `pick_and_place` 패키지는 박스 색상 명령을 받아 로봇 관절을 스캔하며 목표를 찾고, 중심 정렬 후 집기 동작을 수행합니다.
  - TF2를 이용해 실제 물체 위치가 확인되면 스캔을 멈추고 정밀 제어를 진행합니다.

- **PyQt GUI**
  - `pyqt_gui` 패키지에서 카메라 영상 보기, 관절·그리퍼 상태 모니터링 및 수동 제어가 가능합니다.
  - 원하는 색상의 박스를 선택하여 `Pick` 버튼을 누르면 로봇이 자동으로 동작합니다.

<gui_img>

## 폴더 구조

```
controllers/         Webots용 C++ 컨트롤러 (카메라, 관절, 그리퍼 등)
protos/              Webots PROTO 파일들 (RedBox, BlueBox, GreenBox)
src/
├── object_detector/ YOLOv8 기반 박스 검출 노드 (Python)
├── object_depth_estimator/ 깊이 정보를 사용한 3D 포즈 추정 노드 (C++)
├── pick_and_place/ 로봇 제어 및 스캔 로직 (C++)
├── panda_description/ Panda 로봇 URDF 및 메시
└── pyqt_gui/        PyQt5 기반 모니터링 GUI
worlds/              Webots 시뮬레이션 월드 파일
```

## 설치 방법

1. ROS 2(Jazzy 추천)와 Webots가 사전에 설치되어 있어야 합니다.
2. 프로젝트 루트에서 다음 스크립트를 실행합니다.

```bash
./setup.sh
```

   - 필요한 APT 패키지, Python 패키지, ROS 의존성이 자동으로 설치되고 `colcon build`가 수행됩니다.
   - YOLO 모델(`src/object_detector/models/box_detect.pt`)과 학습 데이터셋은 용량 관계로 제공되지 않으므로 별도로 준비해 같은 경로에 위치시켜야 합니다.

## 실행 방법

1. Webots에서 월드 파일을 열어 시뮬레이션을 시작합니다.

```bash
webots worlds/ros2_webots_vision.wbt
```

2. 별도의 터미널에서 ROS 2 노드들을 실행합니다.

```bash
# 비전 파이프라인 (객체 검출 + 깊이 추정)
ros2 launch object_detector vision_pipeline.launch.py

# 픽 앤 플레이스 노드들
ros2 launch pick_and_place pick_and_place.launch.py

# PyQt GUI 실행
ros2 run pyqt_gui main
```

3. GUI에서 원하는 박스 색상을 선택한 뒤 `Pick` 버튼을 클릭하면 로봇이 목표 박스를 집어 올립니다.

<final_sim_img>

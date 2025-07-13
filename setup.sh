#!/bin/bash
set -e

# 사용할 ROS 배포판
ROS_DISTRO=${ROS_DISTRO:-jazzy}

# ROS 설치 확인
if [ ! -f /opt/ros/$ROS_DISTRO/setup.bash ]; then
    echo "ROS 2 배포판 '$ROS_DISTRO' 이(가) /opt/ros에 없습니다. 먼저 설치해 주세요."
    exit 1
fi

# 필수 패키지 설치
echo "필수 APT 패키지를 설치합니다..."
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-desktop \
    ros-$ROS_DISTRO-cv-bridge \
    python3-colcon-common-extensions \
    python3-pyqt5 \
    python3-opencv \
    python3-pytest \
    webots \
    python3-venv

# 가상환경 설정
VENV_DIR="./webots_venv"
if [ ! -d "$VENV_DIR" ]; then
    echo "가상환경(webots_venv)을 생성합니다..."
    python3 -m venv "$VENV_DIR"
fi

echo "가상환경(webots_venv)을 활성화합니다..."
source "$VENV_DIR/bin/activate"

# requirements.txt 설치
if [ -f "requirements.txt" ]; then
    echo "requirements.txt 기반으로 Python 패키지를 설치합니다..."
    pip install --upgrade pip
    pip install -r requirements.txt
else
    echo "requirements.txt 파일이 존재하지 않아 Python 패키지를 생략합니다."
fi

# YOLO 모델 안내
echo "YOLO 모델 로드 안내:"
echo "커스텀으로 학습한 모델 파일을 다음 경로에 복사하세요:"
echo "src/object_detector/models/box_detect.pt"
echo ""
echo "예시 데이터셋:"
echo "https://www.kaggle.com/datasets/udaysankarmukherjee/box-dataset"
echo "※ 필요 시 다운로드하여 src/object_detector/dataset/에 압축 해제하세요."

# rosdep 설정
if ! command -v rosdep >/dev/null; then
    echo "rosdep을 설치합니다..."
    sudo apt-get install -y python3-rosdep
fi

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "rosdep 초기화 중..."
    sudo rosdep init || true
fi

echo "rosdep 업데이트 중..."
rosdep update

# rosdep 사용자 정의 키 등록
CUSTOM_ROSDEP_FILE="$HOME/.ros/rosdep/user.yaml"
mkdir -p "$(dirname "$CUSTOM_ROSDEP_FILE")"
cat > "$CUSTOM_ROSDEP_FILE" <<EOL
pytest:
  ubuntu: [python3-pytest]

opencv:
  ubuntu: [python3-opencv]

pyqt5:
  ubuntu: [python3-pyqt5]
EOL

if ! grep -q "user.yaml" /etc/ros/rosdep/sources.list.d/20-default.list; then
    echo "yaml file://${CUSTOM_ROSDEP_FILE}" | sudo tee -a /etc/ros/rosdep/sources.list.d/20-default.list
fi

echo "ROS 의존성 설치 중..."
rosdep install --from-paths . --ignore-src -r -y

# ROS 빌드
echo "colcon 빌드 중..."
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build || { echo "colcon 빌드 실패"; exit 1; }

# 완료
echo -e "\n설치 완료!"
echo "환경 활성화:"
echo "source webots_venv/bin/activate"
echo "source install/setup.bash"

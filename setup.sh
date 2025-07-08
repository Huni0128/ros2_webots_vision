#!/bin/bash
set -e  # 오류 발생 시 스크립트 즉시 종료

# 사용할 ROS 배포판
ROS_DISTRO=${ROS_DISTRO:-jazzy}

# ROS가 설치되어 있는지 확인
if [ ! -f /opt/ros/$ROS_DISTRO/setup.bash ]; then
    echo "ROS 2 배포판 '$ROS_DISTRO' 이(가) /opt/ros에 없습니다. 먼저 설치해 주세요."
    exit 1
fi

# 필수 패키지 설치
echo "패키지 목록을 업데이트하고 필요한 패키지를 설치합니다..."
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-desktop \
    ros-$ROS_DISTRO-cv-bridge \
    python3-colcon-common-extensions \
    python3-pyqt5 \
    python3-opencv \
    python3-pytest \
    webots                          

# rosdep 설치 및 초기화
if ! command -v rosdep >/dev/null; then
    echo "rosdep을 설치합니다..."
    sudo apt-get install -y python3-rosdep
fi

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "rosdep을 초기화합니다..."
    sudo rosdep init || true
fi

# 사용자 정의 rosdep 키 정의
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

# 사용자 정의 rosdep 파일을 등록
if ! grep -q "user.yaml" /etc/ros/rosdep/sources.list.d/20-default.list; then
    echo "yaml file://${CUSTOM_ROSDEP_FILE}" | sudo tee -a /etc/ros/rosdep/sources.list.d/20-default.list
fi

# rosdep 정보 업데이트
echo "rosdep을 업데이트합니다..."
rosdep update

# 패키지 의존성 설치
echo "rosdep을 통해 의존성 패키지를 설치합니다..."
rosdep install --from-paths . --ignore-src -r -y

# 작업공간 빌드
echo "작업공간을 빌드합니다..."
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build || { echo "빌드 실패. 위 로그를 확인하세요."; exit 1; }

# 완료 메시지 출력
echo -e "\n완료되었습니다. 다음 명령어를 실행하여 환경을 활성화하세요:\nsource install/setup.bash"

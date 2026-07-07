#!/bin/bash
set -e

echo "=== ROS2 Humble 설치 스크립트 ==="
echo ""

# 1. Locale 설정
echo "[1/5] Locale 설정..."
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Universe repository 활성화
echo "[2/5] Universe repository 활성화..."
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

# 3. ROS2 GPG key 및 repository 추가
echo "[3/5] ROS2 repository 추가..."
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. ROS2 Humble 설치
echo "[4/5] ROS2 Humble Desktop 설치 (시간이 좀 걸립니다)..."
sudo apt update
sudo apt install -y ros-humble-desktop

# 5. 개발 도구 설치
echo "[5/5] 개발 도구 설치..."
sudo apt install -y ros-dev-tools python3-colcon-common-extensions python3-rosdep

# rosdep 초기화
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# bashrc에 source 추가
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS2 Humble" >> ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "bashrc에 ROS2 환경 설정 추가됨"
fi

echo ""
echo "=== ROS2 Humble 설치 완료! ==="
echo ""
echo "터미널을 새로 열거나 다음 명령을 실행하세요:"
echo "  source /opt/ros/humble/setup.bash"
echo ""
echo "설치 확인:"
echo "  ros2 --help"

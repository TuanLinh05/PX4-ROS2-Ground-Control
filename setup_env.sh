#!/bin_bash
# Script tự động cài đặt toàn bộ môi trường cho đồ án PX4 GCS trên máy Ubuntu 22.04 mới
# Tác giả: TuanLinh05
# Hướng dẫn: Chạy lệnh `bash setup_env.sh`

set -e # Dừng script nếu có lỗi xảy ra

echo "=========================================================="
echo "🚀 BẮT ĐẦU CÀI ĐẶT MÔI TRƯỜNG ROS 2 HUMBLE & PX4"
echo "=========================================================="

# 1. Cập nhật hệ thống và cài đặt tool cơ bản
echo "📦 1. Cài đặt công cụ cơ bản..."
sudo apt update && sudo apt upgrade -y
sudo apt install -y software-properties-common curl gnupg2 lsb-release git wget python3-pip cmake build-essential

# 2. Cài đặt ROS 2 Humble
echo "🐢 2. Cài đặt ROS 2 Humble..."
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 3. Cài đặt PX4 Autopilot
echo "✈️ 3. Tải và cài đặt PX4 Autopilot..."
cd ~
if [ ! -d "PX4-Autopilot" ]; then
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
fi
cd PX4-Autopilot
# Chạy script setup môi trường mặc định của PX4
bash ./Tools/setup/ubuntu.sh

# 4. Cài đặt Micro-XRCE-DDS-Agent
echo "📡 4. Tải và cài đặt Micro-XRCE-DDS-Agent..."
cd ~
if [ ! -d "Micro-XRCE-DDS-Agent" ]; then
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
fi
cd Micro-XRCE-DDS-Agent
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/

# 5. Cài đặt Python Dependencies cho Web GCS
echo "🌐 5. Cài đặt thư viện Python..."
pip3 install fastapi uvicorn[standard] websockets

echo "=========================================================="
echo "✅ CÀI ĐẶT HOÀN TẤT!"
echo "Vui lòng khởi động lại máy tính (hoặc WSL) để các thay đổi có hiệu lực."
echo "Sau khi khởi động lại, bạn có thể build và chạy dự án DoAn1."
echo "=========================================================="

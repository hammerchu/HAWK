#!/usr/bin/env bash
set -euo pipefail

# Detect Ubuntu and pick the Isaac Sim ROS2 distro
. /etc/os-release
case "${VERSION_ID}" in
  22.04) ROS_DISTRO=humble ;;
  24.04) ROS_DISTRO=jazzy ;;
  *) echo "Unsupported Ubuntu ${VERSION_ID}. Isaac Sim wants 22.04 (Humble) or 24.04 (Jazzy)."; exit 1 ;;
esac

sudo apt update
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt update
sudo apt install -y "ros-${ROS_DISTRO}-desktop" "ros-${ROS_DISTRO}-vision-msgs"

# so Isaac Sim + this publisher share the same ROS2
grep -q "source /opt/ros/${ROS_DISTRO}/setup.bash" ~/.bashrc \
  || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

echo "ROS2 ${ROS_DISTRO} installed. New terminal, then: source /opt/ros/${ROS_DISTRO}/setup.bash"
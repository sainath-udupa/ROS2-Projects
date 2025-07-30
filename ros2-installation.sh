nano install_ros2_humble.sh

#!/bin/bash
#ROS2 Humble Installation Script for Ubuntu 22.04
set -e

#1.Set Locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

#2.Enable Required Repositories
sudo apt install -y software-properties-common
sudo add-apt-repository universe

#3.Add ROS2 GPG Key and Repository
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

#4.Install ROS2 Humble Desktop
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop

#5.Source ROS2 in bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

#6.Install Colcon
sudo apt install -y python3-colcon-common-extensions
echo "ROS2 Humble Installation Completed Successfully!"

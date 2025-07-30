#create workspace
mkdir -p ~/ros2_ws/src 
cd ~/ros2_ws

#initialize colcon workspace
colcon build
source install/setup.bash

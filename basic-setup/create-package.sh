#python package (creates a Python ROS2 package named `my_py_node` with dependencies on `rclpy` and `std_msgs`)
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_py_node --dependencies rclpy std_msgs

#c++ package (creates a C++ ROS2 package named `my_cpp_node`)
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_cpp_node --dependencies rclcpp std_msgs


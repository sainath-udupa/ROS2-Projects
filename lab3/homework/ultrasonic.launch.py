from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #launch the publisher node
        Node(
            package='janyu_lab3_ros2',          #replace that `lab2_demo` with your actual package name
            executable='publisher',       #name of the publisher executable
            name='robot_publisher'        #optional node name
        ),

        #launch the subscriber node
        Node(
            package='janyu_lab3_ros2',          #replace that `lab2_demo` with your actual package name
            executable='subscriber',      #name of the subscriber executable
            name='robot_subscriber'       #optional node name
        )
    ])

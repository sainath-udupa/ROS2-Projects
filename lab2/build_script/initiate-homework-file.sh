#this is the script for creating a new package and within the same package creating a new node
#make sure you have ROS2-Humble installed in your Ubuntu-22.04

#Step-1 : create a ROS2 Workspace as `src` folder:
cd ~/ros2_ws/src

#Step-2: create a package with your own name 
ros2 pkg create --build-type ament_python your_name

#Step-3 : move into that package
cd your_name/your_name

#Step-4 : create a python node named as mahe_janyu
nano mahe_janyu.py

#Step-5 : after this you'll have to paste the code in the Homework Folder of this repository (Lab2 Folder)
#make sure the code is copy-pasted properly and it should not have any error (double-check)

#Step-6 : make it properly executable
chmod +x mahe_janyu.py

#Step-7 : update your setup.py file 
cd ~/ros2_ws/src/aayush_ros2
nano setup.py
#this will open a already setup file in which you'll have to edit your node name in the console script

entry_points={
    'console_scripts': [
        'mahe_janyu = your_package_name.mahe_janyu:main',
    ],
},

#Step-8 : build the package now using `colcon`
cd ~/ros2_ws
colcon build
source install/setup.bash

#Step-9 : final step for the build, all you have to do is run the whole package
ros2 run your_package_name mahe_janyu


#we will initially create a workspace (basically making a directory) and then move onto creating a package and then a node file in that package
mkdir -p ~/your_name_ws/src
cd ~/your_name_ws

#initialise the workspace 
colcon build 
#source the workspace
source install/setup.bash

#create a package inside the workspace
cd ~/your_name_ws/src
ros2 pkg create --build-type ament_python your_package_name

#create a node file inside the package 
cd ~/your_name_ws/src/your_package_name
touch publisher.py
chmod +x publisher.py
nano publisher.py 
#the `nano` command is not needed if you're using vs-code for writing the whole code by yourself
#however if you're just copy-pasting the code from teams then you can use this command instead.

#edit the node file 
#for closing the node file, use Ctrl + O -> Enter -> Ctrl + X
#back to the terminal, then you need to edit the setup.py

cd ~/your_name_ws/src/your_package_name
nano setup.py 
#again here, the `nano` command is not needed if you're using vs-code for writing the whole code by yourself
#however if you're just copy-pasting the code from teams then you can use this command instead.

#edit the setup.py file and then use, Ctrl + O -> Enter -> Ctrl + X
#back to the terminal, now run the `publisher.py` node file

cd ~/your_name_ws
colcon build
source install/setup.bash
ros2 run your_package_name publisher


#!/bin/bash

echo ""
echo "[Note] Target OS version  >>> Ubuntu 20.04.x (Focal Fossa) or Linux Mint 21.x"
echo "[Note] Target ROS version >>> ROS Noetic Ninjemys"
echo "[Note] Catkin workspace   >>> $HOME/catkin_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read 

name_ros_version=${name_ros_version:="noetic"}
name_catkin_workspace=${name_catkin_workspace:="catkin_ws"}

echo "[Make the catkin workspace and test the catkin_make]"
sudo apt remove ros-noetic-turtlebot3-msgs 
sudo apt remove ros-noetic-turtlebot3
mkdir -p $HOME/$name_catkin_workspace/src
cd $HOME/$name_catkin_workspace/src
git clone -b https://github.com/zhl017/turtlebot3_fet
git clone https://github.com/zhl017/turtlebot3_msgs_idm_custom
cd $HOME/$name_catkin_workspace
catkin_make

#!/bin/bash

echo "Creating the Workspace"
# MoveIt2
mkdir -p ./ros-humble-sim_workspace/ws_moveit2/src
git clone -b humble https://github.com/ros-planning/moveit2_tutorials ./ros-humble-sim_workspace/ws_moveit2/src/moveit2_tutorials/
# Rviz2 11.2.18
mkdir -p ./ros-humble-sim_workspace/ws_rviz2/src
git clone -b humble https://github.com/ros2/rviz ./ros-humble-sim_workspace/ws_rviz2/src/rviz
cd ./ros-humble-sim_workspace/ws_rviz2/src/rviz
git checkout 11.2.18
cd ../../../../
# Universal Robots Repositories
mkdir -p ./ros-humble-sim_workspace/ws_ur_ext/src
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description ./ros-humble-sim_workspace/ws_ur_ext/src/Universal_Robots_ROS2_Description
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation ./ros-humble-sim_workspace/ws_ur_ext/src/Universal_Robots_ROS2_Gazebo_Simulation
# Robotiq Gripper Repository
mkdir -p ./ros-humble-sim_workspace/ws_robotiq_gripper/src
git clone -b humble https://github.com/PickNikRobotics/ros2_robotiq_gripper ./ros-humble-sim_workspace/ws_robotiq_gripper/src/ros2_robotiq_gripper
## Remove unused packages
rm -rf ./ros-humble-sim_workspace/ws_robotiq_gripper/src/ros2_robotiq_gripper/robotiq_driver/
rm -rf ./ros-humble-sim_workspace/ws_robotiq_gripper/src/ros2_robotiq_gripper/robotiq_hardware_tests/
# Path Predictor Workspace
mkdir ./path-predictor_workspace
# Common Workspace
mkdir -p ./common_workspace/image_processing
mkdir -p ./common_workspace/trajectory_planning
# X11 desktop sock
mkdir -p ./utils/novnc_server/desktop-x11
touch ./utils/novnc_server/desktop-x11/.Xauthority

# VNC Password
read -p "Enter your noVNC Password, it has to be at least 6 characters long and alphanumeric only: " vnc_password
pass_length=${#vnc_password}
while [[ "$pass_length" -lt 6 ]] || [[ ! $vnc_password =~ ^[A-Za-z0-9]+$ ]];
do
    read -p "The password has to be at least 6 characters long and alphanumeric only, please input another one: " vnc_password
    pass_length=${#vnc_password}
done
echo The noVNC password is: $vnc_password
echo $vnc_password > ./utils/novnc_server/scripts/.vncpass
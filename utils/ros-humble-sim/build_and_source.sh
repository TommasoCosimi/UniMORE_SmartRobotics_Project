#!/bin/bash

# MoveIt2
cd /ros_workspace/ws_moveit2
colcon build --mixin release
source /ros_workspace/ws_moveit2/install/setup.bash
# Rviz2
cd /ros_workspace/ws_rviz2
colcon build --merge-install
source /ros_workspace/ws_rviz2/install/setup.bash
# Universal Robots
cd /ros_workspace/ws_ur_ext
colcon build
source /ros_workspace/ws_ur_ext/install/setup.bash
# Robotiq Gripper
cd /ros_workspace/ws_robotiq_gripper
colcon build
source /ros_workspace/ws_robotiq_gripper/install/setup.bash
# Project
cd /ros_workspace/ws_ur
colcon build
source /ros_workspace/ws_ur/install/setup.bash
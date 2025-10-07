#/bin/bash

# Installing all of the needed dependencies
## Place the temporary files of this operation in a specific folder in the home directory of the container
mkdir $HOME/deps_install
cd $HOME/deps_install
## Update the Package list
sudo apt-get update
sudo apt-get -y full-upgrade
## MoveIt2
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
cd /ros_workspace/ws_moveit2/src
vcs import < ./moveit2_tutorials/moveit2_tutorials.repos
rosdep install -r --from-paths /ros_workspace/ws_moveit2/src/ --ignore-src --rosdistro $ROS_DISTRO -y
cd $HOME/deps_install
## Rviz2
### Note: the source compiled 11.2.18 version of Rviz2 will be used to ensure compatibility with the MoveIt2 Setup Assistant 
rosdep install -r --from-paths /ros_workspace/ws_rviz2/src/ --ignore-src --rosdistro $ROS_DISTRO -y
## Universal Robots
rosdep install -r --from-paths /ros_workspace/ws_ur_ext/src/Universal_Robots_ROS2_Description --ignore-src --rosdistro $ROS_DISTRO -y
rosdep install -r --from-paths /ros_workspace/ws_ur_ext/src/Universal_Robots_ROS2_Gazebo_Simulation --ignore-src --rosdistro $ROS_DISTRO -y
## Robotiq Gripper
rosdep install -r --from-paths /ros_workspace/ws_robotiq_gripper/src/ros2_robotiq_gripper --ignore-src --rosdistro $ROS_DISTRO -y
## Project
rosdep install -r --from-paths /ros_workspace/ws_ur/src/ur_gripper_sim --ignore-src --rosdistro $ROS_DISTRO -y
rosdep install -r --from-paths /ros_workspace/ws_ur/src/image_processing --ignore-src --rosdistro $ROS_DISTRO -y
### Distro packages
sudo apt-get -y install \
    ros-humble-tf-transformations

# Compile the workspaces
## MoveIt2
cd /ros_workspace/ws_moveit2
colcon build --mixin release
## Rviz2
cd /ros_workspace/ws_rviz2
colcon build --merge-install
## Universal Robots
cd /ros_workspace/ws_ur_ext
colcon build
## Robotiq Gripper
cd /ros_workspace/ws_robotiq_gripper
colcon build
## Project
cd /ros_workspace/ws_ur
colcon build

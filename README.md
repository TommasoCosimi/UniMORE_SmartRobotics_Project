# UniMORE_SmartRobotics_Project

## Project Structure
The project aims to simulate a controllable Universal Robot UR5 in Gazebo and Rviz.

To achieve this, three main containers are used:
- **ROS Container** to simulate the robot itself using ROS2 Humble  
- **Python Container** to implement an object detector and edge detector which are used to describe a trajectory
- **noVNC Container** to be able to view and interact with the GUI components of both of the other two containers

The ROS container and Python container have two workspaces:
1. A Project Workspace which holds the specific project data for each of the two;
2. A Common Workspace which holds only the data which has to be shared between them.

## Requirements
- [Docker](https://docs.docker.com/engine/install/)
- [VS Code](https://code.visualstudio.com/) [DevContainers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) (optional, but recommended)

## How to setup
### 1. Download necessary files from external repositories
In order to correctly execute the project you need to rely on external resources:
- [MoveIt2 Tutorials](https://github.com/moveit/moveit2_tutorials/tree/humble) which provides a way to install MoveIt2 and some useful configurations for specific robots which are then used as a baseline for our workspace;
- [Rviz2](https://github.com/ros2/rviz/tree/humble) 11.2.18 (the latest 11.2.19 version makes the MoveIt Setup Assistant crash on loading the URDF, see [here](https://github.com/moveit/moveit2/issues/3541));
- [Universal Robots Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/humble) which contains a URDF description of various Universal Robots Products; 
- [Universal Robots Gazebo Simulation](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation) which contains useful files and requirements for the Gazebo Simulation of the aforementioned Universal Robots products;
- [Robotiq Gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper) which contains description and utilities related to different models of the Robotiq Gripper.

This can be automated by the `create_workspace.sh` script in the root of the repository by simply running:
```bash
bash ./create_workspace.sh
```

Other than downloading such dependencies, it will create a file with the specified password for the noVNC Server, which has to be at least 6 characters long and alphanumeric only, with no spaces.

### 2. Run the DevContainers
The project is structured to take advantage of DevContainers: it is in fact possible to open the root of the repository in VS Code and using its [DevContainers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) build and spin up the necessary containers, and develop directly in them without needing to install anything other than [Docker](https://www.docker.com/) on the host machine.

It is obviously possible to spin up the DevContainers in other ways too, or even use the Docker Compose file alone.

> It is recommended to run at least the `ros-humble-sim` DevContainer once before working to let it build all of the necessary packages for the simulation.

### 3. GUI Apps
In case GUI Apps are used (Rviz, Gazebo, Python), the output is redirected to an additional container with a Desktop Environment and a noVNC Server running which is spun up by the `docker-compose.yml` file in conjunction with the two DevContainers. 

To access its GUI it's possible to open [http://localhost:6080/vnc.html](http://localhost:6080/vnc.html) in a browser and use the password provided at [step 1](#1-download-necessary-files-from-external-repositories).

#### Note: accessing the noVNC Server from external devices
The default Docker Compose configuration in the `docker-compose.yml` file only allows for connections to the noVNC server coming from the same machine, as show below.
```yml
novnc_server:
container_name: novnc_server
ports:
    # Allow connections to the noVNC Host from the Docker host only
    - 127.0.0.1:6080:6080
    # Allow connections to the noVNC Host from every interface of the Docker Host
    # - 6080:6080
build:
    context: ./utils/novnc_server/
    dockerfile: Dockerfile
volumes:
    # noVNC Server utilities and scripts
    - ./utils/novnc_server/scripts:/scripts
    # X11 Socket
    - ./utils/novnc_server/desktop-x11:/tmp/.X11-unix
    - ./utils/novnc_server/desktop-x11/.Xauthority:/tmp/.Xauthority
environment:
    # X11 Setup
    - DISPLAY=:1
    - XAUTHORITY=/tmp/.Xauthority
command: [ "/scripts/startup.sh" ]
```
It is possible to tweak this configuration by changing the "`ports`" section as follows
```yml
novnc_server:
container_name: novnc_server
ports:
    # Allow connections to the noVNC Host from the Docker host only
    # - 127.0.0.1:6080:6080
    # Allow connections to the noVNC Host from every interface of the Docker Host
    - 6080:6080
build:
    context: ./utils/novnc_server/
    dockerfile: Dockerfile
volumes:
    # noVNC Server utilities and scripts
    - ./utils/novnc_server/scripts:/scripts
    # X11 Socket
    - ./utils/novnc_server/desktop-x11:/tmp/.X11-unix
    - ./utils/novnc_server/desktop-x11/.Xauthority:/tmp/.Xauthority
environment:
    # X11 Setup
    - DISPLAY=:1
    - XAUTHORITY=/tmp/.Xauthority
command: [ "/scripts/startup.sh" ]
```
If the firewall is correctly set up (most of the times it is on Linux at least, since UFW is completely bypassed by Docker and iptables rules are automatically generated to work seamlessly with FirewallD) it should be possible to reach the noVNC server from any machine capable of reaching [http://host.machine.ip.address:6080/vnc.html]().

It is possible (and highly recommended!) to run the noVNC server behind a reverse proxy in case it is made available publicly.

## Run the simulation
### 1. Spawn the Robot with the depth camera and take pictures
Inside the ROS Humble DevContainer:
```bash
ros2 launch ur_gripper_sim get_data_w{i}.launch.py
```
Where "`{i}`" has to be substituted with a number from 1 to 3 to select the demo world where to run the simulation.

### 2. Predict the contours of the object
Inside the Path Predictor DevContainer:
```bash
python ./predict.py
```

### 3. Spawn the Robot with the MoveIt2 Controllers and run the inverse kinematics
Inside the ROS Humble DevContainer, in two separate terminal windows:
```bash
# Terminal Window 1
ros2 launch ur_gripper_sim sim_w{i}.launch.py
```
```bash
# Terminal Window 2
## Run the node once the simulation environment (Terminal Window 1) is ready
ros2 run ur_gripper_sim run_ik
```
Where "`{i}`" has to be the same as the one at step 2.1, otherwise other contours will be used.

## Acknowledgements
The simulation setup (Robot spawning in Gazebo, its MoveIt setup and the attachment of the Robotiq Gripper) has been inspired by the tutorials from [LearnRoboticsWithROS](https://www.learn-robotics-with-ros.com/) ([YouTube](https://www.youtube.com/@learn-robotics-with-ros), [GitHub](https://github.com/LearnRoboticsWROS)), setup which has been adapted for its usage in our simulation.
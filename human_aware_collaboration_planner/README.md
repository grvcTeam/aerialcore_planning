# Human-Aware Collaboration Planner
## Overview
This repository is about a cognitive task planner in charge of planning missions and governing the behaviour of multi-UAV teams carrying out inspection and maintenance tasks jointly with human operators.

The system is designed to ensure the safety of both airborne equipment and human workers at all times.

This system implements a software layer for a multi-layer software architecture that is further divided into two, a High-Level Planner centralised on the ground and an Agent Behavior Manager distributed on board each UAV. In this way, the High-Level Planner receives task requests as inputs, and its work is to coordinate all the UAVs, asigning to each of them a mission plan to follow. The Agent Behavior Manager is the one in charge of executing and supervising those plans, calling the appropriate lower-level controllers at any given time.

The controllers for each specific action inside each of the tasks are not included in this software layer, but simple
version of those controllers can be found in this repository in order to be able to test the software layer propoerly in
simulation. Real controllers will be inplemented in the next layer of the multi-layer software architecture.

This software is currently being developed on Ubuntu 18.04 with ROS Melodic.


A more detailed description of the system can be found [here](https://www.researchgate.net/publication/360514763_Mission_Planning_and_Execution_in_Heterogeneous_Teams_of_Aerial_Robots_supporting_Power_Line_Inspection_Operations).

If you are using this software layer or you found this approach inspiring for your own research, please cite:
```bibtex
@INPROCEEDINGS{CalvoICUAS22,  
  author        = {Calvo, Alvaro and Silano, Giuseppe and Capitan, Jesus},  
  booktitle     = {2022 International Conference on Unmanned Aircraft Systems (ICUAS)},   
  title         = {Mission Planning and Execution in Heterogeneous Teams of Aerial Robots supporting Power Line Inspection Operations},
  year          = {2022},  
  pages         = {1644-1649},
  doi           = {10.1109/ICUAS54217.2022.9836234}
}
```

## Installation
To install the repositories correctly, you have to follow the next steps:

0.1. ROS Melodic installation

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
```

0.2. (Recomended) Catkin tools

```
sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y python-catkin-tools
```

1. Install necessary packages

```
sudo apt install -y libeigen3-dev ros-melodic-geodesy ros-melodic-joy ros-melodic-multimaster-fkie
pip install pynput
sudo apt install -y xz-utils
```

2. Create a ROS workspace

```
mkdir -p ~/mission_planner_ws/src
cd ~/mission_planner_ws/
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python
source devel/setup.bash
echo "source $HOME/mission_planner_ws/devel/setup.bash" >> ~/.bashrc
```

3. Clone necessary repositories

```
cd ~/mission_planner_ws/src/

git clone https://github.com/ctu-mrs/aerialcore_simulation.git
git clone https://github.com/grvcTeam/aerialcore_planning.git
git clone https://github.com/Angel-M-Montes/path_planner.git
git clone https://github.com/grvcTeam/grvc-ual.git
git clone https://github.com/grvcTeam/grvc-utils.git
```

4. Clone the necessary packages, which are: grvc-ual, grvc-utils, BehaviorTree.CPP, Groot

```
git clone https://github.com/ctu-mrs/aerialcore_simulation.git
git clone https://github.com/Angel-M-Montes/path_planner.git
git clone https://github.com/grvcTeam/grvc-ual
git clone https://github.com/grvcTeam/grvc-utils
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
git clone https://github.com/BehaviorTree/Groot.git
```

5. Ignore some packages

```
touch ~/mission_planner_ws/src/aerialcore_planning/large_scale_inspection_planner/CATKIN_IGNORE
touch ~/mission_planner_ws/src/grvc-utils/mission_lib/CATKIN_IGNORE
```

6. Install BehaviorTree.CPP package and its dependencies

```
sudo apt-get install -y libzmq3-dev libboost-dev
sudo apt-get install -y ros-melodic-behaviortree-cpp-v3
```

7. Install Groot and its dependencies

```
cd ~/mission_planner_ws/src/
sudo apt install -y qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev
git clone https://github.com/BehaviorTree/Groot.git
touch ~/mission_planner_ws/src/Groot/CATKIN_IGNORE
cd ..
rosdep install --from-paths src --ignore-src
catkin build
```

8. Install and configure UAL. Only MAVROS needed. Make sure to install its dependencies when asked

```
cd ~/mission_planner_ws/src/grvc-ual
./configure.py
```

9. Install MAVROS packages

```
sudo apt install -y ros-melodic-mavros ros-melodic-mavros-extras
sudo geographiclib-get-geoids egm96-5
sudo usermod -a -G dialout $USER
sudo apt remove modemmanager
```

10.1 (Optional) Install RealSense plugins for real-life execution

```
sudo apt install -y ros-melodic-realsense2-camera ros-melodic-realsense2-description
```

10.2 (Optional) Download 99-realsense-libusb.rules file from [github](https://github.com/IntelRealSense/librealsense/blob/master/config/99-realsense-libusb.rules)

10.3 (Optional) Give permissions to read the data from the RealSense camera

```
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules
```

11. Install PX4 for SITL simulations

```
sudo apt install -y libgstreamer1.0-dev python-jinja2 python-pip
pip install numpy toml
cd ~/mission_planner_ws/src/
git clone https://github.com/PX4/Firmware.git
cd Firmware
git checkout v1.10.2
git submodule update --init --recursive
make
make px4_sitl_default gazebo
```

12. Build

```
cd ~/mission_planner_ws/
catkin build
```

**Note**: In case that the installation did not go well, try compile each package individually.

## Test

To test if the system is working correctly you can launch a simulation and order tasks or unespected events by executing Makefile recipes.

**Note**: Simulation.launch file has some parameters to facilitate the configuration of the simulations, having parameters to select the number of UAVs, the Gazebo world, debug modes and some other things. See the heading of the file to know the different world options.

```
make launch
...
make monitor task_id=1 human_target=human_target_1 number=2 distance=1.5
make inspect task_id=2
make deliver task_id=3 human_target=human_target_1 tool=hammer
...
make battery_off agent_id=1
make battery_ok  agent_id=1
...
make mission_over
...
rosnode kill /uav_1/agent_behaviour_manager
rosrun human_aware_collaboration_planner agent_behaviour_manager __ns:uav_1
...
rosnode kill /high_level_planner
```

**Note**: For information on how to launch tasks manually you can run `make gesture_info` or just read the recipes in the Makefile.

## Monitoring the Behaviour Tree execution with Groot

There is a Make recipe for launching a Groot node to monitor the execution of the BT in real time or replay a log file.


```
make groot
```

To monitor a BT we just has to specify:
* Server IP: localhost
* Publisher Port: 1666 + ID * 2
* Server Port: 1667 + ID * 2

E.g. for the UAV_1:
* Server IP: localhost
* Publisher Port: 1668
* Server Port: 1669

**Note**: fbl log files are stored in `~/.ros` and are named as `bt_trace_uav_` + ID + `.fbl`

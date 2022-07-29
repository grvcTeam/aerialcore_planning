# Human-Aware Collaboration Planner
## Overview
This repository is about a cognitive task planner in charge of planning missions and governing the behaviour of multi-UAV teams carrying out inspection and maintenance tasks jointly with human operators.

The system is designed to ensure the safety of both airborne equipment and human workers at all times.

This system implements a software layer for a multi-layer software architecture that is further divided into two, a High-Level Planner centralised on the ground and an Agent Behavior Manager distributed on board each UAV. In this way, the High-Level Planner receives task requests as inputs, and its work is to coordinate all the UAVs, asigning to each of them a mission plan to follow. The Agent Behavior Manager is the one in charge of executing and supervising those plans, calling the appropriate lower-level controllers at any given time.

The controllers for each specific action inside each of the tasks are not included in this software layer, but simple
version of those controllers can be found in this repository in order to be able to test the software layer propoerly in
simulation. Real controllers will be inplemented in the next layer of the multi-layer software architecture.

This software is currently being developed on Ubuntu 20.04 with ROS Noetic.


A more detailed description of the system can be found [here](https://www.researchgate.net/publication/360514763_Mission_Planning_and_Execution_in_Heterogeneous_Teams_of_Aerial_Robots_supporting_Power_Line_Inspection_Operations).

If you are using this software layer or you found this approach inspiring for your own research, please cite:
```bibtex
@INPROCEEDINGS{CalvoICUAS22,  
  author        = {Calvo, Alvaro and Silano, Giuseppe and Capitan, Jesus},  
  booktitle     = {2022 International Conference on Unmanned Aircraft Systems (ICUAS)},   
  title         = {Mission Planning and Execution in Heterogeneous Teams of Aerial Robots supporting Power Line Inspection Operations},
  year          = {2022},  
  pages         = {1644-1649},
  doi            = {10.1109/ICUAS54217.2022.9836234}
}
```

## Installation
To install the repositories correctly, you have to follow the next steps:

1. Create a workspace

```
cd ~
mkdir your_ws
cd your_ws
mkdir src
cd src
```

2. Install the aerialcore_planner repository

```
git clone https://github.com/grvcTeam/aerialcore_planning.git
```

3. Clone the necessary packages, which are: grvc-ual, grvc-utils, BehaviorTree.CPP, Groot

```
git clone https://github.com/ctu-mrs/aerialcore_simulation.git
git clone https://github.com/Angel-M-Montes/path_planner.git
git clone https://github.com/grvcTeam/grvc-ual
git clone https://github.com/grvcTeam/grvc-utils
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
git clone https://github.com/BehaviorTree/Groot.git
```

4. Compile BehaviorTree.CPP

```
sudo apt-get install libzmq3-dev libboost-dev
cd ~/your_ws/src/BehaviorTree.CPP
mkdir build; cd build
cmake ..
make
sudo make install
```

5. Groot dependencies

```
cd ~/your_ws/
sudo apt install qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev
rosdep install --from-paths src --ignore-src
catkin build
```

6. Install and configure UAL. Only MAVROS needed. Install Dependencies

https://github.com/grvcTeam/grvc-ual/wiki/How-to-build-and-install-grvc-ual

```
cd ~/your_ws/src/grvc-ual
./configure.py
```

7. Install dependencies of grvc-ual

```
sudo apt-get install libeigen3-dev ros-melodic-geodesy ros-melodic-joy
```

8. Install MAVROS packages

```
sudo apt install -y ros-melodic-mavros ros-melodic-mavros-extras
sudo geographiclib-get-geoids egm96-5
sudo usermod -a -G dialout $USER
sudo apt remove modemmanager
```

9. Install PX4 for SITL simulations

https://github.com/grvcTeam/grvc-ual/wiki/Setup-instructions:-PX4-SITL

```
sudo apt install libgstreamer1.0-dev python-jinja2 python-pip
pip install numpy toml
cd ~/your_ws/src/
git clone https://github.com/PX4/Firmware.git
cd Firmware
git checkout v1.10.2
git submodule update --init --recursive
make
make px4_sitl_default gazebo
```

10. Build and source

```
cd ~/your_ws/
catkin build
```

**Note**: add the source to .bashrc or .zshrc

```
echo "source ~/your_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/your_ws/devel/setup.zsh" >> ~/.zshrc
```


**Note**: In case that the installation did not go well, try compile each package individually.

## Test

To test if the system is working correctly you can launch a simulation and order tasks or unespected events by executing Makefile recipes.

**Note**: The number of UAVs in the simulation can be configured by editing the first argument of launch/simulation.launch. Giving values from 1 to 5.

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

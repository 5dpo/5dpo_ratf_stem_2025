# 5dpo_ratf_stem_2025

TBC

## Setup

### Repository

```sh
mkdir ~/ros1_ws/src -p

cd ~/ros1_ws/src
git clone git@github.com:5dpo/5dpo_ratf_stem_2025.git

cd 5dpo_ratf_stem_2025
git submodule init
git submodule update --recursive
```

### Robot Operating System (ROS)

**System Requirements**
- ROS 1 Noetic Ninjemys ([wiki](https://wiki.ros.org/noetic))
- Ubuntu 20.04.6 LTS (Focal Fossa) ([url](https://releases.ubuntu.com/focal/))

**Dependencies**
```sh
# Ubuntu 20.04.6 LTS
sudo apt install ros-noetic-robot-pose-ekf ros-noetic-joy

# Raspberry Pi
# follow instructions at https://github.com/5dpo/5dpo_ratf_2023/blob/main/doc/sbc/rpi4b.md
# on how to install ROS 1 in RPi
```

## Usage

### Build

```sh
# Ubuntu 20.04.6 LTS
# you may require to ignore MPU node specific for RPi with ext IMU and SPI comms
# touch src/5dpo_ratf_2023/drivers/MPU9255_for_5dpo/CATKIN_IGNORE

source /opt/ros/noetic/setup.bash

cd ~/ros1_ws
catkin_make --force-cmake -DCMAKE_BUILD_TYPE=Release

source devel/setup.bash
```

### Launch

**Simulation**
```sh
roslaunch sa_robot_ros_nav_conf_sim wake_up_simulated_robot.launch
```

**5dpo Robot@Factory 4.0 Platform**
```sh
export ROBOT_CONF=<basic|sa_robot>

roslaunch sa_robot_ros_nav_conf wake_up_almighty_ratf.launch
```

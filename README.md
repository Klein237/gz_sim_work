# ros_gz_project
A project integrating ROS 2 and Gazebo simulator.

## Included packages

* `ros_gz_description` - holds the sdf description of the simulated system and any other assets.

* `ros_gz_gazebo` - holds gazebo specific code and configurations. Namely this is where systems end up.

* `ros_gz_application` - holds ros2 specific code and configurations.

* `ros_gz_bringup` - holds launch files and high level utilities.


## Prerequisites

* Ubuntu 24.04 LTS
* ROS 2 Jazzy 
* Gazebo Sim Harmonic

---

## Installation

```bash
# 1. Clone this repository
git clone git@github.com:Klein237/gz_sim_work.git
cd gz_sim_work

# 2. Install dependencies
sudo apt update
sudo apt install \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-slam-toolbox 

# 3. Build and source
colcon build 
source install/setup.bash
```

---

## Usage

### 1. Triggering the simulation in Gazebo and RViz

1. **Launch the simulation**

   ```bash
   ros2 launch ros_gz_bringup gzsim.launch.py 
   ```
2. **Launch the mapping with slamtoolbox**

   ```bash
   ros2 launch ros_gz_application mapping.launch.py
   ```

### 2. Autonomous navigation with nav2

1. **Launch with ROS 2 control enabled**

   ```bash
   ros2 launch ros_gz_application navigation.launch.py 
   ```
---

> **Tip:** When launching with we launch the simulation by default, the teleoperation is activate (`use_joy:=false` to disable ) and rviz is disable (`rviz:=true` to activate) . For the navigation, rviz is activate.

---
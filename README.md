# Robot Programming Project 
This project uses the ICP (Iterative Closest Point) algorithm in order to localize a robot inside a map, using ROS (Robot Operating System).

## Requirements
This projects assumes you have _ROS Noetic_already installed in your machine.
### Map Server
- Install the `ros-${DISTRO}-map-server` package.
   ```sh
    sudo apt install ros-noetic-map-server
   ```
### Stage-ROS
- Install the `ros-${DISTRO}-stage-ros` and `ros-${DISTRO}-teleop-twist-keyboard` package.
  ```sh
  sudo apt install ros-noetic-stage-ros ros-noetic-teleop-twist-keyboard
  ```
## Create workspace & clone directory
To run the package create a ROS workspace folder.
In particular, follow the steps below:
```sh
cd ~
mkdir ros_workspaces
cd ros_workspaces
mkdir ICP_localization_ws
cd ICP_localization_ws
mkdir src
cd src
catkin_init_workspace
```
Then, clone the git repository:
```sh
git clone https://github.com/antoscardi/ICP_Localization.git
```

Then build your catkin workspace (-DPYTHON_EXECUTABLE=/usr/bin/python3 argument is needed only if it's the first time the workspace is initialized):
```sh
cd ..
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```
## Run the project 
Then to launch the project you just run the launch file:
```sh
roslaunch src/ICP_Localization/launch_file.launch
```
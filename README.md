# Robot Programming Project 
This project uses the ICP (Iterative Closest Point) algorithm in order to localize a robot inside a map, using ROS (Robot Operating System).

## Requirements
This projects assumes you have _ROS Noetic_ already installed in your machine.
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
To run the package create a ROS workspace in your root folder.
In particular, follow the steps below (create a folder for all ROS workspaces):
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
Then to launch the project you only need to run the launch file:
```sh
roslaunch src/ICP_Localization/launch_file.launch
```
In RViz you need to set the initial pose of the robot, sufficiently close to the true one shown in the map server.
Now, you can freely move the robot around and see it is correctly localized in the map, as shown in the example below:

![icp_video](https://github.com/antoscardi/ICP_Localization/assets/99209099/32e2e4f6-7ea4-4fe7-ba59-70a74e535b32)

This project uses the ICP algorithm to localize a robot inside a map.

To run the package create a ROS workspace folder.
I followed the following steps:
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

Then build your catkin workspace (-DPYTHON_EXECUTABLE=/usr/bin/python3 argument is needed if it's the first time the workspace is initialized):
```sh
cd ..
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

Then to launch the project you just run the launch file:
```sh
roslaunch src/ICP_Localization/launch_file.launch
```
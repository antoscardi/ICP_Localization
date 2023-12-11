This project uses the ICP algorithm to localize a robot inside a map.

To run the package create a ROS workspace folder.
I followed the following steps:
```sh
cd ~
mkdir ros_workspaces
cd ros_workspaces
mkdir ICP_localization_ws
cd CP_localization_ws
mkdir src
cd src
catkin_init_workspace
cd ..
```
Then, clone the git repository:
cd src
git clone https://github.com/antoscardi/ICP_Localization.git

Then build your catkin workspace
cd ..
catkin_make

Then to launch the project you just run the launch file:
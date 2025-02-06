# 🤖 Robot Programming Project
This is the repository of the Robot programming course project, which employs **laser scan** data processed by the **ICP (Iterative Closest Point) algorithm** to localize a robot inside a predetermined map of the environment, using **ROS (Robot Operating System)**. 🚀  

---

## 📌 Requirements  

This project assumes you have **_ROS Noetic_** already installed on your machine.  

### 🌏 Map Server  
- Install the `ros-${DISTRO}-map-server` package.
   ```sh
    sudo apt install ros-noetic-map-server
   ```
### 🔭 Stage-ROS & Teleop
- Install the `ros-${DISTRO}-stage-ros` and `ros-${DISTRO}-teleop-twist-keyboard` package.
  ```sh
  sudo apt install ros-noetic-stage-ros ros-noetic-teleop-twist-keyboard
  ```
## 🛠️ Create Workspace & Clone Repository
To run the package, create a **ROS workspace** in your root folder.
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
Then, **clone the GitHub repository**: 
```sh
git clone https://github.com/antoscardi/ICP_Localization.git
```

Build your catkin workspace (**only add `-DPYTHON_EXECUTABLE=/usr/bin/python3` if it's the first time initializing the workspace**): 
```sh
cd ..
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```
## ▶️ Run the Project  
To launch the project, simply run the launch file:
```sh
roslaunch src/ICP_Localization/launch_file.launch
```
In **RViz**, set the **initial pose of the robot** 📍 **close to its actual position** shown in the **map server** 🌍.

Now, you can freely move the robot around and see it is correctly localized in the map.

Example demonstration: 
![simulation](temp.gif)




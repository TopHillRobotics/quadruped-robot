# 1.Overview

This navigation package provides the functions of robot simultaneous localization and mapping (SLAM) and autonomous navigation.

Here some pictures in simulation

<p align="left">
    <image src = "https://github.com/TopHillRobotics/quadruped-robot/blob/develop/media/gmapping_demo.png" height = "300"/>
    <image src = "https://github.com/TopHillRobotics/quadruped-robot/blob/develop/media/cartographer_demo.png" height = "300"/>
</p>

# 2. Source Code Structure

This source code include four directories

- **sensor_gazebo_plugin** contains some sensor gazebo_plugin

- **slam** contains some slam demos of some basic frameworks, such as radar-based gmapping, cartographer, as well as tests and configuration files for the current mainstream SLAM frameworks. Such as vision-based ORB-SLAM2, vision and IMU fusion ORB-SLAM3, radar, IMU and gps fusion LIO-SAM, radar, vision and IMU fusion LVI-SAM, etc.

- **navigation** Contains some demos for path planning and autonomous navigation.

---

# 3. Installation

## 3.1 Install gmapping

The gmapping package is already integrated with ros, you just need to run .

```
sudo apt install ros-${your_ros_version}-gmapping
```
If you want to know more,please visit https://wiki.ros.org/slam_gmapping 

We also should install the pointcloud_to_laserscan ros package to convert our 3D point cloud to 2D laser.
```
sudo apt install ros-${your_ros_version}-pointcloud_to_laserscan
```
## 3.2 Install catrographer
The cartographer_ros package is already integrated with ros, you just need to run .
```
sudo apt install ros-${your_ros_version}-cartographer-ros ros-${your_ros_version}-cartographer-rviz
```
But the version of ros-noetic  do not release, you should install it by source https://github.com/cartographer-project/cartographer

If you want to know more,please visit https://google-cartographer-ros.readthedocs.io/en/latest/ 

## 3.3 Install navigation
The amcl package and move_base package is already integrated with ros
```
sudo apt install ros-${your_ros_version}-navigation
```
If you want to know more,please visit http://wiki.ros.org/navigation

# 4. Run Demos
First, in one terminal, source the `setup.bash` to set up the environment

```
source ${your_workspace}/devel/setup.bash
```

Second, run the Gazebo simulator and load a robot.

```
roslaunch qr_gazebo normal.launch rname:=a1 wname:=mini_maze use_xacro:=true use_lidar:=true
```

In this command, **rname** specifies the robot you use, **wname** specifies the gazebo_world you use, **use_xacro** means whether you use URDF or XACRO file, **use_lidar** means whether you use the sensor of lidar.

Third, in a new terminal, launch the slam demo. It will start the node of rviz and the demo_trot_keyboard, you can move the robot through the keyboard to build the map.

```
rosrun demo demo_slam_gmapping
```
Similarly，you also can run
```
rosrun demo demo_slam_cartographer
```
You can run the navigation demo
```
rosrun demo demo_navigation_2d_use_map
```
Use the 2D Nav Goal to navigate your robot to the target point,you can chose the lidar or camera to avoid obstacles.The map is provided by the demo of slam,you can use the map_server to save the map.

You alse can run the demo
```
rosrun demo demo_navigation_2d_gmapping
```
It don't need the map,you can map and navigate in real time.












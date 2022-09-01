# 1. Overview
Here goes overview

# 2. Installation

## 2.1 Install ROS

You need install ROS (Robot Operating System) first. We tested the codes under Ubuntu Linux and ROS 1 Melodic Morenia distribution. Other newer ROS distributions are supposed to be supported. Please visit http://www.wiki.ros.org for ROS installation.

## 2.2 Clone the source code

```
cd ${your_workspace}
mkdir src
cd ${your_workspace}/src
catkin_init_workspace
git clone https://github.com/TopHillRobotics/quadruped-robot/
```

## 2.3 Install the following third party dependencies

* yaml-cpp
* eigen3
* lcm
* glm

```
sudo apt install libyaml-cpp-dev
sudo apt install libeigen3-dev
sudo apt install liblcm-dev
sudo apt install libglm-dev
```

## 2.4 Compile the codes

```
cd ${your_workspace}
catkin_make
```

# 3. Run Demos

## 3.1 Browse the demos

Browse the directories `src/demo/${robot}` to find the corresponding demo. We support the robots provided by two companies: unitree and deeprobotics. 

Our locomotion controllers support two modes:  velocity control and position control. Please check out the corresponding demos.

## 3.2 Run a demo

First, run

```
`${your_workspace}/devel/setup.bash` 
```

Second, run the Gazebo simulator to load a robot.

```
roslaunch unitree_gazebo normal.launch
```

Third, in a new terminal, launch a demo and run the quadruped controller node. Here, a demo helloworld lets the quadruped robot stand up.

```
rosrun a1sim demo_helloworld
```


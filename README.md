# 1. Overview

This repository provides an architecture to control quadruped robots. It provides state estimator, gait generator, stance and swing leg controllers.  

The project includes demo, extern quadruped and simulation, totally four ROS packages. The **demo** package provides some simple examples of the control flow and examples that uses different control modes. The **extern** packages consists of third party libraries. The **quadruped** is the main package that provides the control architecture. The **simulation** package is used for launching simulation environment in gazebo. Currently it uses code and structure provided by unitree. More details about simulation is at [this link](https://github.com/unitreerobotics/unitree_ros).

The control architecture now supports:

- **velocity mode** allow user controlling the speed and rotation of the robot.

- **position mode** will generate gaits from specific gait configuration.

- **hybrid mode** uses position and torque information. It allows more complicated control methods


The project now supports Unitree robots, such as A1, AlienGO and GO1. In near future, the project will support robots from DEEPRobotics. For more information, please check websites of [Unitree](https://github.com/unitreerobotics) and [DEEPRobotics](https://www.deeprobotics.cn/).

The **main** branch will support Model-predictive control and Walk Locomotion in near future, which still need some adjustment. We will also provide whole body control.

<p align="center">
  <img src="media/trot-mpc.gif" width="600" />
</p>

<p align="center">
  <img src="media/walk-locomotion.gif" width="600" />
</p>

The project also supports real robots. Currently we test the system on Unitree A1.

https://user-images.githubusercontent.com/56444225/187832912-4fb6e0d9-cb24-4cce-87aa-52b7457a5b4b.mp4

---

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


# 1. Overview

This project provides an architecture and a few key algorithms to control quadruped robots, including state estimator, gait generator, stance and swing leg controllers. 
This project supports three control modes

- **velocity mode** allows a user to control the robot's linear and angular velocity.

- **position mode** generates user-defined gaits using gait configurations.

- **hybrid mode** uses position and torque to implement flexible locomotion.

The project now supports Unitree A1 robot and DeepRobotics Lite2A robot, and can be extended to support other quadruped robots such as Unitree AlienGO/GO1, DeepRobotics Jueying/X20 and Anymal. For more information about quadruped robots, check out the following websites

-  [Unitree](https://github.com/unitreerobotics)

-  [DEEPRobotics](https://www.deeprobotics.cn/)

-  [AnyRobotics](https://www.anybotics.com/anymal-autonomous-legged-robot/)

Here some snapshots in simulation.

<p align="center">
  <img src="media/trot-mpc.gif" width="600" />
</p>

<p align="center">
  <img src="media/walk-locomotion.gif" width="600" />
</p>

Here is a demo for a real quadruped robot (Unitree A1).

<p align="center">
  <img src="media/real.gif" width="600" />
</p>


---


# 2. Source Code Structure

This source code include four directories

- **demo** contains various demos to understand the software architecture and algorithms.

- **extern** contains the third-party dependencies to successfully run our code.

- **quadruped** contains the core algorithms of our project.

- **simulation** contains the configuration to run the simulation.

---

# 3. Installation

## 3.1 Install ROS

You need install ROS (Robot Operating System) first. We tested the codes under Ubuntu Linux and ROS 1 Melodic Morenia distribution. Other newer ROS distributions are supposed to be supported. Please visit http://www.wiki.ros.org for ROS installation.

## 3.2 Clone the source code

```
cd ${your_workspace}
mkdir src
cd ${your_workspace}/src
catkin_init_workspace
git clone https://github.com/TopHillRobotics/quadruped-robot/
```

## 3.3 Install the following third party dependencies

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

## 3.4 Compile the codes

```
cd ${your_workspace}
catkin_make
```
For a smooth compilation, we suggest using CMake version 3.15 or greater.

---

# 4. Run Demos

## 4.1 Browse the demos

Browse the directories `src/demo/${robot}` to find the corresponding demo. We support the robots provided by two companies: unitree and deeprobotics. 

Our locomotion controllers support two modes:  velocity control and position control. Please check out the corresponding demos.

## 4.2 Run a demo

First, in one terminal, source the `setup.bash` to set up the environment

```
source ${your_workspace}/devel/setup.bash
```

Second, run the Gazebo simulator and load a robot.

```
roslaunch unitree_gazebo normal.launch rname:=a1 use_xacro:=true
```

In this command, **rname** specifies the robot you use, **use_xacro** means whether you use URDF or XACRO file.

Third, in a new terminal, launch a demo and run the quadruped controller node. Here, a demo helloworld lets the quadruped robot stand up.

```
rosrun demo demo_helloworld
```

For more demos, please check out the directory /demos. If you have a robot **YAML** configuration file as long as  **xacro** or **URDF**, you can specify the file location when instantiate the **qrRobotSim** class, or write your own subclass to run your robot and then configure  your xacro and URDF files in **simulation** package, you can try your own robot.

---

# 5. Feedback and Bugs

Please file bugs and feature requests here: [https://github.com/TopHillRobotics/quadruped-robot/issues](https://github.com/TopHillRobotics/quadruped-robot/issues)

You can help to ensure your issue gets fixed if you provide sufficient detail. 

---

# 6. Documentation

Check out the [readthedocs](https://quadruped-robot-docs.readthedocs.io/en/main/index.html)  for helps, tutorials, demo explanation. 

---

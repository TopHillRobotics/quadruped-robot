# Quadruped Overview

This repository provides an architecture to control quadruped robots. It provides state estimator, geit generator, stance and swing leg controllers.  

The project includes demo, extern quadruped and simulation, totally four ROS packages. The **demo** package provides some simple examples of the control flow and examples that uses different control modes. The **extern** packages consists of third party libraries. The **quadruped** is the main package that provides the control architecture. The **simulation** package is used for launching simulation environment in gazebo. Currently it uses code and structure provided by unitree. More details about simulation is at [this link](https://github.com/unitreerobotics/unitree_ros).

The control architecture now supports:

- **velocity mode** allow user controlling the speed and rotation of the robot.

- **position mode** will generate gaits from specific gait configuration.

- **hybrid mode** uses position and torque information. It allows more complicated control methods


The project now supports Unitree robots, such as A1, AlienGO and GO1. In near future, the project will support robots from DEEPRobotics. For more information, please check websites of [Unitree](https://github.com/unitreerobotics) and [DEEPRobotics](https://www.deeprobotics.cn/).

The **main** branch will support Model-predictive control and Walk Locomotion in near future, which still need some adjustment. We will also provide whole body control.

<img src="media/trot-mpc.gif" width="300" />

<img src="media/walk-locomotion.gif" width="300" />

The project also supports real robots. Currently we test the system on Unitree A1.

<video src="media/real.mp4"></video>




---

# Install guide
## Step 1: install the third party dependencies:s
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

And you need to install ros environment. Please go to www.wiki.ros.org for the ros installation information.

Other dependencies please see in `src/simulation/README.md`.

## Step 2: compile the exectuable locomotion controller or other ros packages

```
cd ${WorkSpace}
catkin_make
```

Currently, the locomotion controller support Velocity Mode and Position Mode, as you can browse through `src/demo/${robot}` directory to select corresponding demo.

## Step 3: run the controller

Please always run `devel/setup.bash` before running these demo.

You need to start the gazebo simulation environment to load the robot model first.

```
roslaunch unitree_gazebo normal.launch
```

Then in another new terminal, launch the controller node. This is the simplest demo which makes the quadruped robot stand up only.

```
rosrun a1sim demo_helloworld
```

## Step 4 (Option): Advances

We also provided many different demoes which are combined with different gait and different locomotion. For example, you could run trot in velocity locomotion demo with `rosrun a1sim demo_trot_velocity`. What's more you can run `rosrun a1sim demo_trot_keyboard` to control the robot via keyboard. By the way, the joystick only support ps4 controller so far.

And also you can read the cpp code of  each demo and the files in `config` to learn how to combine the locomotion and gait to realize different demo

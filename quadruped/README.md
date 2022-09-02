## Step 1: install the third party dependencies:

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

And also you can read the cpp code of  each demo and the files in `config` to learn how to combine the locomotion and gait to realize different demo.

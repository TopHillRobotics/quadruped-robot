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
rosrun a1sim stand_up
```

## Step 4 (Option): Advances

We also provided many different demoes which are combined with different gait and different locomotion. For example, you could run trot in velocity locomotion demo with `rosrun a1sim trot_velocity_motion`. What's more you can run `rosrun a1sim trot_velocity_motion_with_keyboard` to control the robot via keyboard.

And also you can read the code of  `main.cpp` and the files in `config` to learn how to combine the locomotion and gait to realize different demo.

# 1. Overview

This project branch integrates some advanced algrithms such as MPC and WBC to control quadruped robots.

This branch have been tested in **ROS Noetic** under **Ubuntu 20.04**. Some users report it can run in **ROS Melodic** under **Ubuntu 18.04**. To ensure optimal performance, we recommend testing this branch in **ROS Noetic** under **Ubuntu 20.04**.

Note that this branch is different from the one in the `main` branch. The architecture and major modules are shown below

<div align="center">
    <img src="./img/mpc-wbc process diagram.png">
</div>

The codes are tested for Unitree A1 and DeepRobotics Lite3. To fine-tune the MPC and WBC algorithms, you can adjust the corresponding parameters (e.g. KP and KD in **quadruped/src/controllers/wbc/task_set** or weights in WBC locomotion controller).

# 2. Source Code Structure

The source code includes five major directories

- **examples** contains a few examples to help users understand the usage and the project architecture.
- **quadruped** contains the major modules defining robots, controllers, planner, dynamics and supporting algorithms.
- **robot_description** contains the files needed to represent robot models.
- **simulation** contains the necessary environment to run examples in the simulator.
- **xpp** contains a visualization tool for robots.

# 3. Build the Project

## Step 1: Install the third party dependencies
Please ensure that all required dependencies have been installed before building the project
```
sudo apt install libyaml-cpp-dev
sudo apt install libeigen3-dev
sudo apt install liblcm-dev
sudo apt install libglm-dev
```

## Step 2: Compile the codes
Navigate to your workspace
```
cd ${your_workspace}
```
Build the project using ROS tool `catkin_make`
```
catkin_make
```
If you have a less powerful machine, there are a few steps you can take to optimize the project performance.
+ To improve simulation performance, navigate to the `.world` file (the default file is `earth.world`) located in the **simulation/qr_gazebo/worlds** folder and adjust the `<real_time_update_rate>`. The default value is 1000, but you can reduce it to 800 or lower. Observe the `<real_time_factor>` during simulation, if it falls below 1.0, you can further adjust the `<real_time_factor>` by reducing it to 0.9 or lower.

+ Navigate to the **quadruped** folder, uncomment any `BLAS` related content in the `CMakeLists.txt` file if they are commented and then set the followihng option to `ON`.
    ```
    option(USE_BLAS   "USE MKL BLAS"     ON)
    ```
+ To boost the performance of a program running on an Intel chip, consider using the MKL library. To install MKL, download the Intel oneAPI Math Kernel Library Offline Installer. After installation, execute the following command prior to running `catkin_make` on your workspace.
    ```
    source /opt/intel/oneapi/setvars.sh
    ```

# 4. Run the Project in Gazebo Simulator

## Step 1: Set up the ROS environment
To set up the ROS development environment, execute the following command.
```
source ${your_workspace}/devel/setup.bash
```
**Note that this command must be executed for each new terminal to set up the environment.**

## Step 2: Start the Gazebo simulator and load a robot
Run the Gazebo simulator
```
roslaunch qr_gazebo gazebo_startup.launch wname:=earth
```
The `wname` (optional) parameter specifies the world to be loaded in Gazebo. The default value is `earth`.

In a new terminal, spawn a robot model and manage controllers in the simulation environment.
Below is for loading a Unitree A1 robot.
```
roslaunch qr_gazebo model_spawn.launch rname:=a1 use_xacro:=true use_camera:=false
```
Below is for loading a DeepRobotics Lite3 robot.
```
roslaunch qr_gazebo model_spawn.launch rname:=lite3 use_xacro:=true use_camera:=false
```
The `rname` (optional) parameter specifies the robot to be loaded in Gazebo. The default value is `a1`.

The `use_xacro` (optional) parameter determines whether to load the model by `xacro` or `urdf`. The default value is `true`.

The `use_camera` (optional) parameter controls whether to load camera model with the robot. The default value is `false`.

**Note that, after executing the command above, press the `Enter` key to start a robot’s controller. The low-level controller is now active and ready to support the movement of a quadruped robot.**

## Step 3: Run an example
Here is an example to control Unitree robot A1 (or DeepRobotics Lite3) to move around. Please check the `user_parameters.yaml` file in the **quadruped/config** folder is properly configured for the robot.

To run an example, open a new terminal and execute
```
rosrun examples example_a1_sim 
```
If you want to switch from robot A1 to Lite3, modify the parameters in the `user_parameters.yaml` file and ensure they are set for the new robot. Then execute
```
rosrun examples example_lite3_sim 
```

# 5. Control A Robot to Move Around using Keyboard

## Step 1: Run a keyboard node
Open a new terminal and run the keyboard node
```
rosrun examples example_keyboard
```

## Step 2: Control robot by keyboard
Control the quadruped robot using the following keys.

`K` : switch control mode

`J` : change the gait

`W` `A` `S` `D` : move forward, left, backward, right

`Q` `E` : rotate left, right

`L` : stop troting

`I` : sit down and exit

# 6. Control A Robot to Move Around using Joystick

Utilizing the ROS joy package, you can operate a quadruped robot by using a game joystick (e.g., Logitech F710) to send speed commands.

Once you launch the joy node, activate joy-control mode by pressing the `A` button, then maneuver the joystick handle to control the robot to move.

## Step 1: Install the ROS dependencies
Install the joystick dependencies in ROS before using joystick to control the robot
```
sudo apt install ros-${your_ros_version}-joy
```

## Step 2: Run a joy_node in ROS
Run a joy_node to receive joy’s command
```
rosrun joy joy_node
```

## Step 3: Control robot by handle
Control the quadruped robot using joystick handle.

`A` : switch joy-control mode

`X` : change the gait

Joystick : control robot's movement

`B` : stop troting

`Y` : sit down and exit

# 7. Run on Real Robot

Note that, if your code is running on an ARM architecture board (e.g. DeepRobotics Lite3), please navigate to the **quadruped** folder and add the following commands in the `CMakeLists.txt` file. 
```
set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
```

## Step 1: Start a ROS master
Launch a ROS master node
```
roscore
```

## Step 2: Run an example
To run an example code on a real robot (A1), open a new terminal and execute
```
rosrun examples example_a1_real
```
Or run the code on the Lite3
```
rosrun examples example_lite3_real
```

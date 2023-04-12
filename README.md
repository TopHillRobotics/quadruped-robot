# 1. Build the project

Navigate to your workspace
```
cd ${your_workspace}
```
Build the project using ROS tool `catkin_make` 
```
catkin_make
```
If you have a less powerful machine, there are a few steps you can take to optimize the project performance.
1. To improve simulation performance, navigate to the `.world` file (the default file is `earth.world`) located in the **simulation/qr_gazebo/worlds** folder and adjust the `<real_time_update_rate>`. The default value is 1000, but you can reduce it to 800 or lower. <real_time_factor>1.0</real_time_factor>
2. Navigate to the **quadruped** folder, uncomment any `BLAS` related content in the `CMakeLists.txt` file if they are commented and then switch the followihng option to `ON`. 
```
option(USE_BLAS   "USE MKL BLAS"     ON)
```
3. To boost the performance of a program running on an Intel chip, consider using the MKL library. To install MKL, download the Intel oneAPI Math Kernel Library Offline Installer. After installation, execute the following command prior to running `catkin_make` on your workspace.
```
source /opt/intel/oneapi/setvars.sh
```

# 2. Run the simulation

## Step 1: Set up the ROS environment
Source `setup.bash` to set up the ROS development environment
```
source ${your_workspace}/devel/setup.bash
```

## Step 2: Start simulation environment
Run the Gazebo simulator with the command below:
```
roslaunch qr_gazebo gazebo_startup.launch wname:=earth
```
The `wname` (optional) parameter specifies the world to load in Gazebo. The default value is set to `earth`.

In a new terminal, using command to spawn model and manage controllers in the simulation environment:
```
roslaunch qr_gazebo model_spawn.launch rname:=a1 use_xacro:=true use_camera:=false
```
The `rname` (optional) parameter specifies the robot to load in Gazebo. The default value is set to `a1`.

The `use_xacro` (optional) parameter specifies whether to load the model by xacro or urdf. The default value is set to `true`.

The `use_camera` (optional) parameter specifies whether to load camera model with the robot. The default value is set to `false`.

**Before running an example, remember to press the `Enter` key to start the robot’s controllers.**

## Step 3: Run an example
Now in another new terminal, you can run the example, like
```
rosrun examples example_a1_sim
```
If you want to switch A1 to Lite3, please check files, especially user_parameters.yaml,  in **config**  folder. Some parameters should be changed to fit the robot.

# 3. Run on Real Robot

## Step 1: Start a ROS master
Use ROS by running a ROS master node with command:
```
roscore
```

## Step 2: Run an example
In a new terminal, source `setup.bash` to set up the ROS development environment
```
source ${your_workspace}/devel/setup.bash
```
and then, you can run the example, like
```
rosrun examples example_a1_real
```

# 4. Joy Control

In this branch, we do not provide some simple examples such as standing up and sitting down. We recommand users to run and debug with a joystick. With the ROS joy package, you can use a gamepad (such as Logitech F710) to send speed commands to control a quadruped robot. After opening the joy node, press the `A` key on the handle to switch joy-control mode, then manipulate the handle to control the robot.

## Step 1: Install the ROS dependencies
Make sure you have installed the joystick dependencies in ROS before using joystick to control the robot.
```
sudo apt install ros-${your_ros_version}-joy
```

## Step 2: Run a joy_node in ROS
Run a joy_node to receive joy’s command with the command below:
```
rosrun joy joy_node
```

## Step 3: Control robot by handle
Now you can manipulate the handle to control the quadruped robot.

Press the `A` key to switch joy-control mode;

Press the `X` key to change the gait;

Push the joystick to control its movement;

Press the `B` key to stop trot;

Press the `Y` key to sit down and exit;

# 5. Keyboard Control

If you want to use the keyboard to control the robot.

## Step 1: Run a keyboard node
Since the keyboard example is customized, you need to source `setup.bash` in the new terminal before running it to ensure that the necessary environment has been set up.
```
source ${your_workspace}/devel/setup.bash
```
Then, run our keyboard node with command
```
rosrun examples example_keyboard
```

## Step 2: Control robot by keyboard
Now you can control the quadruped robot by pressing keys on the keyboard.

Press the `K` key to switch control mode;

Press the `J` key to change the gait;

Press the `W` `A` `S` `D` keys to control its movement;

Press the `Q` `E` keys to control its rotation;

Press the `L` key to stop trot;

Press the `I` key to sit down and exit;

# 6. Code Structure

The code is quite different from that in main branch. The process structure will seems like:

<div align="center">
    <img src="./img/mpc-wbc process diagram.png">
</div>

Currently, the code is tested on Unitree A1 and Deeprobotics Lite3 robots. If you wanna test it on your own robot, you may need to adjust the parameters in **quadruped/config** folder. If you wanna fine-tune the MPC-WBC algorithm, you will need to adjust some parameters(usually KP and KD in **quadruped/controllers/task_set** and weights in WBC locomotion controller).


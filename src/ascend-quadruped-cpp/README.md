
## Step 1: install the third party dependencies:
* Eigen3
* yaml-cpp
* lcm
* Quadprogpp

```
apt install libyaml-cpp-dev
apt install libeigen3-dev
cd ${ROBOTS}/src/ascend-quadruped-cpp/third_party/lcm-1.4.0
mkdir build && cd build
cmake ..
```

## Step2: edit compile option.
open the `CMakeLists.txt` file in `ascend-quadruped-cpp` folder, check the compile options matches the hardware and applications. In particular,  when compiling unitree_legged_skd, the binary `.so` file should be `*amd64.so` in ./lib/ subdirectory, according to current X86/AMD64 platform.


## Step 3: compile the exectuable locomotion controller or other ros packages
```
cd ${ROBOTS}/src/ascend-quadruped-cpp/
mkdir build && cd build
cmake ..
```
or in ROS env,
```
cd ${ROBOTS_DIR}
catkin_make
```
Currently, the locomotion controller support Velocity Mode and Position Mode, as you can browse through `src/ascend-quadruped-cpp/config` directory to select corresponding configuration.


## Step 4: run the controller
You need not to login into subdirectories to launch exec file, instead input the commands
```
roslaunch unitree_gazebo normal.launch
```
to launch gazebo simulation env,
the in another new terminal, launch the controller node,
```
rosrun ascend_quadruped_cpp a1_sim
```
Remember always run 
```
source ./devel/setup.bash
```
before launch local ROS nodes.


## Step 5 (Option): Advances
* Visualization
  
    `xpp` is a ROS package for legged roobt visualization, that wraping Rviz internally.
    We adopt it for unitree A1 robot. 
    To start it, run following command in terminal:
    ```
    roslaunch xpp_example a1_bag.launch
    ```
* Realsense Camera
  
  In `normal.launch` file, you can determines wheather to use camera or not, by setting `use_camera` param true or false.



If you have any problems about this repository, pls report them to Yijie Zhu(z00592961).

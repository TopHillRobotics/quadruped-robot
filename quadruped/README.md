The program has been tested on Ubuntu18(gazebo9) and Ubuntu20(gazebo11). Make sure that you computer computation resource is enough. The cost time of running a while loop in the main function(e.g., in `exec/test.cpp`) indicates the information. In gazebo simulation, if the cost time is smaller than 1 ms, the controller is able to run in realtime. If it is larger than 1 ms but smaller than 3ms, the controller runs successfully under ros time, which is slowed down. For example, to keep the controller making action with frequency at 500 Hz(in simulation time measurement), while the loop cost time (in realworld time measurement) is 2.5 ms, you need to set the `real_time_update_rate` in `.world` configuration file of gazebo to be `1000/2.5 = 400`, and make `useRosTime` true.

## Step 1: install the third party dependencies:
* Eigen3
* yaml-cpp
* lcm
* Quadprogpp
* ros

```
apt install libyaml-cpp-dev
apt install libeigen3-dev
apt install libglm-dev
git clone https://gitee.com/zhulinsen1/robots.git ./robots
cd ${ROBOTS_DIR}/src/ascend-quadruped-cpp/third_party/lcm-1.4.0
mkdir build && cd build
cmake .. && make && sudo make install
sudo ldconfig

# install ros, related ros packages and gazebo

# After that, to install ros-gazebo packages, run the following command
sudo apt update && apt install ros-{version_name}-catkin ros-{version_name}-gazebo* 
```

## Step2: edit compile option.
open the `CMakeLists.txt` file in `ascend-quadruped-cpp` folder, check the compile options so as to match the hardware and applications. In particular, when compiling unitree_legged_skd, the binary `.so` file should be `*amd64.so` in ./lib/ subdirectory, according to current X86/AMD64 platform. If you do not want to compile test examples, please disable `ENABLE_TEST`.


## Step 3: compile the exectuable locomotion controller or other ros packages
```
cd ${ROBOTS_DIR}/src/ascend-quadruped-cpp/
mkdir build && cd build
cmake .. && make
```
or in ROS env (recommended),
```
cd ${ROBOTS_DIR}
catkin_make
```
Currently, the locomotion controller support Velocity Mode(normal trot), Position Mode(for walk through gaps), Walk Mode(static walk) and Advanced-trot Mode(for climb up stairs), among which you can browse through `src/ascend-quadruped-cpp/config` directory to select corresponding configuration. There are several terrains, such as slope of 15 degree, slope of 30 degree, stairs of 10cm height. You can chose each of them by modifying the value of variable `wname` in `qr_gazebo/launch/normal.launch`. To load the slope terrain, you need to append substring `${ROBOT_DIR}/src/simulation/qr_gazebo/worlds/building_editor_models` to the env variable `GAZEBO_MODEL_PATH`.

In a word, the table1 list gaits for different terrains.

| gait   |  locomotion mode |    stance time(s)      |  duty factor |  terrian |
|--------|:----------------:|:-------------:|:-------------:|---------:|
| trot   |  velocity        |      0.3             |     0.6     | flat ground, slope15 |
| trot   | position         |      0.3             |    0.6      | plum piles |
| walk | position           |   7.5      |   0.75     | flat ground, slope15, stairs|
| advanced trot | velocity |    0.5     |    0.7     | flat ground, slope15, stairs|


## Step 4: run the controller
You need not to login into subdirectories to launch exec file, instead input the commands
```
roslaunch qr_gazebo gazebo_startup.launch
```
to launch gazebo simulation env.

And in another new terminal, use command
```
roslaunch qr_gazebo model_spawn.launch
```
to spawn model and manage controllers in the simulation env.

Then in a new terminal, launch the controller node,
```
rosrun ascend_quadruped_cpp sim
```
Remember always run 
```
source ./devel/setup.bash
```
before launch local ROS nodes.


## Step 5 (Option): Advances
* Joy Control

    With ROS joy package, we can use gamepad (such as Logitech F710) to send speed command to control quadruped. Press Key `A` to switch to joy-control mode after open JOY Node. Then manipulate handles to control the quadruped.

* Visualization

    `xpp` is a ROS package for legged roobt visualization, that wraping Rviz internally.
    We adopt it for unitree A1 robot. 
    To start it, run following command in terminal:
    ```
    roslaunch xpp_examples a1_bag.launch
    ```

* Realsense Camera

    In `model_spawn.launch` file, you can determines wheather to use camera or not, by setting `use_camera` param true or false.



If you have any problems about this repository, pls contact with Yijie Zhu(zhuyijie2@hisilicon.com).

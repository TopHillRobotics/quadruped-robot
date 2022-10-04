// The BSD License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <thread>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>

void run_publish_odom() {
    int run_publish_odom = system(std::string("rosrun demo demo_publish_odom").c_str());
}
int main(int argc, char **argv)
{
    // initialize ROS nodes
    ros::init(argc, argv, "demo_slam_gmapping");
    ros::NodeHandle nh;

   // confirm the environment
    std::string pathTOgmapping = ros::package::getPath("gmapping");
   
    bool  if_use_lidar;
    nh.getParam("if_use_lidar", if_use_lidar);

    if (pathTOgmapping.empty()) {
        std::cout << "--------- Please Install Gmapping Package  ---------" << std::endl;
        return false;
    }

    if (if_use_lidar == false) {
        std::cout <<"--------- Please Set Up use_lidar:=true ----------" << std::endl;
        return false;
    }

   //publish odom
    std::thread run_odom(run_publish_odom);
    run_odom.detach();

    sleep(6);
    int launch_gmapping = system(std::string("roslaunch slam gmapping_demo.launch").c_str());
    return 0;
}
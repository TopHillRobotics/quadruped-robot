// The MIT License

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
#include <typeinfo>
#include "quadruped/exec/runtime.h"
#include "quadruped/robots/qr_robot_sim.h"
#include "quadruped/robots/qr_robot_real.h"
#include "quadruped/ros/qr_gazebo_controller_manager.h"

int main(int argc, char **argv)
{
    // initialize ROS nodes
    ros::init(argc, argv, "demo_helloworld");
    ros::NodeHandle nh;

    // get the node package path
    std::string pathToPackage = ros::package::getPath("demo");
    std::string pathToNode =  pathToPackage + ros::this_node::getName();
    std::string robotName = "a1";
    qrRobot *quadruped;
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    nh.setParam("isSim", true);
    
    if(argc == 1 || (argc == 2 && std::string(argv[1]) == "sim")) {
        nh.getParam("robotName", robotName);

        // reset the gazebo controller and robot
        ResetRobotBySystem(nh, robotName);
        ROS_INFO("---------finished: ROS, Gazebo controller and loading robot model---------");
        
        // create a quadruped robot.
        quadruped = new qrRobotSim(nh, robotName, LocomotionMode::VELOCITY_LOCOMOTION);
    
    } else if(argc == 2 && std::string(argv[1]) == "real"){
        nh.setParam("isSim", false);
        quadruped = new qrRobotReal(robotName, LocomotionMode::VELOCITY_LOCOMOTION);
    }

    quadruped->ReceiveObservation();

    // perform the first action: standing up
    // It takes 3 seconds to stand up and keep 5 seconds before any other action
    // 0.0001 is the specified time step.
    Action::StandUp(quadruped, 3.f, 5.f, 0.001f);

    float startTime = quadruped->GetTimeSinceReset();
    float currentTime = startTime;
    float startTimeWall = startTime;

    // keep the quadruped robot standing for 20.0 seconds and 0.001 is the time step
    Action::KeepStand(quadruped, 5.f, 0.001f);
    
    // let the quadruped robot sit down. It takes 3 seconds to finish the action.
    Action::SitDown(quadruped, 3.f, 0.001f);

    // shutdown all the ROS nodes
    ROS_INFO("The demo is closed!");
    ros::shutdown();

    return 0;
}

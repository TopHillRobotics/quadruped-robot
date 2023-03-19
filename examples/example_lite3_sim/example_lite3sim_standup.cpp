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

#include <ros/package.h>

#include "quadruped/exec/qr_robot_runner.h"
#include "quadruped/ros/qr_control2gazebo_msg.h"
#include "quadruped/action/qr_action.h"

using namespace std;
using namespace Quadruped;


int main(int argc, char **argv)
{

    std::string homeDir = ros::package::getPath("quadruped") + "/";

    ros::init(argc, argv, "lite3sim_standup");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");

    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();

    std::cout << "---------ROS node init finished---------" << std::endl;

    qrRobot *quadruped = new qrRobotSim(nh, privateNh, "lite3", homeDir);

    std::cout << "robot created........" << std::endl;

    quadruped->Step(Eigen::Matrix<float,5,12>::Zero(), HYBRID_MODE);

    quadruped->ReceiveObservation();

    std::cout << "BaseOrientation:\n" << quadruped->GetBaseOrientation().transpose() << std::endl;

    // ros::Rate loop_rate(round(1.0 / quadruped->timeStep)); // 500--1000 Hz
    ros::Rate loop_rate1(1000);
    ros::Rate loop_rate2(500);

    float standUpTime = 5;
    float totalTime = 6;
    float timeStep = 0.001;

    // standing up
    qrTimer timer;
    float startTime = timer.GetTimeSinceReset();// robot->GetTimeSinceReset();
    float endTime = startTime + standUpTime;

    Eigen::Matrix<float, 12, 1> motorAnglesBeforeStandUP = quadruped->GetMotorAngles();
    std::cout << "motorAnglesBeforeStandUP: \n" << motorAnglesBeforeStandUP.transpose() << std::endl;
    std::cout << "---------------------Standing Up---------------------" << std::endl;
    std::cout << "robot->standMotorAngles: \n" << quadruped->standUpMotorAngles.transpose() << std::endl;
    for (float t = startTime; t < totalTime; t += timeStep) {
        float blendRatio = (t - startTime) / standUpTime;
        Eigen::Matrix<float, 12, 1> action;
        if (blendRatio < 1.0f) {
            action = blendRatio * quadruped->standUpMotorAngles + (1 - blendRatio) * motorAnglesBeforeStandUP;
            quadruped->Step(action, MotorMode::POSITION_MODE);
            while (timer.GetTimeSinceReset() < t + timeStep) {}
        } else {
            quadruped->Step(action, MotorMode::POSITION_MODE);
            while (timer.GetTimeSinceReset() < t + timeStep) {}
        }
    }

    std::cout << "robot->GetMotorAngles: \n" << quadruped->GetMotorAngles().transpose() << std::endl;
    std::cout << "---------------------Stand Up Finished---------------------" << std::endl;


    // if (count > 20000) {
    //     quadruped->stateDataFlow.visualizer.Show();
    // }

    ros::shutdown();
    return 0;
}


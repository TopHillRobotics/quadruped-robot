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

#include "action/qr_action.h"

#include <Eigen/Dense>
#include <fstream>

static const Eigen::Matrix<float, 3, 1> standUpAnglesConfig = {0.0f, 0.9f, -1.8f};

static const Eigen::Matrix<float, 3, 1> sitDownAnglesConfig = {-0.167136f, 0.934969f, -2.54468f};

Eigen::Matrix<float, 12, 1> LegAngles(qrRobotConfig *robotConfig, Eigen::Matrix<float, 3, 1> config){
    Eigen::Matrix<float, 12, 1> angles;
    float abAngle = 0.0f;
    float hipAngle = std::acos(robotConfig->GetBodyHeight() / 2.0f / robotConfig->GetUpperLength());
    float kneeAngle = -2.0f * hipAngle;
    Eigen::Matrix<float, 3, 1> singleAngles(abAngle, hipAngle, kneeAngle);
    std::cout << robotConfig->GetBodyHeight() << " " <<robotConfig->GetUpperLength()<< std::endl;
    std::cout << abAngle << " " << hipAngle<< " " << kneeAngle << std::endl;

    angles << singleAngles, singleAngles, singleAngles, singleAngles;
    return angles;
}

void StandUp(qrRobot *robot, float standUpTime, float totalTime, float timeStep) {
    float startTime = robot->GetTimeSinceReset();
    float endTime = startTime + standUpTime;

    robot->Observation();
    Eigen::Matrix<float, 12, 1> motorAnglesBeforeStandUP = robot->GetRobotState()->q;
    Eigen::Matrix<float, 12, 1> motorAnglesAfterStandUP  = LegAngles(robot->GetRobotConfig(), standUpAnglesConfig);
    std::cout << "motorAnglesBeforeStandUP: \n" << motorAnglesBeforeStandUP.transpose() << std::endl;
    std::cout << "---------------------Standing Up---------------------" << std::endl;
    std::cout << "robot->standMotorAngles: \n" << motorAnglesAfterStandUP.transpose() << std::endl;

    std::ofstream ofs;
    ofs.open("/home/gk/Documents/angles.data",std::ios::out|std::ios::app);
    for (float t = startTime; t < totalTime; t += timeStep) {
        float blendRatio = (t - startTime) / standUpTime;
        Eigen::Matrix<float, 12, 1> action;
        if (blendRatio < 1.0f) {
            action = blendRatio * motorAnglesAfterStandUP + (1 - blendRatio) * motorAnglesBeforeStandUP;
            robot->ApplyAction(action,MotorMode::POSITION);
            while (robot->GetTimeSinceReset() < t + timeStep) {}
        } else {
            robot->ApplyAction(action,MotorMode::POSITION);
            while (robot->GetTimeSinceReset() < t + timeStep) {}
        }
        ofs << t << " ";
        for(int i = 0; i < 12; ++i){
            ofs << action[i] << " ";
        }
        ofs << '\n';
    }
    ofs.close();
    std::cout << "---------------------Stand Up Finished---------------------" << std::endl;
}

void SitDown(qrRobot *robot, float sitDownTime, float timeStep) {
    float startTime = robot->GetTimeSinceReset();
    float endTime = startTime + sitDownTime;

    robot->Observation();
    Eigen::Matrix<float, 12, 1> motorAnglesBeforeSitDown = robot->GetRobotState()->q;
    Eigen::Matrix<float, 12, 1> motorAnglesAfterStandUP  = LegAngles(robot->GetRobotConfig(), sitDownAnglesConfig);

    std::cout << "motorAnglesBeforeSitDown: \n" << motorAnglesBeforeSitDown.transpose() << std::endl;
    std::cout << "robot->sitDownMotorAngles: \n" << motorAnglesAfterStandUP.transpose() << std::endl;

    for (float t = startTime; t < endTime; t += timeStep) {
        float blendRatio = (t - startTime) / sitDownTime;
        Eigen::Matrix<float, 12, 1> action;
        action = blendRatio * motorAnglesAfterStandUP + (1 - blendRatio) * motorAnglesBeforeSitDown;
        robot->ApplyAction(action,MotorMode::POSITION);
        while (robot->GetTimeSinceReset() < t + timeStep) {}
    }
}

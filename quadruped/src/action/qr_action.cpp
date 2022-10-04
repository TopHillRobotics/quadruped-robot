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

namespace Action {

    void StandUp(qrRobot *robot, float standUpTime, float totalTime, float timeStep)
    {
        float startTime = robot->GetTimeSinceReset();
        float endTime = startTime + standUpTime;
        Eigen::Matrix<float, 12, 1> motorAnglesBeforeStandUP = robot->GetMotorAngles();
        for (float t = startTime; t < totalTime; t += timeStep) {
            float blendRatio = (t - startTime) / standUpTime;
            Eigen::Matrix<float, 12, 1> action;
            if (blendRatio < 1.0f) {
                action = blendRatio * robot->config->standUpMotorAngles + (1 - blendRatio) * motorAnglesBeforeStandUP;
                robot->Step(action, MotorMode::POSITION_MODE);
                while (robot->GetTimeSinceReset() < t + timeStep) {}
            } else {
                robot->Step(action, MotorMode::POSITION_MODE);
                while (robot->GetTimeSinceReset() < t + timeStep) {}
            }
            // std::cout << "motorAngles: \n" << robot->GetMotorAngles().transpose() << std::endl;
        }
        std::cout << "---------------------Stand Up Finished---------------------" << std::endl;
    }

    void SitDown(qrRobot *robot, float sitDownTime, float timeStep)
    {
        float startTime = robot->GetTimeSinceReset();
        float endTime = startTime + sitDownTime;
        Eigen::Matrix<float, 12, 1> motorAnglesBeforeSitDown = robot->config->standUpMotorAngles;
        std::cout << "motorAnglesBeforeSitDown: \n" << motorAnglesBeforeSitDown.transpose() << std::endl;
        std::cout << "robot->sitDownMotorAngles: \n" << robot->config->sitDownMotorAngles.transpose() << std::endl;

        for (float t = startTime; t < endTime; t += timeStep) {
            float blendRatio = (t - startTime) / sitDownTime;
            Eigen::Matrix<float, 12, 1> action;
            action = blendRatio *  robot->config->sitDownMotorAngles + (1 - blendRatio) * motorAnglesBeforeSitDown;
            robot->Step(action, MotorMode::POSITION_MODE);
            while (robot->GetTimeSinceReset() < t + timeStep) {}
        }
    }

    void KeepStand(qrRobot *robot, float KeepStandTime, float timeStep)
    {
        float startTime = robot->GetTimeSinceReset();
        float endTime = startTime + KeepStandTime;
        Eigen::Matrix<float, 12, 1> motorAnglesBeforeKeepStand =  robot->config->standUpMotorAngles;
        Eigen::Matrix<float, 12, 1> motorAngles;
        for (float t = startTime; t < endTime; t += timeStep) {
            motorAngles = motorAnglesBeforeKeepStand;
            
            robot->Step(motorAngles, MotorMode::POSITION_MODE);
            // std::cout << "motorAngles: \n" << robot->GetMotorAngles().transpose() << std::endl;
            while (robot->GetTimeSinceReset() < t + timeStep) {}
        }
    }

    void ControlFoot(qrRobot *robot, qrLocomotionController *locomotionController, float walkTime, float timeStep)
    {
        const float FREQ = 1;   
        float startTime = robot->GetTimeSinceReset();
        float currentTime = startTime;
        float startTimeWall = startTime;
        float total_time = 0.f;
        int cycles = 1000;
        long long i = 0;

        float endTime = startTime + walkTime;
        Eigen::Matrix<float, 12, 1> motorAnglesBeforeWalk = robot->GetMotorAngles();
        std::cout << "motorAnglesBeforeWalk: \n" << motorAnglesBeforeWalk.transpose() << std::endl;
        Eigen::Matrix<float, 12, 1> action;
        
        Vec3<int> jointIdx;
        Vec3<float> jointAngles;
        Vec3<float> footTargetPosition;

        Eigen::Matrix<float, 3, 4> hipPositions = robot->config->defaultHipPosition;
    
        for (int legId=0; legId < 4; ++legId) {
            Vec3<float> hipOffset = hipPositions.col(legId);
            
            footTargetPosition << hipOffset[0], hipOffset[1], -robot->config->bodyHeight;
            robot->config->ComputeMotorAnglesFromFootLocalPosition(legId, footTargetPosition, jointIdx, jointAngles);
            for (int i = 0; i < 3; ++i) {
                action[jointIdx[i]] = jointAngles[i];
            }
        }
        
        while (currentTime - startTime < walkTime) {
            startTimeWall = robot->GetTimeSinceReset();
            locomotionController->Update();
            robot->Step(action, MotorMode::POSITION_MODE);
            Eigen::Matrix<float, 12, 1> motor_angles = robot->GetMotorAngles();
            currentTime = robot->GetTimeSinceReset();
            total_time += currentTime - startTimeWall;
            i += 1;
            if (i % cycles == 0) {
                printf("time = %f (ms)\n", total_time/cycles*1000.f);
                total_time = 0;
            }
            while (robot->GetTimeSinceReset() - startTimeWall < robot->timeStep) {}
        }
    }
} // Action
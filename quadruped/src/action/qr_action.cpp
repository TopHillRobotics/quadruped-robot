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

namespace Quadruped {

namespace Action {

void StandUp(qrRobot *robot, float standUpTime, float totalTime, float timeStep)
{
    qrTimer timer;
    float startTime = timer.GetTimeSinceReset();// robot->GetTimeSinceReset();
    float endTime = startTime + standUpTime;
    Eigen::Matrix<float, 12, 1> motorAnglesBeforeStandUP = robot->GetMotorAngles();
    std::cout << "motorAnglesBeforeStandUP: \n" << motorAnglesBeforeStandUP.transpose() << std::endl;
    std::cout << "---------------------Standing Up---------------------" << std::endl;
    std::cout << "robot->standMotorAngles: \n" << robot->standUpMotorAngles.transpose() << std::endl;
    Eigen::Matrix<float, 12, 1> action;
    Eigen::Matrix<float, 12, 1> currentAngles = robot->GetMotorAngles();

    Visualization2D& vis = robot->stateDataFlow.visualizer;

  for (float t = startTime; t < totalTime; t += timeStep) {
        float blendRatio = (t - startTime) / standUpTime;

        if (blendRatio < 1.0f) {
            action = blendRatio * robot->standUpMotorAngles + (1 - blendRatio) * motorAnglesBeforeStandUP;
            robot->Step(action, MotorMode::POSITION_MODE);
            // vis.datay5.push_back(action[2]);
            //currentAngles = robot->GetMotorAngles();

             /// stand up by torque
             //action = 60.0f * (action - currentAngles) + 0.45f * (- robot->GetMotorVelocities());
             //action = action.cwiseMax(-15.0f).cwiseMin(15.0f);
             //obot->Step(action, MotorMode::TORQUE_MODE);

            /*
        vis.datax.push_back(t);
            vis.datay1.push_back(currentAngles[0]);
            vis.datay2.push_back(currentAngles[1]);
            vis.datay3.push_back(currentAngles[2]);
            vis.datay4.push_back(action[0]);
            //vis.datay5.push_back(action[1]);
            vis.datay6.push_back(action[2]);
            */
        } else {
            robot->Step(robot->standUpMotorAngles, MotorMode::POSITION_MODE);
        }
    while (timer.GetTimeSinceReset() < t + timeStep) {}
    }
    std::cout << "robot->GetMotorAngles: \n" << robot->GetMotorAngles().transpose() << std::endl;
    std::cout << "---------------------Stand Up Finished---------------------" << std::endl;
}


void SitDown(qrRobot *robot, float sitDownTime, float timeStep)
{
    robot->ReceiveObservation();
    float startTime = robot->GetTimeSinceReset();
    float endTime = startTime + sitDownTime;
    Eigen::Matrix<float, 12, 1> motorAnglesBeforeSitDown = robot->GetMotorAngles();
    std::cout << "motorAnglesBeforeSitDown: \n" << motorAnglesBeforeSitDown.transpose() << std::endl;
    std::cout << "robot->sitDownMotorAngles: \n" << robot->sitDownMotorAngles.transpose() << std::endl;
    std::cout << "---------------------Sit down ---------------------" << std::endl;

    for (float t = startTime; t < endTime; t += timeStep) {
        float blendRatio = (t - startTime) / sitDownTime;
        Eigen::Matrix<float, 12, 1> action;
        action = blendRatio * robot->sitDownMotorAngles + (1 - blendRatio) * motorAnglesBeforeSitDown;
        robot->Step(action, MotorMode::POSITION_MODE);
        while (robot->GetTimeSinceReset() < t + timeStep) {}
    }
    std::cout << "---------------------Sit down Finished---------------------" << std::endl;
}


void KeepStand(qrRobot *robot, float KeepStandTime, float timeStep)
{
    float startTime = robot->GetTimeSinceReset();
    float endTime = startTime + KeepStandTime;
    // record current motor angles.
    Eigen::Matrix<float, 12, 1>
        motorAnglesBeforeKeepStand = robot->standUpMotorAngles;
    Eigen::Matrix<float, 12, 1> motorAnglesAfterKeepStand = motorAnglesBeforeKeepStand;
    motorAnglesAfterKeepStand[3] = 0.;
    motorAnglesAfterKeepStand[4] = 1.2;
    motorAnglesAfterKeepStand[5] = -2.4;
    Eigen::Matrix<float, 12, 1> motorAngles;
    for (float t = startTime; t < endTime; t += timeStep) {
        float blendRatio = (t - startTime) / KeepStandTime;
        motorAngles = blendRatio * motorAnglesAfterKeepStand + (1 - blendRatio) * motorAnglesBeforeKeepStand;

        robot->Step(motorAngles, MotorMode::POSITION_MODE);
        while (robot->GetTimeSinceReset() < t + timeStep) {}
    }
}


void ControlFoot(qrRobot *robot, qrLocomotionController *locomotionController, float walkTime, float timeStep)
{
    const float FREQ = 1;
    qrTimer timer;
    float startTime = timer.GetTimeSinceReset();
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

    Eigen::Matrix<float, 3, 4> hipPositions = robot->GetDefaultHipPosition();

    for (int legId=0; legId < 4; ++legId) {
        Vec3<float> hipOffset = hipPositions.col(legId);

        footTargetPosition << hipOffset[0], hipOffset[1], -robot->bodyHeight;
        robot->ComputeMotorAnglesFromFootLocalPosition(legId, footTargetPosition, jointIdx, jointAngles);
        for (int i = 0; i < 3; ++i) {
            action[jointIdx[i]] = jointAngles[i];
        }
    }
    Visualization2D& vis = robot->stateDataFlow.visualizer;

    while (currentTime - startTime < walkTime) {
        startTimeWall = timer.GetTimeSinceReset();
        // locomotionController->Update();
        // Move the legs in a sinusoidal curve
        // float angle_hip = 0.8 + 0.2 * std::sin(M_2PI * FREQ * (startTimeWall-startTime));
        // float angle_calf = -2 * angle_hip;
        // action <<   0., angle_hip, angle_calf,
        //             0., angle_hip, angle_calf,
        //             0., angle_hip, angle_calf,
        //             0., angle_hip, angle_calf;
        float angle_calf = motorAnglesBeforeWalk[5]  + 0.2 * std::sin(M_2PI * FREQ * (startTimeWall-startTime));
        float angle_hip = motorAnglesBeforeWalk[1] + 0.2 * std::sin(M_2PI * FREQ * (startTimeWall-startTime));

    // float angle_calf = -2 * angle_hip;
        action <<   0., angle_hip, angle_calf,
                    0., angle_hip, angle_calf,
                    0., 0, 0,
                    0., 0, 0;
        vis.datay5.push_back(action[1]);

        Eigen::Matrix<float, 12, 1> motor_angles = robot->GetMotorAngles();
        Eigen::Matrix<float, 12, 1> des_vel;
        des_vel.setZero();
        des_vel[1] = M_2PI * FREQ *  0.2 * std::cos(M_2PI * FREQ * (startTimeWall-startTime));
        des_vel[4] = des_vel[1];
        action = 60.0f * (action - motor_angles) + 0.5f * ( des_vel- robot->GetMotorVelocities());
        action = action.cwiseMax(-10.0f).cwiseMin(10.0f);
        robot->Step(action, MotorMode::TORQUE_MODE);
    //robot->Step(action, MotorMode::POSITION_MODE);

        vis.datax.push_back(startTimeWall);
        vis.datay1.push_back(motor_angles[0]);
        vis.datay2.push_back(motor_angles[1]);
        vis.datay3.push_back(motor_angles[2]);
        vis.datay4.push_back(action[0]);
        //vis.datay5.push_back(action[1]);
        vis.datay6.push_back(action[2]);
        printf("t = %f, des_ang = %f, recv_ang = %f\n", startTimeWall,  action[1], motor_angles[1]);
    // auto [hybridAction, qpSol] = locomotionController->GetAction();
        // robot->Step(MotorCommand::convertToMatix(hybridAction), MotorMode::HYBRID_MODE);
        currentTime = timer.GetTimeSinceReset();
        total_time += currentTime - startTimeWall;
        i += 1;
        if (i % cycles == 0) {
            printf("time = %f (ms)\n", total_time/cycles*1000.f);
            total_time = 0;
        }
        // std::cout << "cycle time: " << (currentTime - startTimeWall) * 1000.f << " ms" << std::endl;
        while (timer.GetTimeSinceReset() - startTimeWall < timeStep) {}
    }
}

} // Namespace Action

} // Namespace Quadruped

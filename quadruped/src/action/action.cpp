/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Actions
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-11-3
* Notes: xx
* Modify: init the file. @ Zhao Yao 2021.11.19;
*        add KeepStand @ Zhu Yijie 2021.11.23
*/

#include "action/action.h"

namespace Quadruped {
    namespace Action {

        void StandUp(Robot *robot, float standUpTime, float totalTime, float timeStep)
        {
            Timer timer;
            float startTime = timer.GetTimeSinceReset();// robot->GetTimeSinceReset();
            float endTime = startTime + standUpTime;
            Eigen::Matrix<float, 12, 1> motorAnglesBeforeStandUP = robot->GetMotorAngles();
            std::cout << "motorAnglesBeforeStandUP: \n" << motorAnglesBeforeStandUP.transpose() << std::endl;
            std::cout << "---------------------Standing Up---------------------" << std::endl;
            std::cout << "robot->standMotorAngles: \n" << robot->standUpMotorAngles.transpose() << std::endl;
            for (float t = startTime; t < totalTime; t += timeStep) {
                float blendRatio = (t - startTime) / standUpTime;
                Eigen::Matrix<float, 12, 1> action;
                if (blendRatio < 1.0f) {
                    action = blendRatio * robot->standUpMotorAngles + (1 - blendRatio) * motorAnglesBeforeStandUP;
                    robot->Step(action, MotorMode::POSITION_MODE);
                    while (timer.GetTimeSinceReset() < t + timeStep) {}
                } else {
                    robot->Step(action, MotorMode::POSITION_MODE);
                    while (timer.GetTimeSinceReset() < t + timeStep) {}
                }
            }
            std::cout << "robot->GetMotorAngles: \n" << robot->GetMotorAngles().transpose() << std::endl;
            std::cout << "---------------------Stand Up Finished---------------------" << std::endl;
        }

        void SitDown(Robot *robot, float sitDownTime, float timeStep)
        {
            float startTime = robot->GetTimeSinceReset();
            float endTime = startTime + sitDownTime;
            Eigen::Matrix<float, 12, 1> motorAnglesBeforeSitDown = robot->GetMotorAngles();
            std::cout << "motorAnglesBeforeSitDown: \n" << motorAnglesBeforeSitDown.transpose() << std::endl;
            std::cout << "robot->sitDownMotorAngles: \n" << robot->sitDownMotorAngles.transpose() << std::endl;

            for (float t = startTime; t < endTime; t += timeStep) {
                float blendRatio = (t - startTime) / sitDownTime;
                Eigen::Matrix<float, 12, 1> action;
                action = blendRatio * robot->sitDownMotorAngles + (1 - blendRatio) * motorAnglesBeforeSitDown;
                robot->Step(action, MotorMode::POSITION_MODE);
                while (robot->GetTimeSinceReset() < t + timeStep) {}
            }
        }

        void KeepStand(Robot *robot, float KeepStandTime, float timeStep)
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

        void ControlFoot(Robot *robot, LocomotionController *locomotionController, float walkTime, float timeStep)
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

            Eigen::Matrix<float, 3, 4> hipPositions = robot->GetHipPositionsInBaseFrame();
        
            for (int legId=0; legId < 4; ++legId) {
                Vec3<float> hipOffset = hipPositions.col(legId);
                
                footTargetPosition << hipOffset[0], hipOffset[1], -robot->bodyHeight;
                robot->ComputeMotorAnglesFromFootLocalPosition(legId, footTargetPosition, jointIdx, jointAngles);    
                for (int i = 0; i < 3; ++i) {
                    action[jointIdx[i]] = jointAngles[i];
                }
            }
            
            while (currentTime - startTime < walkTime) {
                startTimeWall = robot->GetTimeSinceReset();
                locomotionController->Update();
                // Move the legs in a sinusoidal curve
                // float angle_hip = 0.7 + 0.2 * std::sin(M_2PI * FREQ * (startTimeWall-startTime));
                // float angle_calf = -2 * angle_hip;
                // action <<   0., angle_hip, angle_calf,
                //             0., angle_hip, angle_calf,
                //             0., angle_hip, angle_calf,
                //             0., angle_hip, angle_calf;       

                robot->Step(action, MotorMode::POSITION_MODE);
                Eigen::Matrix<float, 12, 1> motor_angles = robot->GetMotorAngles();
                // auto [hybridAction, qpSol] = locomotionController->GetAction();
                // robot->Step(MotorCommand::convertToMatix(hybridAction), MotorMode::HYBRID_MODE);
                currentTime = robot->GetTimeSinceReset();
                total_time += currentTime - startTimeWall;
                i += 1;
                if (i % cycles == 0) {
                    printf("time = %f (ms)\n", total_time/cycles*1000.f);
                    total_time = 0;
                }
                // std::cout << "cycle time: " << (currentTime - startTimeWall) * 1000.f << " ms" << std::endl;
                while (robot->GetTimeSinceReset() - startTimeWall < robot->timeStep) {}
            }
        }
    } // Action
} // Quadruped

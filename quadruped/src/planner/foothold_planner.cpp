/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-11-08
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#include "planner/foothold_planner.h"

namespace Quadruped {
    FootholdPlanner::FootholdPlanner(Robot *quadrupedIn, GaitGenerator* gaitGeneratorIn,
                                    StateEstimatorContainer<float>* stateEstimatorsIn,
                                    UserParameters* userParametersIn,
                                    DesiredStateCommand* desiredStateCommandIn)
    : robot(quadrupedIn), gaitGenerator(gaitGeneratorIn), 
      groundEsitmator(stateEstimatorsIn->GetGroundEstimator()), 
      userParameters(userParametersIn), desiredStateCommand(desiredStateCommandIn),
      terrain(groundEsitmator->terrain), timeSinceReset(0.f), stepCount(0)
    {
        footstepper = new FootStepper(terrain, 0.10f, "optimal");
        Reset(0);
    }

    void FootholdPlanner::Reset(float t)
    {
        resetTime = t;
        timeSinceReset = 0.f;
        footstepper->Reset(timeSinceReset);
        comPose << robot->GetBasePosition(), robot->GetBaseRollPitchYaw();
        desiredComPose.setZero();
        desiredFootholdsOffset.setZero();
        if (robot->controlParams["mode"] == LocomotionMode::ADVANCED_TROT) {
            swingKp = Eigen::MatrixXf::Map(&userParameters->swingKp["advanced_trot"][0], 3, 1);
        } else {
            swingKp = Eigen::MatrixXf::Map(&userParameters->swingKp["trot"][0], 3, 1);
        }
        firstSwingBaseState << robot->basePosition, robot->stateDataFlow.baseVInWorldFrame, 
                                robot->baseRollPitchYaw, robot->baseRollPitchYawRate;
    }

    void FootholdPlanner::UpdateOnce(Eigen::Matrix<float, 3, 4> currentFootholds, std::vector<int> legIds)
    {
        // std::cout << "------------------------[FootholdPlanner::UpdateOnce]------------------------" << std::endl;
        // std::cout << "current footholds: \n" << currentFootholds << std::endl;
        comPose << robot->GetBasePosition(), robot->GetBaseRollPitchYaw();
        desiredComPose << 0.f,0.f,0.f,0.f,0.f,0.f; //comPose;
        desiredFootholds = currentFootholds;
        if (legIds.empty()) { // if is empty, update all legs.
            legIds = {0,1,2,3};
        } else {
            std::cout<<"update foothold of Legs : ";
            for(int legId : legIds) {
                std::cout << legId << " ";
            }
            std::cout << "\n";
        }
        
        if (terrain.terrainType != TerrainType::STAIRS) { 
            ComputeFootholdsOffset(currentFootholds, comPose, desiredComPose, legIds);
        } else {
            ComputeNextFootholds(currentFootholds, comPose, desiredComPose, legIds);
        }
        // std::cout << "desired footholds offset: \n" << desiredFootholdsOffset << std::endl;
        // std::cout << "------------------------[FootholdPlanner::UpdateOnce Finished]------------------------" << std::endl;
    }

    Eigen::Matrix<float, 3, 4> FootholdPlanner::ComputeFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds,
                                                                    Eigen::Matrix<float, 6, 1> currentComPose,
                                                                    Eigen::Matrix<float, 6, 1> desiredComPose,
                                                                    std::vector<int> legIds)
    {
        desiredFootholdsOffset = footstepper->GetOptimalFootholdsOffset(currentFootholds);
        return desiredFootholdsOffset;
    }

    Eigen::Matrix<float, 3, 4> FootholdPlanner::ComputeNextFootholds(Eigen::Matrix<float, 3, 4>& currentFootholds,
                                                                        Eigen::Matrix<float, 6, 1>& currentComPose,
                                                                        Eigen::Matrix<float, 6, 1>& desiredComPose,
                                                                        std::vector<int>& legIds)
    {
        auto res = footstepper->GetFootholdsInWorldFrame(currentFootholds, currentComPose, desiredComPose, legIds);
        desiredFootholds = std::get<0>(res);
        desiredFootholdsOffset = std::get<1>(res);
        return desiredFootholdsOffset;
    }

    void FootholdPlanner::ComputeHeuristicFootHold(std::vector<u8> swingFootIds)
    {
        if (swingFootIds.empty()) {
            return;
        }
        Vec3<float> desiredSpeed = desiredStateCommand->stateDes.segment(6, 3);
        float desiredTwistingSpeed = desiredStateCommand->stateDes(11);
        Vec3<float> desiredHeight(0.f,0.f, desiredStateCommand->stateDes(2) - userParameters->footClearance);
        
        Eigen::Matrix<float,3,4> footPositionsInBaseFrame = robot->GetFootPositionsInBaseFrame();
        Eigen::Matrix<float,3,4> footPositionsInWorldFrame = robot->GetFootPositionsInWorldFrame();
        
        Mat3<float> robotBaseR = robot->stateDataFlow.baseRMat;
        Mat3<float> controlFrameR = robot->stateDataFlow.groundRMat;
        Mat3<float> dR = robot->stateDataFlow.baseRInControlFrame;
        // dR.setIdentity();
        Eigen::Matrix<float, 12, 1> currentJointAngles = robot->GetMotorAngles();
        Vec3<float> comVelocity = robot->GetEstimatedVelocityInBaseFrame(); // in base frame
        Vec3<float> w = robot->GetBaseRollPitchYawRate();
        float yawDot = w[2]; // in base frame
        Eigen::Matrix<float,3,4> hipPositions = robot->GetHipPositionsInBaseFrame();
        Eigen::Matrix<float, 3, 4> abadPosInBaseFrame = robot->hipOffset;
        
        for (u8& legId : swingFootIds) {
            Vec3<float> hipOffset = abadPosInBaseFrame.col(legId); // hipPositions.col(legId);
            Vec3<float> twistingVector = {-hipOffset[1], hipOffset[0], 0.f};
            Vec3<float> hipHorizontalVelocity = comVelocity + w.cross(hipOffset); // yawDot * twistingVector; // in base frame
            //  std::cout <<"1. hipHorizontalVelocity = " << hipHorizontalVelocity.transpose() << std::endl; 
            hipHorizontalVelocity = dR * hipHorizontalVelocity; // in control frame
            hipHorizontalVelocity[2] = 0.f;
            Vec3<float> targetHipHorizontalVelocity = desiredSpeed + desiredTwistingSpeed * twistingVector; // in control frame
            // std::cout << "comVelocity = " << comVelocity.transpose() << std::endl;
            
            // std::cout << "targetHipHorizontalVelocity = " << targetHipHorizontalVelocity.transpose() << std::endl;
            // std::cout << "2. hipHorizontalVelocity = " << hipHorizontalVelocity.transpose() << std::endl;
            
            // Vec3<float> hightOffset = desiredHeight + Vec3<float>(0.f,0.f,0.01f); // todo
            Vec3<float> footTargetPosition;
            Vec3<float> footTargetHipPosition;
            
            float hipLen = 0.085;
            float side_sign[4] = {-1, 1, -1, 1}; // y-axis
            // float abad = currentJointAngles[3*legId+0];
            float abad = -robot->baseRollPitchYaw[0];
            footTargetHipPosition[0] = abadPosInBaseFrame(0, legId);
            footTargetHipPosition[1] = abadPosInBaseFrame(1, legId) + side_sign[legId]*hipLen * cos(abad);
            footTargetHipPosition[2] = abadPosInBaseFrame(2, legId) + side_sign[legId]*hipLen * sin(abad);
            // footTargetHipPosition = robotBaseR*footTargetHipPosition;  // in translated world frame;
            Vec3<float> angles = currentJointAngles.segment(3*legId,3);
            angles[0] = 0;
            Vec3<float> footPosInHipFrame = robot->FootPositionInHipFrame(angles,pow((-1), legId + 1));
            
            if (!gaitGenerator->allowSwitchLegState[legId]) {
                // // phaseSwitchFootLocalPos.col(legId) = footPositionsInBaseFrame.col(legId);
                footTargetPosition = robotBaseR *(footPositionsInBaseFrame.col(legId) - hipPositions.col(legId));
                // if (footTargetPosition[0]>0.001){
                //     footTargetPosition[0] -= 0.001;
                // } else if (footTargetPosition[0]<-0.001) {
                //     footTargetPosition[0] += 0.001;
                // }
                // std::cout << "cur angle = " << currentJointAngles[3*legId] << std::endl;
                if (footTargetPosition[1] > 0.01 + 0.00*(-side_sign[legId])) {
                    // std::cout << "s = " << footTargetPosition.transpose() <<" - 0.01"<< std::endl;
                    footTargetPosition[1] -= 0.005; // 0.001
                } else if (footTargetPosition[1]<-0.01 + 0.00 * side_sign[legId]) {
                    footTargetPosition[1] += 0.005;
                    // std::cout << "s = " << footTargetPosition.transpose() << "+ 0.01" << std::endl;
                }
                footTargetPosition[2] -= 0.02; // 0.015
                
                footTargetPosition = robotBaseR.transpose() * footTargetPosition + hipPositions.col(legId);
                // footTargetPosition[1] = footTargetHipPosition[1];
                // std::cout << "[leg i] des hip = " << footPosInHipFrame.transpose() <<std::endl;
                // ++moveDown[legId];
                // footPosInHipFrame[2] = desiredFootholds(2, legId);
                // footTargetPosition = footPosInHipFrame - robotBaseR.transpose() * Vec3<float>(0,0,moveDown[legId]*0.0005);
                // footPositionsInWorldFrame(2, legId) -= 0.0002f;
                // footTargetPosition = robotBaseR.transpose()* (footPositionsInWorldFrame.col(legId) - robot->basePosition);
                // footTargetPosition +=  robot->hipOffset.col(legId);
                phase[legId] = 1.0f;
                // printf("[leg %d] swing, normal phase = %f \n", legId, gaitGenerator->normalizedPhase[legId]);
                
            } else {
                // if (gaitGenerator->firstSwing[legId]) {
                moveDown[legId] = 0;
                Vec3<float> dP;
                float swingRemainTime = gaitGenerator->swingTimeRemaining[legId];
                float rollCorrect = w[0] * 0.3f * swingRemainTime;
                float pitchCorrect = -w[1] * 0.3f * swingRemainTime;
                if (robot->controlParams["mode"]==LocomotionMode::ADVANCED_TROT && false) {
                    dP = dR.transpose() * (targetHipHorizontalVelocity - hipHorizontalVelocity) * gaitGenerator->stanceDuration[legId] / 2.0
                            - swingKp.cwiseProduct(targetHipHorizontalVelocity - hipHorizontalVelocity);
                            // -Vec3<float>(pitchCorrect, rollCorrect, 0)
                } else {
                    // dP = dR.transpose() * (targetHipHorizontalVelocity * gaitGenerator->stanceDuration[legId] / 2.0
                    //         - swingKp.cwiseProduct(targetHipHorizontalVelocity - hipHorizontalVelocity)
                    //     );
                    dP = dR.transpose() * (targetHipHorizontalVelocity * swingRemainTime
                            - swingKp.cwiseProduct(targetHipHorizontalVelocity - hipHorizontalVelocity)
                        );
                }
                // std::cout << "legId = " << legId << ", dP = " << dP.transpose() << std::endl;
                const float dPthresold = 0.2f;
                
                dP = dP.cwiseMin(dPthresold).cwiseMax(-dPthresold); // clip
                // dP.setZero();
                if (dP[1] > 0.15) {
                    dP[1] = 0.15;
                } else if (dP[1] < -0.15) {
                    dP[1] = -0.15;
                }
                dP[2] = 0;
                Vec3<float> rpy = robot->GetBaseRollPitchYaw();
                float interleave_y[4] = {-0.08, 0.08, -0.08, 0.08};
                Mat3<float> rollR = robotics::math::coordinateRotation(CoordinateAxis::X, rpy[0]); // abad->hip offset vector
            
                footTargetPosition = dP
                        + Vec3<float>(hipOffset[0], hipOffset[1], 0)
                        + rollR * Vec3<float>{0,interleave_y[legId],0}
                            //  - robotBaseR.transpose() * desiredHeight;
                            // - desiredHeight;
                        ;
                if (desiredStateCommand->stateDes(6,0)< -0.01) {
                    footTargetPosition(0,2) -= 0.02;
                    footTargetPosition(0,3) -= 0.02;
                }
                if (legId < 2) {
                    footTargetPosition -=  robotBaseR.transpose() * desiredHeight;
                } else {
                    footTargetPosition -=  robotBaseR.transpose() * (desiredHeight /*+ Vec3<float>(0,0,0.02)*/);
                }
                // } else {
                //     footTargetPosition = desiredFootholds.col(legId);
                // }
                phase[legId] = gaitGenerator->normalizedPhase(legId);
            }
            
            desiredFootholds.col(legId) = footTargetPosition; // in base frame       
        }
    }

    void FootholdPlanner::ComputeMITFootHold(int legId)
    {
        // MIT foot placement
        auto& seResult = robot->stateDataFlow;
        Vec3<float> desiredSpeed = desiredStateCommand->stateDes.segment(6, 3);
        float desiredTwistingSpeed = desiredStateCommand->stateDes(11, 0);
        Vec3<float> desiredHeight(0.f,0.f, userParameters->desiredHeight - userParameters->footClearance);
        
        Eigen::Matrix<float,3,4> footPositionsInBaseFrame = robot->GetFootPositionsInBaseFrame();
        Eigen::Matrix<float,3,4> footPositionsInWorldFrame = robot->GetFootPositionsInWorldFrame();
        
        Mat3<float> robotBaseR = seResult.baseRMat;
        Mat3<float> controlFrameR = seResult.groundRMat;
        Mat3<float> dR = seResult.baseRInControlFrame;
        // dR.setIdentity();
        Eigen::Matrix<float, 12, 1> currentJointAngles = robot->GetMotorAngles();
        Vec3<float> comVelocity = robot->GetEstimatedVelocityInBaseFrame(); // in base frame
        float yawDot = robot->GetBaseRollPitchYawRate()[2]; // in base frame
        Vec3<float> rpy = robot->GetBaseRollPitchYaw();
        Eigen::Matrix<float,3,4> hipPositions = robot->GetHipPositionsInBaseFrame();
        Vec3<float> pRobotFrame = robot->hipOffset.col(legId); // in com frame
        float interleave_y[4] = {-0.08, 0.08, -0.08, 0.08};
        // float interleave_gain = -0.2;
        // float v_abs = std::fabs(robot->baseVelocityInBaseFrame[0]);
        // pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
        // Vec3<float> twistingVector = {-hipOffset[1], hipOffset[0], 0.f};
        float stance_time = gaitGenerator->stanceDuration[legId];
        float swing_time = gaitGenerator->swingDuration[legId];
        // std::cout << "stance_time = " << stance_time << std::endl;
        // std::cout << "pRobotFrame = " << pRobotFrame.transpose() << std::endl;
        // std::cout << "robotBaseR = " << robotBaseR << std::endl;
        // std::cout << "robotP = " << robot->basePosition.transpose() << std::endl;
         
        Vec3<float> pYawCorrected = 
          robotics::math::coordinateRotation(CoordinateAxis::Z, -desiredTwistingSpeed*stance_time/2) * pRobotFrame; // in base frame, // 机身旋转yaw后，得到在机身坐标系下的hip坐标
        // std::cout << "pYawCorrected " << pYawCorrected.transpose() << std::endl; 
          //世界坐标系下hip坐标 以剩余摆动时间内匀速运动来估计
          Mat3<float> rollR = robotics::math::coordinateRotation(CoordinateAxis::X, rpy[0]); // abad->hip offset vector
            Vec3<float> Pf = robotBaseR * (pYawCorrected + desiredSpeed * 0/*gaitGenerator->swingTimeRemaining[legId]*/ + rollR * Vec3<float>{0,interleave_y[legId],0})
                                // +robot->basePosition
                                ; 
            // std::cout << "Pf " << Pf.transpose() << std::endl; 
        
            float p_rel_max = 0.2f;
            Vec3<float> v_des_world = robotBaseR * desiredSpeed;
            // std::cout << "VdesWorld = " << v_des_world  <<std::endl;
            // std::cout << "Vworld = " << seResult.baseVInWorldFrame.transpose()  <<std::endl;
            float pfx_rel = seResult.baseVInWorldFrame[0] * stance_time/2 +
                .03f * (seResult.baseVInWorldFrame[0] - v_des_world[0]);// + (0.5f * sqrt(robot->basePosition[2] / 9.81f)) * (seResult.baseVInWorldFrame[1] * desiredTwistingSpeed);

            float pfy_rel = seResult.baseVInWorldFrame[1] * swing_time/2 +
                .03f * (seResult.baseVInWorldFrame[1] - v_des_world[1]);// + (0.5f * sqrt(robot->basePosition[2] / 9.81f)) * (-seResult.baseVInWorldFrame[0] * desiredTwistingSpeed);
            
            pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
            pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
            Pf[0] += pfx_rel;
            Pf[1] += pfy_rel;
            // float groundP =  groundEsitmator->GetControlFrameRPY()[1];
            // if (fabs(groundP)< 0.1 ) {
            //     groundP = 0;
            // }
            Pf[2] = -desiredHeight[2];//*(1-sin(-groundP));
        // }
        // footSwingTrajectories[i].setFinalPosition(Pf);
        // 最终得到足底的位置，并作为轨迹终点 世界坐标系下的落足点
        desiredFootholds.col(legId) = robotBaseR.transpose()* Pf; // in base frame
        phase[legId] = gaitGenerator->normalizedPhase(legId);
    }

} // namespace Quadruped

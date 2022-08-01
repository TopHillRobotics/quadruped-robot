/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Swing Leg Controller
* Author: Xie Ming Cheng & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: add head comment and add some function comments and delete some test functions. @ xie_mingcheng 2021.11.22;
*       add position mode control API for foothold. @ Zhu Yijie 2021.11.24;
*/

#include "mpc_controller/qr_raibert_swing_leg_controller.h"

using namespace Eigen;
using namespace std;
namespace Quadruped {
// The position correction coefficients in Raibert's formula.
    const Matrix<float, 3, 1> swingKp(0.03, 0.03, 0.03);

    float qrSwingLegController::GenParabola(float phase, float start, float mid, float end)
    {
        float mid_phase = 0.5;
        float deltaOne, deltaTwo, deltaThree, coefa, coefb, coefc;
        deltaOne = mid - start;
        deltaTwo = end - start;
        deltaThree = pow(mid_phase, 2) - mid_phase;
        coefa = (deltaOne - deltaTwo * mid_phase) / deltaThree;
        coefb = (deltaTwo * pow(mid_phase, 2) - deltaOne) / deltaThree;
        coefc = start;
        return coefa * pow(phase, 2) + coefb * phase + coefc;
    }

    Matrix<float, 3, 1> qrSwingLegController::GenSwingFootTrajectory(float inputPhase,
                                                                          Matrix<float, 3, 1> startPos,
                                                                          Matrix<float, 3, 1> endPos)
    {
        float phase = inputPhase;
        if (inputPhase <= 0.5) {
            phase = 0.8 * sin(inputPhase * M_PI);
        } else {
            phase = 0.8 + (inputPhase - 0.5) * 0.4;
        }
        float x, y, maxClearance, mid, z;
        x = (1 - phase) * startPos(0, 0) + phase * endPos(0, 0);
        y = (1 - phase) * startPos(1, 0) + phase * endPos(1, 0);
        maxClearance = 0.1;
        mid = max(endPos(2, 0), startPos(2, 0)) + maxClearance;
        z = GenParabola(phase, startPos(2, 0), mid, endPos(2, 0));
        return Matrix<float, 3, 1>(x, y, z);
    }

    qrSwingLegController::qrSwingLegController(qrRobot *robot,
                                                         qrGaitGenerator *gaitGenerator,
                                                         qrRobotEstimator *stateEstimator,
                                                         qrGroundSurfaceEstimator *groundEstimator,
                                                         qrFootholdPlanner *footholdPlanner,
                                                         Matrix<float, 3, 1> desiredSpeed,
                                                         float desiredTwistingSpeed,
                                                         float desiredHeight,
                                                         float footClearance,
                                                         std::string configPath)
    {
        this->robot = robot;
        this->gaitGenerator = gaitGenerator;
        this->stateEstimator = stateEstimator;
        this->groundEstimator = groundEstimator;
        this->footholdPlanner = footholdPlanner;
        this->desiredSpeed = desiredSpeed;
        this->desiredTwistingSpeed = desiredTwistingSpeed;
        this->desiredHeight = Matrix<float, 3, 1>(0, 0, desiredHeight - footClearance);
        swingLegConfig = YAML::LoadFile(configPath);
        footInitPose = swingLegConfig["swing_leg_params"]["foot_in_world"].as<std::vector<std::vector<float>>>();
        footOffset = swingLegConfig["swing_leg_params"]["foot_offset"].as<float>();        
    }

    void qrSwingLegController::Reset(float currentTime)
    { 
        phaseSwitchFootLocalPos = robot->state.GetFootPositionsInBaseFrame();
        phaseSwitchFootGlobalPos = robot->state.GetFootPositionsInWorldFrame();
        Matrix<float, 1, 4> footX = MatrixXf::Map(&footInitPose[0][0], 1, 4);
        Matrix<float, 1, 4> footY = MatrixXf::Map(&footInitPose[1][0], 1, 4);
        Matrix<float, 1, 4> footZ = MatrixXf::Map(&footInitPose[2][0], 1, 4);       
        
        footHoldInWorldFrame.row(0) << 0.185f, 0.185f, -0.175f, -0.175f;
        footHoldInWorldFrame.row(1) << -0.145f, 0.145f, -0.145f, 0.145f;
        footHoldInWorldFrame.row(2) << 0.f, 0.f, 0.f, 0.f;
        footHoldInWorldFrame(0, 0) -= 0.05;
        footHoldInWorldFrame(0, 3) -= 0.05;
        swingJointAnglesVelocities.clear();
    }

    void qrSwingLegController::Update(float currentTime)
    {
        // swingJointAnglesVelocities.clear();
        const Vec4<int>& newLegState = gaitGenerator->desiredLegState;
        const Vec4<int>& curLegState = gaitGenerator->curLegState;
        // the footHoldOffset is first init at robot.h, then update it at groundEstimator.cpp 
        Eigen::Matrix<float, 3, 1> constOffset = {robot->config->footHoldOffset, 0.f, 0.f};
        
        // Detects phase switch for each leg so we can remember the feet position at
        // the beginning of the swing phase.
        for (int legId = 0; legId < NumLeg; ++legId) {
            if (newLegState(legId) == LegState::SWING && newLegState(legId) != gaitGenerator->lastLegState(legId)) {
                phaseSwitchFootLocalPos.col(legId) = robot->state.GetFootPositionsInBaseFrame().col(legId);
            }
        }
       
    }

    map<int, Matrix<float, 5, 1>> qrSwingLegController::GetAction()
    {
        Matrix<float, 3, 1> comVelocity;
        Matrix<float, 3, 1> hipOffset;
        Matrix<float, 3, 1> twistingVector;
        Matrix<float, 3, 1> hipHorizontalVelocity;
        Matrix<float, 3, 1> targetHipHorizontalVelocity;
        Matrix<float, 3, 1> footTargetPosition;
        Matrix<float, 3, 1> footPositionInBaseFrame, footVelocityInBaseFrame, footAccInBaseFrame;
        Matrix<float, 3, 1> footPositionInWorldFrame, footVelocityInWorldFrame, footAccInWorldFrame;
        Matrix<float, 3, 1> footPositionInControlFrame, footVelocityInControlFrame, footAccInControlFrame;
        Matrix<int, 3, 1> jointIdx;
        Matrix<float, 3, 1> jointAngles;
        Matrix<float, 3, 4> hipPositions;
        float yawDot;
        Matrix<float, 12, 1> currentJointAngles = robot->GetMotorAngles();
        comVelocity = stateEstimator->GetEstimatedVelocity(); // in base frame
        yawDot = robot->GetBaseRollPitchYawRate()(2, 0); // in base frame
        hipPositions = robot->config->defaultHipPosition;
        for (int legId = 0; legId < NumLeg; ++legId) { 
            int tempState = gaitGenerator->legState[legId];
            if (tempState == LegState::STANCE || tempState == LegState::EARLY_CONTACT) {
                continue;
            }
            // printf("[leg %d] swing, normal phase = %f \n", legId, gaitGenerator->normalizedPhase[legId]);
            footVelocityInBaseFrame = Vec3<float>::Zero();
            footVelocityInWorldFrame = Vec3<float>::Zero();
            footVelocityInControlFrame = Vec3<float>::Zero();
            Quat<float> robotComOrientation = robot->GetBaseOrientation();
            Mat3<float> robotBaseR = robotics::math::quaternionToRotationMatrix(robotComOrientation).transpose();
            Quat<float> controlFrameOrientation = groundEstimator->GetControlFrameOrientation();
            Mat3<float> dR; // represent base frame in control frame
            if (groundEstimator->terrain.terrainType < 2) {
                dR = Mat3<float>::Identity();
                robotBaseR = Mat3<float>::Identity();
            } else {
                dR = robotics::math::quaternionToRotationMatrix(controlFrameOrientation) * robotBaseR;
            }
            hipOffset = hipPositions.col(legId);
            twistingVector = Matrix<float, 3, 1>(-hipOffset[1], hipOffset[0], 0);
            hipHorizontalVelocity = comVelocity + yawDot * twistingVector; // in base frame
            hipHorizontalVelocity = dR * hipHorizontalVelocity; // in control frame
            hipHorizontalVelocity[2] = 0.f;
            targetHipHorizontalVelocity = desiredSpeed + desiredTwistingSpeed * twistingVector; // in control frame
                
            footTargetPosition = dR.transpose() * (hipHorizontalVelocity * gaitGenerator->stanceDuration[legId] / 2.0 -
                swingKp.cwiseProduct(targetHipHorizontalVelocity - hipHorizontalVelocity))
                    + Matrix<float, 3, 1>(hipOffset[0], hipOffset[1], 0)
                    - robotics::math::TransformVecByQuat(robotics::math::quatInverse(robot->state.baseOrientation), desiredHeight);
            footPositionInBaseFrame = GenSwingFootTrajectory(gaitGenerator->normalizedPhase[legId],
                                                                phaseSwitchFootLocalPos.col(legId),
                                                                footTargetPosition);
            // compute joint position & joint velocity
            robot->config->ComputeMotorAnglesFromFootLocalPosition(legId, footPositionInBaseFrame, jointIdx, jointAngles);
            Vec3<float> motorVelocity = robot->config->ComputeMotorVelocityFromFootLocalVelocity(
                legId, jointAngles, footVelocityInBaseFrame);
            // check nan value
            int invalidAngleNum = 0;
            for (int i = 0; i < numMotorOfOneLeg; ++i) {
                if(isnan(jointAngles[i])) {
                    invalidAngleNum++;
                    jointAngles[i] = currentJointAngles[numMotorOfOneLeg* legId + i];
                }
                swingJointAnglesVelocities[jointIdx[i]] = {jointAngles[i], motorVelocity[i], legId};
            }
        }
        map<int, Matrix<float, 5, 1>> actions;
        Matrix<float, 12, 1> kps, kds;
        kps = robot->config->motorKps;
        kds = robot->config->motorKds;
        for (auto it = swingJointAnglesVelocities.begin(); it != swingJointAnglesVelocities.end(); ++it) {
            const std::tuple<float, float, int> &posVelId = it->second;
            const int singleLegId = std::get<2>(posVelId);
            
            bool flag;
            flag = (gaitGenerator->desiredLegState[singleLegId] == LegState::SWING);
            if (flag) {
                actions[it->first] << std::get<0>(posVelId), kps[it->first], std::get<1>(posVelId), kds[it->first], 0.f;
            }
        } 
        return actions;
    }
} // namespace Quadruped

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

#include "controller/qr_swing_leg_controller.h"

qrSwingLegController::
    qrSwingLegController(qrRobot *robot,
                         qrOpenloopGaitGenerator *gaitGenerator,
                         qrRobotEstimator *stateEstimator,
                         qrGroundSurfaceEstimator *groundEstimator,
                         qrFootholdPlanner *footholdPlanner,
                         Eigen::Matrix<float, 3, 1> desiredSpeed,
                         float desiredTwistingSpeed,
                         float desiredHeight,
                         float footClearance,
                         std::string configPath):robot(robot),
                                                 gaitGenerator(gaitGenerator),
                                                 robotEstimator(stateEstimator),
                                                 groundEstimator(groundEstimator),
                                                 footholdPlanner(footholdPlanner),
                                                 desiredSpeed(desiredSpeed),
                                                 desiredTwistingSpeed(desiredTwistingSpeed),
                                                 configFilepath(configPath)
{
    this->desiredHeight = Matrix<float, 3, 1>(0, 0, desiredHeight - footClearance);
    YAML::Node swingLegConfig = YAML::LoadFile(configPath);
    this->footInitPose = swingLegConfig["swing_leg_params"]["foot_in_world"].as<std::vector<std::vector<float>>>();
}

qrSwingLegController::Reset()
{
    this->phaseSwitchFootLocalPos = robot->GetFootPositionsInBaseFrame();
    this->phaseSwitchFootGlobalPos = robot->GetFootPositionsInWorldFrame();
    Matrix<float, 1, 4> footX = MatrixXf::Map(&this->footInitPose[0][0], 1, 4);
    Matrix<float, 1, 4> footY = MatrixXf::Map(&this->footInitPose[1][0], 1, 4);
    Matrix<float, 1, 4> footZ = MatrixXf::Map(&this->footInitPose[2][0], 1, 4);      
    
    this->footHoldInWorldFrame.row(0) << 0.185f, 0.185f, -0.175f, -0.175f;
    this->footHoldInWorldFrame.row(1) << -0.145f, 0.145f, -0.145f, 0.145f;
    this->footHoldInWorldFrame.row(2) << 0.f, 0.f, 0.f, 0.f;
    this->footHoldInWorldFrame(0, 0) -= 0.05;
    this->footHoldInWorldFrame(0, 3) -= 0.05;
    
    switch (this->robot->controlParams["mode"]) {
        case LocomotionMode::POSITION_LOCOMOTION: {
            std::cout << "[SwingLegController Reset] phaseSwitchFootLocalPos: \n" << this->phaseSwitchFootLocalPos
                    << std::endl;
            std::cout << "[SwingLegController Reset] phaseSwitchFootGlobalPos: \n" << this->phaseSwitchFootGlobalPos
                    << std::endl;
            std::cout << "[SwingLegController Reset] footHoldInWorldFrame: \n" << this->footHoldInWorldFrame << std::endl;
        } break;
        case LocomotionMode::WALK_LOCOMOTION: {
            this->footHoldInWorldFrame = this->phaseSwitchFootGlobalPos; //todo reset by default foot pose setting
            std::cout << "[SwingLegController Reset] phaseSwitchFootLocalPos: \n" << this->phaseSwitchFootLocalPos
                    << std::endl;
            std::cout << "[SwingLegController Reset] phaseSwitchFootGlobalPos: \n" << this->phaseSwitchFootGlobalPos
                    << std::endl;
            std::cout << "[SwingLegController Reset] footHoldInWorldFrame: \n" << this->footHoldInWorldFrame << std::endl;
        } break;
        default: break;  
    }

    swingJointAnglesVelocities.clear();
}

qrSwingLegController::Update()
{
    const Vec4<int>& newLegState = this->gaitGenerator->desiredLegState;
    const Vec4<int>& curLegState = this->gaitGenerator->curLegState;
    // the footHoldOffset is first init at robot.h, then update it at groundEstimator.cpp 
    Eigen::Matrix<float, 3, 1> constOffset = {this->robot->footHoldOffset, 0.f, 0.f};
    
    // Detects phase switch for each leg so we can remember the feet position at
    // the beginning of the swing phase.
    switch (this->robot->controlParams["mode"]) {
        case LocomotionMode::POSITION_LOCOMOTION: {
            for (int legId = 0; legId < 4; ++legId) {
                if ((newLegState(legId) == LegState::SWING || newLegState(legId) == LegState::USERDEFINED_SWING)
                    && curLegState(legId) == LegState::STANCE 
                    && !this->robot->stop) {
        
                    this->phaseSwitchFootLocalPos.col(legId) = this->robot->GetFootPositionsInBaseFrame().col(legId);
                    this->phaseSwitchFootGlobalPos.col(legId) = this->robot->GetFootPositionsInWorldFrame().col(legId);
                    if (legId == 0) { //update four leg footholds
                        this->footholdPlanner->UpdateOnce(footHoldInWorldFrame); // based on the last foothold position
                    }
                    this->footHoldInWorldFrame.col(legId) += this->footholdPlanner->GetFootholdsOffset().col(legId);
                }
            }
        } break;
        case LocomotionMode::WALK_LOCOMOTION: {
            for (int legId = 0; legId < 4; ++legId) {
                if ((newLegState(legId) == SubLegState::TRUE_SWING || newLegState(legId) == LegState::USERDEFINED_SWING)
                    && curLegState(legId) == SubLegState::UNLOAD_FORCE 
                    && !this->robot->stop) {
                    this->phaseSwitchFootLocalPos.col(legId) = this->robot->GetFootPositionsInBaseFrame().col(legId);
                    this->phaseSwitchFootGlobalPos.col(legId) = this->robot->GetFootPositionsInWorldFrame().col(legId);
                    // case 1:
                    this->footholdPlanner->UpdateOnce(footHoldInWorldFrame, {legId});
                    this->footHoldInWorldFrame.col(legId) = this->footholdPlanner->GetFootholdInWorldFrame(legId);
                    // case 2:
                    Vec3<float> footSourcePosition;
                    Vec3<float> footTargetPosition;
                    if (this->robot->robotConfig["is_sim"]) {
                        // running in simulation
                        footSourcePosition = this->phaseSwitchFootGlobalPos.col(legId);
                        footTargetPosition = this->footHoldInWorldFrame.col(legId);
                        footTargetPosition[2] = footSourcePosition[2] + this->footholdPlanner->desiredFootholdsOffset(2, legId);                      
                    } else {
                        // swing in base frame
                        footSourcePosition = this->phaseSwitchFootLocalPos.col(legId);
                        footTargetPosition = footSourcePosition + constOffset;
                        if (legId<=1) {
                            footTargetPosition[0] = 0.30f;
                            
                        }
                        else {
                            footTargetPosition[0] = -0.17f;
                        }
                        footTargetPosition[1] = -0.145 * pow(-1, legId);
                        footTargetPosition[2] = -0.32f;
                    }                        
                    
                    SplineInfo splineInfo;
                    splineInfo.splineType = "BSpline";
                    this->swingFootTrajectories[legId] = SwingFootTrajectory(splineInfo, footSourcePosition, footTargetPosition, 1.f, 0.15);
                    cout << "[SwingLegController::Update leg " << legId << "  update footHoldInWorldFrame: \n"
                        << this->footHoldInWorldFrame.col(legId) << endl;
                }
            }
        } break;
        default : {
            for (int legId = 0; legId < 4; ++legId) {
                if (newLegState(legId) == LegState::SWING && newLegState(legId) != this->gaitGenerator->lastLegState(legId)) {
                    this->phaseSwitchFootLocalPos.col(legId) = this->robot->GetFootPositionsInBaseFrame().col(legId);
                }
            }
        } break;
    }
}

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

std::tuple<std::vector<MotorCommand>, Eigen::Matrix<float, 3, 4>> qrSwingLegController::GetAction()
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
    // The position correction coefficients in Raibert's formula.
    const Matrix<float, 3, 1> swingKp(0.03, 0.03, 0.03);
    float yawDot;
    Matrix<float, 12, 1> currentJointAngles = robot->GetMotorAngles();
    comVelocity = stateEstimator->GetEstimatedVelocity(); // in base frame
    yawDot = robot->GetBaseRollPitchYawRate()(2, 0); // in base frame
    hipPositions = robot->GetHipPositionsInBaseFrame();
    for (int legId = 0; legId < NumLeg; ++legId) { 
        switch (robot->controlParams["mode"]) {
            case LocomotionMode::WALK_LOCOMOTION: {
                int tempState = gaitGenerator->detectedLegState[legId];
                if (tempState == LegState::STANCE || tempState == LegState::EARLY_CONTACT
                    || gaitGenerator->desiredLegState[legId] != SubLegState::TRUE_SWING
                    || robot->stop
                ) {
                    continue;
                }
            } break;
            default: { // velocity or position mode
                int tempState = gaitGenerator->legState[legId];
                if (tempState == LegState::STANCE || tempState == LegState::EARLY_CONTACT) {
                    continue;
                }
            } break;
        }
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
        switch (robot->controlParams["mode"]) {
            case LocomotionMode::VELOCITY_LOCOMOTION: {
                hipOffset = hipPositions.col(legId);
                twistingVector = Matrix<float, 3, 1>(-hipOffset[1], hipOffset[0], 0);
                hipHorizontalVelocity = comVelocity + yawDot * twistingVector; // in base frame
                hipHorizontalVelocity = dR * hipHorizontalVelocity; // in control frame
                hipHorizontalVelocity[2] = 0.f;
                targetHipHorizontalVelocity = desiredSpeed + desiredTwistingSpeed * twistingVector; // in control frame
                    
                footTargetPosition = dR.transpose() * (hipHorizontalVelocity * gaitGenerator->stanceDuration[legId] / 2.0 -
                    swingKp.cwiseProduct(targetHipHorizontalVelocity - hipHorizontalVelocity))
                        + Matrix<float, 3, 1>(hipOffset[0], hipOffset[1], 0)
                        - robotics::math::TransformVecByQuat(robotics::math::quatInverse(robot->baseOrientation), desiredHeight);
                footPositionInBaseFrame = GenSwingFootTrajectory(gaitGenerator->normalizedPhase[legId],
                                                                 phaseSwitchFootLocalPos.col(legId),
                                                                 footTargetPosition);
            } break;
            case LocomotionMode::POSITION_LOCOMOTION: {
                footPositionInWorldFrame = GenSwingFootTrajectory(gaitGenerator->normalizedPhase[legId],
                                                                  phaseSwitchFootGlobalPos.col(legId),
                                                                  footHoldInWorldFrame.col(legId)); // interpolation in world frame
                footPositionInBaseFrame = robotics::math::RigidTransform(robot->basePosition,
                                                                         robot->baseOrientation,
                                                                         footPositionInWorldFrame); // transfer to base frame
            } break;
            case LocomotionMode::WALK_LOCOMOTION: {
                bool flag;
                if (robot->robotConfig["is_sim"]) {
                    flag = swingFootTrajectories[legId].GenerateTrajectoryPoint(footPositionInWorldFrame,
                                                                                footVelocityInWorldFrame,
                                                                                footAccInWorldFrame, 
                                                                                gaitGenerator->normalizedPhase[legId],
                                                                                false);
                    footPositionInBaseFrame = robotics::math::RigidTransform(robot->basePosition,
                                                                             robot->baseOrientation,
                                                                             footPositionInWorldFrame); // transfer to base frame
                    footVelocityInBaseFrame = robotics::math::RigidTransform({0.f, 0.f, 0.f},
                                                                             robot->baseOrientation,
                                                                             footVelocityInWorldFrame); // transfer to base frame                                                                                        
                } else {
                    flag = swingFootTrajectories[legId].GenerateTrajectoryPoint(footPositionInBaseFrame, //footPositionInControlFrame
                                                                                footVelocityInBaseFrame, //footVelocityInControlFrame,
                                                                                footAccInBaseFrame, //footAccInControlFrame
                                                                                gaitGenerator->normalizedPhase[legId],
                                                                                false);                        
                }

                if (!flag) {
                    throw std::logic_error("error flag!\n");
                }                 
            } break;
            default: {throw std::domain_error("controlParams[mode] is not correct!\n");}
        }
        // compute joint position & joint velocity
        robot->ComputeMotorAnglesFromFootLocalPosition(legId, footPositionInBaseFrame, jointIdx, jointAngles);
        Vec3<float> motorVelocity = robot->ComputeMotorVelocityFromFootLocalVelocity(
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
    count++;
    auto RLAngles = currentJointAngles.tail(3);
    map<int, Matrix<float, 5, 1>> actions;
    Matrix<float, 12, 1> kps, kds;
    kps = robot->GetMotorPositionGains();
    kds = robot->GetMotorVelocityGains();
    for (auto it = swingJointAnglesVelocities.begin(); it != swingJointAnglesVelocities.end(); ++it) {
        const std::tuple<float, float, int> &posVelId = it->second;
        const int singleLegId = std::get<2>(posVelId);
        
        bool flag;
        if (robot->controlParams["mode"] == LocomotionMode::WALK_LOCOMOTION) {
            flag = (gaitGenerator->desiredLegState[singleLegId] == SubLegState::TRUE_SWING
                    && gaitGenerator->detectedLegState[singleLegId] != LegState::EARLY_CONTACT
                    );
        } else {
            flag = (gaitGenerator->desiredLegState[singleLegId] == LegState::SWING); // pos mode
        }
        
        if (flag) {
            actions[it->first] << std::get<0>(posVelId), kps[it->first], std::get<1>(posVelId), kds[it->first], 0.f;
        }
    }
    return actions;
}
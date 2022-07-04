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
#include "common/qr_se3.h"

qrSwingLegController::qrSwingLegController(qrRobot *robot,
                                           qrGaitGenerator *gaitGenerator,
                                           qrRobotVelocityEstimator* robotEstimator,
                                           qrGroundSurfaceEstimator *groundEstimator,
                                           Vec3<float> desiredLinearVelocity,
                                           float desiredTwistingVelocity,
                                           float desiredHeight,
                                           float footClearance,
                                           std::string configPath)
    : gaitGenerator(gaitGenerator),
      groundEstimator(groundEstimator),
      robotEstimator(robotEstimator),
      desiredLinearVelocity(desiredLinearVelocity),
      desiredTwistingVelocity(desiredTwistingVelocity),
      configFilepath(configPath)
{
    this->robotState = robot->GetRobotState();
    this->robotConfig = robot->GetRobotConfig();
    this->desiredHeight = Vec3<float>(0, 0, desiredHeight - footClearance);
    YAML::Node swingLegConfig = YAML::LoadFile(configPath);
    this->footInitPose = swingLegConfig["swing_leg_params"]["foot_in_world"].as<std::vector<std::vector<float>>>();
}

void qrSwingLegController::Reset()
{
    // TODO: interface
    this->phaseSwitchFootLocalPos = this->robotState->GetFootPositionInBaseFrame();
    // this->phaseSwitchFootGlobalPos = this->robot->GetFootPositionsInWorldFrame();
//    Eigen::Matrix<float, 1, 4> footX = Eigen::MatrixXf::Map(&this->footInitPose[0][0], 1, 4);
//    Eigen::Matrix<float, 1, 4> footY = Eigen::MatrixXf::Map(&this->footInitPose[1][0], 1, 4);
//    Eigen::Matrix<float, 1, 4> footZ = Eigen::MatrixXf::Map(&this->footInitPose[2][0], 1, 4);
    
    this->footHoldInWorldFrame.row(0) << 0.185f, 0.185f, -0.175f, -0.175f;
    this->footHoldInWorldFrame.row(1) << -0.145f, 0.145f, -0.145f, 0.145f;
    this->footHoldInWorldFrame.row(2) << 0.f, 0.f, 0.f, 0.f;
    this->footHoldInWorldFrame(0, 0) -= 0.05;
    this->footHoldInWorldFrame(0, 3) -= 0.05;
    
//    switch (this->robotConfig->controlMode) {
//        case LocomotionMode::POSITION_LOCOMOTION: {
//            std::cout << "[SwingLegController Reset] phaseSwitchFootLocalPos: \n" << this->phaseSwitchFootLocalPos
//                    << std::endl;
//            std::cout << "[SwingLegController Reset] phaseSwitchFootGlobalPos: \n" << this->phaseSwitchFootGlobalPos
//                    << std::endl;
//            std::cout << "[SwingLegController Reset] footHoldInWorldFrame: \n" << this->footHoldInWorldFrame << std::endl;
//        }
//        break;

//        case LocomotionMode::WALK_LOCOMOTION: {
//            this->footHoldInWorldFrame = this->phaseSwitchFootGlobalPos; //todo reset by default foot pose setting
//            std::cout << "[SwingLegController Reset] phaseSwitchFootLocalPos: \n" << this->phaseSwitchFootLocalPos
//                    << std::endl;
//            std::cout << "[SwingLegController Reset] phaseSwitchFootGlobalPos: \n" << this->phaseSwitchFootGlobalPos
//                    << std::endl;
//            std::cout << "[SwingLegController Reset] footHoldInWorldFrame: \n" << this->footHoldInWorldFrame << std::endl;
//        }
//        break;

//        default: break;
//    }
}

void qrSwingLegController::Update()
{
    const Vec4<int>& newLegState = this->gaitGenerator->desiredLegState;
    const Vec4<int>& curLegState = this->gaitGenerator->curLegState;
    // the footHoldOffset is first init at robot.h, then update it at groundEstimator.cpp 
    Vec3<float> constOffset = {qrRobotConfig::footHoldOffset, 0.f, 0.f};
    
    // Detects phase switch for each leg so we can remember the feet position at
    // the beginning of the swing phase.
//    switch (this->robotConfig->controlMode) {
//        case LocomotionMode::POSITION_LOCOMOTION: {
//            for (int legId = 0; legId < 4; ++legId) {
//                if ((newLegState(legId) == LegState::SWING || newLegState(legId) == LegState::USERDEFINED_SWING)
//                    && curLegState(legId) == LegState::STANCE
//                    && !this->robot->stop) {
        
//                    this->phaseSwitchFootLocalPos.col(legId) = this->robot->GetFootPositionsInBaseFrame().col(legId);
//                    this->phaseSwitchFootGlobalPos.col(legId) = this->robot->GetFootPositionsInWorldFrame().col(legId);
//                    if (legId == 0) { //update four leg footholds
//                        this->footholdPlanner->UpdateOnce(footHoldInWorldFrame); // based on the last foothold position
//                    }
//                    this->footHoldInWorldFrame.col(legId) += this->footholdPlanner->GetFootholdsOffset().col(legId);
//                }
//            }
//        } break;
//        case LocomotionMode::WALK_LOCOMOTION: {
//            for (int legId = 0; legId < 4; ++legId) {
//                if ((newLegState(legId) == SubLegState::TRUE_SWING || newLegState(legId) == LegState::USERDEFINED_SWING)
//                    && curLegState(legId) == SubLegState::UNLOAD_FORCE
//                    && !this->robot->stop) {
//                    this->phaseSwitchFootLocalPos.col(legId) = this->robot->GetFootPositionsInBaseFrame().col(legId);
//                    this->phaseSwitchFootGlobalPos.col(legId) = this->robot->GetFootPositionsInWorldFrame().col(legId);
//                    // case 1:
//                    this->footholdPlanner->UpdateOnce(footHoldInWorldFrame, {legId});
//                    this->footHoldInWorldFrame.col(legId) = this->footholdPlanner->GetFootholdInWorldFrame(legId);
//                    // case 2:
//                    Vec3<float> footSourcePosition;
//                    Vec3<float> footTargetPosition;
//                    if (this->robot->robotConfig["is_sim"]) {
//                        // running in simulation
//                        footSourcePosition = this->phaseSwitchFootGlobalPos.col(legId);
//                        footTargetPosition = this->footHoldInWorldFrame.col(legId);
//                        footTargetPosition[2] = footSourcePosition[2] + this->footholdPlanner->desiredFootholdsOffset(2, legId);
//                    } else {
//                        // swing in base frame
//                        footSourcePosition = this->phaseSwitchFootLocalPos.col(legId);
//                        footTargetPosition = footSourcePosition + constOffset;
//                        // TODO :: question
//                        if (legId<=1) {
//                            footTargetPosition[0] = 0.30f;
//                        }
//                        else {
//                            footTargetPosition[0] = -0.17f;
//                        }
//                        footTargetPosition[1] = -0.145 * pow(-1, legId);
//                        footTargetPosition[2] = -0.32f;
//                    }
                    
//                    SplineInfo splineInfo;
//                    splineInfo.splineType = "BSpline";
//                    this->swingFootTrajectories[legId] = qrSwingFootTrajectory(splineInfo, footSourcePosition, footTargetPosition, 1.f, 0.15);
//                    cout << "[SwingLegController::Update leg " << legId << "  update footHoldInWorldFrame: \n"
//                        << this->footHoldInWorldFrame.col(legId) << endl;
//                }
//            }
//        } break;
//        default : {
            for (int legId = 0; legId < 4; ++legId) {
                if (newLegState(legId) == LegState::SWING && newLegState(legId) != this->gaitGenerator->lastLegState(legId)) {
                    this->phaseSwitchFootLocalPos.col(legId) = this->robotState->GetFootPositionInBaseFrame().col(legId);
                }
            }
//        } break;
//    }
}

float qrSwingLegController::GenerateParabola(float x, float y0, float ym, float y1)
{
    // float mid_phase = 0.5;
    // float deltaOne, deltaTwo, deltaThree, coefa, coefb, coefc;
    // deltaOne = mid - start;
    // deltaTwo = end - start;
    // deltaThree = pow(mid_phase, 2) - mid_phase;
    // coefa = (deltaOne - deltaTwo * mid_phase) / deltaThree;
    // coefb = (deltaTwo * pow(mid_phase, 2) - deltaOne) / deltaThree;
    // coefc = start;
    // return coefa * pow(phase, 2) + coefb * phase + coefc;
    
    float a;
    float b;
    float c;
   
    a = 2*(y0 - 2*ym + y1);
    b = -3*y0 + 4*ym - y1;
    c = y0;
    
    return a * x * x + b * x + c;
}

Vec3<float> qrSwingLegController::GenerateSwingFootTrajectory(float inputPhase,
                                                                      Vec3<float> startPos,
                                                                      Vec3<float> endPos,
                                                                      float clearance)
{
    // refer to google's motion_imitation code (Python)
    // For the first half of the swing cycle, the swing leg moves faster and finishes 
    // 80% of the full swing trajectory. The rest 20% of trajectory takes another half swing cycle. 
    // Intuitely, we want to move the swing foot quickly to the target landing location and 
    // stay above the ground. In this way the control is more robust to perturbations to the body
    // that may cause the swing foot to drop onto the ground earlier than expected.
    // This is a common practice similar to the MIT cheetah and Marc Raibert's original controllers.
    float phase;
    float x;
    float y;
    float z;
    float mid;

    phase = inputPhase;

    if (inputPhase <= 0.5f) {
        phase = 0.8f * sinf(inputPhase * M_PI);
    } else {
        phase = 0.8f + (inputPhase - 0.5f) * 0.4f;
    }
    
    clearance = 0.1f;
    
    x = (1 - phase) * startPos(0, 0) + phase * endPos(0, 0);
    y = (1 - phase) * startPos(1, 0) + phase * endPos(1, 0);
    mid = std::max(endPos(2, 0), startPos(2, 0)) + clearance;
    z = GenerateParabola(phase, startPos(2, 0), mid, endPos(2, 0));

    return Vec3<float>(x, y, z);
}


// std::map<int, Eigen::Matrix<float, 5, 1>> qrSwingLegController::GetAction()
std::map<int, qrMotorCmd> qrSwingLegController::GetAction()
{
    Vec3<float> comVelocity;
    Vec3<float> hipOffset;
    Vec3<float> twistingVector;
    Vec3<float> hipHorizontalVelocity;
    Vec3<float> targetHipHorizontalVelocity;
    Vec3<float> footTargetPosition;
    Vec3<float> footPositionInBaseFrame, footVelocityInBaseFrame, footAccInBaseFrame;
    Vec3<float> footPositionInWorldFrame, footVelocityInWorldFrame, footAccInWorldFrame;
    Vec3<float> footPositionInControlFrame, footVelocityInControlFrame, footAccInControlFrame;
    Eigen::Matrix<int, 3, 1> jointIdx;
    Vec3<float> jointAngles;
    Mat3x4<float> hipPositions;
    // The position correction coefficients in Raibert's formula.
    const Vec3<float> swingKp(0.03, 0.03, 0.03);
    float yawDot;
    Vec12<float> currentJointAngles = this->robotState->q;
    comVelocity = this->robotEstimator->GetEstimatedVelocity(); // in base frame
    yawDot = this->robotState->GetDrpy()(2, 0); // in base frame
    hipPositions = this->robotConfig->GetHipPositionsInBaseFrame();
    for (int legId = 0; legId < qrRobotConfig::numLegs; ++legId) {
        // switch (robotConfig->controlMode) {
        //     case LocomotionMode::WALK_LOCOMOTION: {
        //         int tempState = this->gaitGenerator->detectedLegState[legId];
        //         if (tempState == LegState::STANCE || tempState == LegState::EARLY_CONTACT
        //             || this->gaitGenerator->desiredLegState[legId] != SubLegState::TRUE_SWING
        //             || robot->stop
        //         ) {
        //             continue;
        //         }
        //     } break;
        //     default: { // velocity or position mode
                int tempState = this->gaitGenerator->legState[legId];
                if (tempState == LegState::STANCE || tempState == LegState::EARLY_CONTACT) {
                    continue;
                }
        //     } break;
        // }
        footVelocityInBaseFrame = Vec3<float>::Zero();
        footVelocityInWorldFrame = Vec3<float>::Zero();
        footVelocityInControlFrame = Vec3<float>::Zero();
        Quat<float> robotComOrientation = this->robotState->GetBaseOrientation();
        Mat3<float> robotBaseR = math::Quat2RotMat(robotComOrientation);
        Quat<float> controlFrameOrientation = this->groundEstimator->GetControlFrameOrientation();
        Mat3<float> dR; // represent base frame in control frame
        if (this->groundEstimator->GetTerrain().terrainType < 2) {
            dR = Mat3<float>::Identity();
            robotBaseR = Mat3<float>::Identity();
        } else {
            dR = math::Quat2RotMat(controlFrameOrientation).transpose() * robotBaseR;
        }
        // switch (this->robotConfig->controlMode) {
        //     case LocomotionMode::VELOCITY_LOCOMOTION: {
                hipOffset = hipPositions.col(legId);
                twistingVector = Vec3<float>(-hipOffset[1], hipOffset[0], 0);
                hipHorizontalVelocity = comVelocity + yawDot * twistingVector; // in base frame
                hipHorizontalVelocity = dR * hipHorizontalVelocity; // in control frame
                hipHorizontalVelocity[2] = 0.f;
                targetHipHorizontalVelocity = desiredLinearVelocity + desiredTwistingVelocity * twistingVector; // in control frame
                    
                footTargetPosition = dR.transpose() * (hipHorizontalVelocity * this->gaitGenerator->stanceDuration[legId] / 2.0 -
                    swingKp.cwiseProduct(targetHipHorizontalVelocity - hipHorizontalVelocity))
                        + Vec3<float>(hipOffset[0], hipOffset[1], 0)
                        - math::TransformVecByQuat(math::QuatInverse(this->robotState->GetBaseOrientation()), desiredHeight);
                footPositionInBaseFrame = GenerateSwingFootTrajectory(this->gaitGenerator->normalizedLegPhase[legId],
                                                                 this->phaseSwitchFootLocalPos.col(legId),
                                                                 footTargetPosition);
        //     } break;
        //     case LocomotionMode::POSITION_LOCOMOTION: {
        //         footPositionInWorldFrame = GenerateSwingFootTrajectory(this->gaitGenerator->normalizedPhase[legId],
        //                                                           this->phaseSwitchFootGlobalPos.col(legId),
        //                                                           this->footHoldInWorldFrame.col(legId)); // interpolation in world frame
        //         footPositionInBaseFrame = Math::RigidTransform(this->robotConfig->GetBasePosition(),
        //                                                        this->robotState->GetBaseOrientation(),
        //                                                        footPositionInWorldFrame); // transfer to base frame
        //     } break;
        //     case LocomotionMode::WALK_LOCOMOTION: {
        //         bool flag;
        //         if (robotConfig->isSim) {
        //             flag = this->swingFootTrajectories[legId].GenerateTrajectoryPoint(footPositionInWorldFrame,
        //                                                                               footVelocityInWorldFrame,
        //                                                                               footAccInWorldFrame, 
        //                                                                               this->gaitGenerator->normalizedPhase[legId],
        //                                                                               false);
        //             footPositionInBaseFrame = Math::RigidTransform(this->robotConfig->GetBasePosition(),
        //                                                            this->robotState->GetBaseOrientation(),
        //                                                            footPositionInWorldFrame); // transfer to base frame
        //             footVelocityInBaseFrame = Math::RigidTransform({0.f, 0.f, 0.f},
        //                                                            this->robotState->GetBaseOrientation(),
        //                                                            footVelocityInWorldFrame); // transfer to base frame                                                                                        
        //         } else {
        //             flag = this->swingFootTrajectories[legId].GenerateTrajectoryPoint(footPositionInBaseFrame, //footPositionInControlFrame
        //                                                                               footVelocityInBaseFrame, //footVelocityInControlFrame,
        //                                                                               footAccInBaseFrame, //footAccInControlFrame
        //                                                                               this->gaitGenerator->normalizedPhase[legId],
        //                                                                               false);                        
        //         }

        //         if (!flag) {
        //             throw std::logic_error("error flag!\n");
        //         }                 
        //     } break;
        //     default: {throw std::domain_error("controlParams[mode] is not correct!\n");}
        // }
        // compute joint position & joint velocity
        jointAngles = this->robotConfig->FootPosition2JointAngles(footPositionInBaseFrame, legId);
        Vec3<float> motorVelocity = this->robotConfig->FootVelocity2JointVelocity(
            jointAngles, footVelocityInBaseFrame, legId);
        // check nan value
        int invalidAngleNum = 0;
        for (int i = 0; i < qrRobotConfig::dofPerLeg; ++i) {
            if(isnan(jointAngles[i])) {
                invalidAngleNum++;
                jointAngles[i] = currentJointAngles[qrRobotConfig::dofPerLeg * legId + i];
            }
            this->swingJointAnglesVelocities[jointIdx[i]] = {jointAngles[i], motorVelocity[i], legId};
        }
    }
    auto RLAngles = currentJointAngles.tail(3);
    // map<int, Eigen::Matrix<float, 5, 1>> actions;
    std::map<int, qrMotorCmd> actions;
    Vec12<float> kps, kds;
    kps = robotConfig->GetKps();
    kds = robotConfig->GetKds();
    for (auto it = this->swingJointAnglesVelocities.begin(); it != this->swingJointAnglesVelocities.end(); ++it) {
        const std::tuple<float, float, int> &posVelId = it->second;
        const int singleLegId = std::get<2>(posVelId);
        
        bool flag = (this->gaitGenerator->desiredLegState[singleLegId] == LegState::SWING); // pos mode

        if (flag) {
            qrMotorCmd temp(std::get<0>(posVelId), kps[it->first], std::get<1>(posVelId), kds[it->first], 0.f);
            // temp.setCmd(std::get<0>(posVelId), kps[it->first], std::get<1>(posVelId), kds[it->first], 0.f);
            // actions[it->first] << std::get<0>(posVelId), kps[it->first], std::get<1>(posVelId), kds[it->first], 0.f;
            actions[it->first] = temp;
        }
    }
    return actions;
}

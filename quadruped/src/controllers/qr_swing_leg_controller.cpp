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

#include "controllers/qr_swing_leg_controller.h"

using namespace Eigen;
using namespace std;


namespace Quadruped {

qrRaibertSwingLegController::qrRaibertSwingLegController(
    qrRobot *robot,
    qrGaitGenerator *gait_generator,
    qrStateEstimatorContainer* state_estimators,
    qrFootholdPlanner *foothold_planner,
    qrUserParameters& user_parameters,
    std::string config_path)
{
    this->robot = robot;
    this->gaitGenerator = gait_generator;
    this->stateEstimator = state_estimators->GetRobotEstimator();
    this->groundEstimator = state_estimators->GetGroundEstimator();
    this->footholdPlanner = foothold_planner;
    this->desiredSpeed << user_parameters.desiredSpeed[0], user_parameters.desiredSpeed[1], user_parameters.desiredSpeed[2];
    this->desiredTwistingSpeed = user_parameters.desiredTwistingSpeed;
    this->desiredHeight = Matrix<float, 3, 1>(0, 0, user_parameters.desiredHeight - user_parameters.footClearance);

    std::cout << config_path <<std::endl;

    swingLegConfig = YAML::LoadFile(config_path);
    footOffset = swingLegConfig["swing_leg_params"]["foot_offset"].as<float>();
    this->userParameters = &user_parameters;

    // this->Reset(0);
}


void qrRaibertSwingLegController::Reset(float current_time)
{
    phaseSwitchFootLocalPos = robot->GetFootPositionsInBaseFrame();
    desiredFootPositionsInBaseFrame = phaseSwitchFootLocalPos;

    foot_pos_rel_last_time = phaseSwitchFootLocalPos;
    foot_pos_target_last_time = foot_pos_rel_last_time;

    phaseSwitchFootGlobalPos = robot->GetFootPositionsInWorldFrame();
    desiredStateCommand->footTargetPositionsInWorldFrame = phaseSwitchFootGlobalPos;
    //footHoldInControlFrame = phaseSwitchFootLocalPos;

    footholdPlanner->Reset(current_time); // reset planner
    swingKp = MatrixXf::Map(&userParameters->swingKp["trot"][0], 3, 1);

    std::cout << "[SwingLegController Reset] phaseSwitchFootLocalPos: \n" << phaseSwitchFootLocalPos << std::endl;
    std::cout << "[SwingLegController Reset] phaseSwitchFootGlobalPos: \n" << phaseSwitchFootGlobalPos << std::endl;
    std::cout << "[SwingLegController Reset] footHoldInWorldFrame: \n" << footHoldInWorldFrame << std::endl;
    switch (robot->controlParams["mode"]) {
        case LocomotionMode::POSITION_LOCOMOTION: {
            footHoldInWorldFrame = phaseSwitchFootGlobalPos;
            footHoldInWorldFrame(0, 0) -= 0.05;
            footHoldInWorldFrame(0, 3) -= 0.05;

        } break;
        case LocomotionMode::WALK_LOCOMOTION: {
            footHoldInWorldFrame = phaseSwitchFootGlobalPos; //todo reset by default foot pose setting
        } break;
        case LocomotionMode::ADVANCED_TROT: {
            swingKp = MatrixXf::Map(&userParameters->swingKp["advanced_trot"][0], 3, 1);
        } break;
        default: break;
    }
    if (robot->controlParams["mode"]!=LocomotionMode::WALK_LOCOMOTION) {
        splineInfo.splineType = SplineType::XYLinear_ZParabola;
        for (int i = 0; i < NumLeg; ++i) {
            swingFootTrajectories[i] = SwingFootTrajectory(splineInfo, phaseSwitchFootLocalPos.col(i),
                phaseSwitchFootLocalPos.col(i), 1.f, 0.15);
        }
    }
    swingJointAnglesVelocities.clear();
}


void qrRaibertSwingLegController::Update(float current_time)
{
    // swingJointAnglesVelocities.clear();
    const Vec4<int>& newLegState = gaitGenerator->desiredLegState;
    const Vec4<int>& curLegState = gaitGenerator->curLegState;
    // the footHoldOffset is first init at robot.h, then update it at groundEstimator.cpp
    Vec3<float> constOffset = {robot->footHoldOffset, 0.f, 0.f};
    Quat<float> robotComOrientation = robot->GetBaseOrientation();
    Vec3<float> robotComRPY = robot->GetBaseRollPitchYaw();
    Mat3<float> Rb = robot->stateDataFlow.baseRMat;
    Vec3<float> groundRPY = groundEstimator->GetControlFrameRPY();
    Mat3<float> RcSource = robot->stateDataFlow.groundRMat;
    Mat3<float> Rcb = robot->stateDataFlow.baseRInControlFrame;
    swingFootIds.clear();

    /* Detects phase switch for each leg so we can remember the feet position at
     * the beginning of the swing phase.
     */
    switch (robot->controlParams["mode"]) {

    case LocomotionMode::POSITION_LOCOMOTION: {
        for (int legId = 0; legId < NumLeg; ++legId) {
            if ((newLegState(legId) == LegState::SWING || newLegState(legId) == LegState::USERDEFINED_SWING)
                && curLegState(legId) == LegState::STANCE
                && !robot->stop) {
                phaseSwitchFootLocalPos.col(legId) = robot->GetFootPositionsInBaseFrame().col(legId);
                phaseSwitchFootGlobalPos.col(legId) = robot->GetFootPositionsInWorldFrame().col(legId);
                if (legId == 0) { // update four leg footholds
                    footholdPlanner->UpdateOnce(footHoldInWorldFrame); // based on the last foothold position
                }
                footHoldInWorldFrame.col(legId) += footholdPlanner->GetFootholdsOffset().col(legId);
            }
        }
    } break;

    case LocomotionMode::WALK_LOCOMOTION: {
        for (int legId = 0; legId < NumLeg; ++legId) {
            if ((newLegState(legId) == SubLegState::TRUE_SWING || newLegState(legId) == LegState::USERDEFINED_SWING)
                && curLegState(legId) == SubLegState::UNLOAD_FORCE
                && !robot->stop) {

                phaseSwitchFootLocalPos.col(legId) = robot->GetFootPositionsInBaseFrame().col(legId);
                phaseSwitchFootGlobalPos.col(legId) = robot->GetFootPositionsInWorldFrame().col(legId);

                footholdPlanner->UpdateOnce(footHoldInWorldFrame, {legId});
                footHoldInWorldFrame.col(legId) = footholdPlanner->GetFootholdInWorldFrame(legId);

                Vec3<float> footSourcePosition;
                Vec3<float> footTargetPosition;
                if (robot->isSim) {
                    footSourcePosition = phaseSwitchFootGlobalPos.col(legId);
                    footTargetPosition = footHoldInWorldFrame.col(legId);
                    footTargetPosition[2] = footSourcePosition[2] + footholdPlanner->desiredFootholdsOffset(2, legId);
                } else {
                    // swing in base frame
                    footSourcePosition = phaseSwitchFootLocalPos.col(legId);
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

                // SplineInfo splineInfo;
                splineInfo.splineType = SplineType::BSpline;
                swingFootTrajectories[legId] = SwingFootTrajectory(splineInfo, footSourcePosition, footTargetPosition, 1.f, 0.15);
                cout << "[SwingLegController::Update leg " << legId << "  update footHoldInWorldFrame: \n"
                    << footHoldInWorldFrame.col(legId) << endl;
                // cout << "[SwingLegController::Update leg " << legId << "  update footHoldInControlFrame: \n"
                //     << footHoldInControlFrame.col(legId) << endl;
            }
        }
    } break;

    default : {
        for (int legId = 0; legId < NumLeg; ++legId) {
            if (newLegState(legId) == LegState::SWING && newLegState(legId) != gaitGenerator->curLegState(legId)) {
                phaseSwitchFootLocalPos.col(legId) = robot->GetFootPositionsInBaseFrame().col(legId);
                phaseSwitchFootGlobalPos.col(legId) = Rb * phaseSwitchFootLocalPos.col(legId); // robot->GetFootPositionsInWorldFrame().col(legId);
                if (robot->controlParams["mode"]==LocomotionMode::ADVANCED_TROT) {
                    splineInfo.splineType = SplineType::XYLinear_ZParabola; // BSpline, XYLinear_ZParabola
                    // lastLegTorque.col(legId) << 0.f,0.f,0.f;
                    footholdPlanner->firstSwingBaseState << robot->GetBasePosition(), robot->stateDataFlow.baseVInWorldFrame,
                            robot->baseRollPitchYaw, robot->baseRollPitchYawRate;
                } else {
                    splineInfo.splineType = SplineType::XYLinear_ZParabola; // BSpline, XYLinear_ZParabola
                }
                swingFootTrajectories[legId] = SwingFootTrajectory(splineInfo, phaseSwitchFootLocalPos.col(legId), phaseSwitchFootLocalPos.col(legId), 1.f, 0.15);
            }
        }
    } break;

    }

    switch (robot->controlParams["mode"]) {

    case LocomotionMode::WALK_LOCOMOTION: {
        for (u8 legId(0); legId < NumLeg; ++legId) {
            int tempState = gaitGenerator->detectedLegState[legId];
            if (tempState == LegState::STANCE || tempState == LegState::EARLY_CONTACT
                || gaitGenerator->desiredLegState[legId] != SubLegState::TRUE_SWING
                || robot->stop
            ) {
                continue;
            } else {
                swingFootIds.push_back(legId);
            }
        }
    } break;

    default: { // velocity / position / advanced_trot
        for (u8 legId(0); legId < NumLeg; ++legId) {
            int tempState = gaitGenerator->legState[legId];

            if ((tempState == LegState::STANCE && gaitGenerator->allowSwitchLegState[legId])
                    || tempState == LegState::EARLY_CONTACT) {
                continue;
            } else {
                swingFootIds.push_back(legId);
            }
        }
    } break;

    }

    if (robot->controlParams["mode"] == LocomotionMode::ADVANCED_TROT) {
        // footholdPlanner->ComputeHeuristicFootHold(legId);
        footholdPlanner->ComputeHeuristicFootHold(swingFootIds);
        // footholdPlanner->ComputeMITFootHold(legId);
    }

}

map<int, Matrix<float, 5, 1>> qrRaibertSwingLegController::GetAction()
{
    auto& stateData = robot->stateDataFlow;
    Matrix<float, 3, 1> baseVelocity;
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
    Matrix<float, 12, 1> currentJointAngles = robot->GetMotorAngles();
    float yawDot = robot->GetBaseRollPitchYawRate()(2, 0); // in base frame;
    baseVelocity = stateEstimator->GetEstimatedVelocity(); // in base frame
    hipPositions = robot->GetDefaultHipPosition();
    Eigen::Matrix<float,3,4> footVTarget;
    Eigen::Matrix<float,3,4> footVCurrent;
    Eigen::Matrix<float, 3, 4> footPositionsInBaseFrame = robot->GetFootPositionsInBaseFrame();
    Eigen::Matrix<float, 3, 4> footVelocitysInBaseFrame = robot->stateDataFlow.footVelocitiesInBaseFrame;

    // Visualization2D& vis = robot->stateDataFlow.visualizer;
    Quat<float> robotComOrientation = robot->GetBaseOrientation();
    Mat3<float> robotBaseR = robot->stateDataFlow.baseRMat;
    Quat<float> controlFrameOrientation = groundEstimator->GetControlFrameOrientation();
    Mat3<float> dR; // represent base frame in control frame
    float phase;
    if (groundEstimator->terrain.terrainType < 2) { // in horizontal terrain
        dR.setIdentity();
        robotBaseR.setIdentity();
    } else {
        dR = robot->stateDataFlow.baseRInControlFrame;
    }
    for (u8& legId : swingFootIds) {
        // printf("[leg %d] swing, normal phase = %f \n", legId, gaitGenerator->normalizedPhase[legId]);
        footVelocityInBaseFrame.setZero();
        footVelocityInWorldFrame.setZero();
        footVelocityInControlFrame.setZero();
        footAccInBaseFrame.setZero();
        footAccInWorldFrame.setZero();
        footAccInControlFrame.setZero();
        switch (robot->controlParams["mode"]) {

        case LocomotionMode::VELOCITY_LOCOMOTION: {
            hipOffset = hipPositions.col(legId).colwise() + robot->comOffset;
            twistingVector = Matrix<float, 3, 1>(-hipOffset[1], hipOffset[0], 0);
            hipHorizontalVelocity = baseVelocity + yawDot * twistingVector; // in base frame
            hipHorizontalVelocity = dR * hipHorizontalVelocity; // in control frame
            hipHorizontalVelocity[2] = 0.f;
            // targetHipHorizontalVelocity = desiredSpeed + desiredTwistingSpeed * twistingVector; // in control frame
            targetHipHorizontalVelocity = desiredStateCommand->stateDes.segment(6,3) + desiredStateCommand->stateDes(11) * twistingVector; // in control frame

            footTargetPosition = dR.transpose() * (hipHorizontalVelocity * gaitGenerator->stanceDuration[legId] / 2.0 -
                swingKp.cwiseProduct(targetHipHorizontalVelocity - hipHorizontalVelocity))
                + Matrix<float, 3, 1>(hipOffset[0], hipOffset[1], 0)
                - robotBaseR.transpose() * desiredHeight;


            swingFootTrajectories[legId].ResetFootTrajectory(1.f, phaseSwitchFootLocalPos.col(legId), footTargetPosition, 0.1);
            bool flag = swingFootTrajectories[legId].GenerateTrajectoryPoint(footPositionInBaseFrame,
                                                                            footVelocityInBaseFrame,
                                                                            footAccInBaseFrame,
                                                                            gaitGenerator->normalizedPhase(legId),
                                                                            true);

        } break;

        case LocomotionMode::POSITION_LOCOMOTION: {
            swingFootTrajectories[legId].ResetFootTrajectory(1.f, phaseSwitchFootGlobalPos.col(legId), footHoldInWorldFrame.col(legId), 0.1);
            bool flag = swingFootTrajectories[legId].GenerateTrajectoryPoint(footPositionInWorldFrame,
                                                                            footVelocityInWorldFrame,
                                                                            footAccInWorldFrame,
                                                                            gaitGenerator->normalizedPhase(legId),
                                                                            true);

            // footPositionInWorldFrame = GenSwingFootTrajectory(gaitGenerator->normalizedPhase[legId],
            //                                                   phaseSwitchFootGlobalPos.col(legId),
            //                                                   footHoldInWorldFrame.col(legId)); // interpolation in world frame
            footPositionInBaseFrame = robotics::math::RigidTransform(robot->basePosition,
                                                                    robot->baseOrientation,
                                                                    footPositionInWorldFrame); // transfer to base frame

        } break;

        case LocomotionMode::WALK_LOCOMOTION: {

            /* Control in world frame, get the target foot position at this phase. */
            bool flag;
            if (robot->isSim) {
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
            // footPositionInWorldFrame = robotics::math::invertRigidTransform(controlFrameOriginSource,
            //                                                             controlFrameOrientationSource,
            //                                                             footPositionInControlFrame); // transfer to world frame

        } break;

        case LocomotionMode::ADVANCED_TROT: {

            footTargetPosition = footholdPlanner->desiredFootholds.col(legId);
            phase = footholdPlanner->phase[legId];

            float H = 0.1;


            Vec3<float> footTargetPositionInWorldFrame = robotics::math::invertRigidTransform(robot->basePosition, robotComOrientation, footTargetPosition);
            desiredStateCommand->footTargetPositionsInWorldFrame.col(legId) = footTargetPositionInWorldFrame;

            swingFootTrajectories[legId].ResetFootTrajectory(1.f, phaseSwitchFootGlobalPos.col(legId), robotBaseR*footTargetPosition, H);
            bool flag = swingFootTrajectories[legId].GenerateTrajectoryPoint(footPositionInWorldFrame,
                                                                            footVelocityInWorldFrame,
                                                                            footAccInWorldFrame,
                                                                            phase,
                                                                            false);

            footPositionInBaseFrame = robotBaseR.transpose()* footPositionInWorldFrame;
            desiredFootPositionsInBaseFrame.col(legId) = footPositionInBaseFrame;
            if (phase < 1.0) {
                footVelocityInBaseFrame = robotBaseR.transpose()*footVelocityInWorldFrame;
                footVelocityInBaseFrame /= gaitGenerator->swingDuration[legId];
            }
            // footVelocityInBaseFrame.setZero();
            // footPositionInBaseFrame = footTargetPosition;

            Vec3<float> foot_vel_cur = footVelocitysInBaseFrame.col(legId);//(footPositionsInBaseFrame.col(legId) - foot_pos_rel_last_time.col(legId)) / 0.001;
            foot_pos_rel_last_time.col(legId) = footPositionsInBaseFrame.col(legId);

            Vec3<float> foot_vel_target = footVelocityInBaseFrame;//(footPositionInBaseFrame - foot_pos_target_last_time.col(legId)) / 0.002;
            foot_pos_target_last_time.col(legId) = footPositionInBaseFrame;

            footVCurrent.col(legId) = foot_vel_cur;

            stateData.wbcData.pFoot_des[legId] = robotics::math::invertRigidTransform(robot->basePosition, robotComOrientation, footPositionInBaseFrame); // in world Frame
            stateData.wbcData.vFoot_des[legId] = stateData.baseVInWorldFrame + robotBaseR * footVelocityInBaseFrame;
            stateData.wbcData.aFoot_des[legId] = robotBaseR * footAccInBaseFrame;

        } break;
        default: {
            throw std::domain_error("controlParams[mode] is not correct!\n");
        }

        }

        /* Compute joint position & joint velocity. */
        robot->ComputeMotorAnglesFromFootLocalPosition(legId, footPositionInBaseFrame, jointIdx, jointAngles);
        Vec3<float> motorVelocity = robot->ComputeMotorVelocityFromFootLocalVelocity(
            legId, jointAngles, footVelocityInBaseFrame);

        /* Check nan value. */
        int invalidAngleNum = 0;
        for (int i = 0; i < NumMotorOfOneLeg; ++i) {
            if(isnan(jointAngles[i])) {
                invalidAngleNum++;
                jointAngles[i] = currentJointAngles[NumMotorOfOneLeg* legId + i];
            }
            swingJointAnglesVelocities[jointIdx[i]] = {jointAngles[i], motorVelocity[i], legId};
        }
        if (invalidAngleNum > 0) {
            printf("[warning] swing leg controll receive nan value!\n");
        }
    }

    std::map<int, Matrix<float, 5, 1>> actions;
    Matrix<float, 12, 1> kps, kds;
    kps = robot->GetMotorKps();
    kds = robot->GetMotorKdp();

    for (auto it = swingJointAnglesVelocities.begin(); it != swingJointAnglesVelocities.end(); ++it) {
        const std::tuple<float, float, int> &posVelId = it->second;
        const int singleLegId = std::get<2>(posVelId);

        bool flag;
        switch (robot->controlParams["mode"])
        {
            case LocomotionMode::WALK_LOCOMOTION: {
                flag = (gaitGenerator->desiredLegState[singleLegId] == SubLegState::TRUE_SWING
                        && gaitGenerator->detectedLegState[singleLegId] != LegState::EARLY_CONTACT
                        );
            } break;
            case LocomotionMode::POSITION_LOCOMOTION: {
                flag = (gaitGenerator->legState[singleLegId] == LegState::SWING);
            } break;
            case LocomotionMode::VELOCITY_LOCOMOTION: {
                flag = (gaitGenerator->desiredLegState[singleLegId] == LegState::SWING);
            } break;
            default: {
                flag = (gaitGenerator->legState[singleLegId] == LegState::SWING
                        || gaitGenerator->legState[singleLegId] == LegState::USERDEFINED_SWING) // for trot up to slope,  // in velocity mode
                        || (gaitGenerator->curLegState[singleLegId] == LegState::SWING && !gaitGenerator->allowSwitchLegState[singleLegId]); // for gait schedule mode
            } break;
        }

        if (flag) {
            actions[it->first] << std::get<0>(posVelId), kps[it->first], std::get<1>(posVelId),kds[it->first], 0;
        }
    }

    return actions;
}

} // Namespace Quadruped

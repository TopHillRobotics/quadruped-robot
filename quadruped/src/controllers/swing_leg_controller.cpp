/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Swing Leg Controller
* Author: Xie Ming Cheng & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: add head comment and add some function comments and delete some test functions. @ xie_mingcheng 2021.11.22;
*       add position mode control API for foothold. @ Zhu Yijie 2021.11.24;
*/

#include "controllers/swing_leg_controller.h"

using namespace Eigen;
using namespace std;
namespace Quadruped {
    RaibertSwingLegController::RaibertSwingLegController(Robot *robot,
                                                         GaitGenerator *gaitGenerator,
                                                         StateEstimatorContainer<float>* stateEstimators,
                                                         FootholdPlanner *footholdPlanner,
                                                         UserParameters& userParametersIn,
                                                         std::string configPath)
    {
        this->robot = robot;
        this->gaitGenerator = gaitGenerator;
        this->stateEstimator = stateEstimators->GetRobotEstimator();
        this->groundEstimator = stateEstimators->GetGroundEstimator();
        this->footholdPlanner = footholdPlanner;
        this->desiredSpeed << userParametersIn.desiredSpeed[0], userParametersIn.desiredSpeed[1], userParametersIn.desiredSpeed[2];
        this->desiredTwistingSpeed = userParametersIn.desiredTwistingSpeed;
        this->desiredHeight = Matrix<float, 3, 1>(0, 0, userParametersIn.desiredHeight - userParametersIn.footClearance);
        std::cout << configPath <<std::endl;
        swingLegConfig = YAML::LoadFile(configPath);
        footInitPose = swingLegConfig["swing_leg_params"]["foot_in_world"].as<std::vector<std::vector<float>>>();
        footOffset = swingLegConfig["swing_leg_params"]["foot_offset"].as<float>();
        this->userParameters = &userParametersIn;
        
        // this->Reset(0);
    }

    void RaibertSwingLegController::Reset(float currentTime)
    { 
        phaseSwitchFootLocalPos = robot->GetFootPositionsInBaseFrame();
        desiredFootPositionsInBaseFrame = phaseSwitchFootLocalPos;

        foot_pos_rel_last_time = phaseSwitchFootLocalPos;
        foot_pos_target_last_time = foot_pos_rel_last_time;
        foot_vel_error.setZero();
        foot_pos_error.setZero();
        
        phaseSwitchFootGlobalPos = robot->GetFootPositionsInWorldFrame();
        desiredStateCommand->footTargetPositionsInWorldFrame = phaseSwitchFootGlobalPos;
        footHoldInControlFrame = phaseSwitchFootLocalPos;

        footholdPlanner->Reset(currentTime); // reset planner
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

    void RaibertSwingLegController::Update(float currentTime)
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
        // Detects phase switch for each leg so we can remember the feet position at
        // the beginning of the swing phase.
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
                        controlFrameOrientationSource = groundEstimator->GetControlFrameOrientation();
                        controlFrameOriginSource = robot->GetBasePosition();
                        phaseSwitchFootLocalPos.col(legId) = robot->GetFootPositionsInBaseFrame().col(legId);
                        phaseSwitchFootGlobalPos.col(legId) = robot->GetFootPositionsInWorldFrame().col(legId);
                        phaseSwitchFootControlPos.col(legId) = Rcb * phaseSwitchFootLocalPos.col(legId);
                        // case 1:
                        // footHoldInWorldFrame.col(legId) = phaseSwitchFootGlobalPos.col(legId); // todo, uncomment in simulation
                        footholdPlanner->UpdateOnce(footHoldInWorldFrame, {legId});
                        footHoldInWorldFrame.col(legId) = footholdPlanner->GetFootholdInWorldFrame(legId);
                        // case 2:
                        // footHoldInWorldFrame.col(legId) += constOffset;
                        
                        // footHoldInControlFrame.col(legId) = phaseSwitchFootControlPos.col(legId) + constOffset;
                        // footHoldInControlFrame(1,legId) = (footHoldInControlFrame(1,legId) - pow(-1, legId) * 0.15f)/2.0;
                        // float dPitch = groundRPY[1] - robotComRPY[1];
                        // printf("sin dPitch = %f\n", sin(dPitch)); 
                        // if (legId<=1) {
                        //     footHoldInControlFrame(0,legId) = min(0.30f+0.1f*abs(sin(dPitch)),footHoldInControlFrame(0,legId));
                        //     footHoldInControlFrame(0,legId) = max(0.10f,footHoldInControlFrame(0,legId));
                        // } else {
                        //     footHoldInControlFrame(0,legId) = max(-0.25f,footHoldInControlFrame(0,legId));
                        //     footHoldInControlFrame(0,legId) = min(-0.05f + 0.1f * abs(sin(dPitch)),footHoldInControlFrame(0,legId));
                        // }
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
                            splineInfo.splineType = SplineType::BSpline; // BSpline, XYLinear_ZParabola
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

    map<int, Matrix<float, 5, 1>> RaibertSwingLegController::GetAction()
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
        hipPositions = robot->GetHipPositionsInBaseFrame();
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
                    hipOffset = hipPositions.col(legId);
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
                    // footPositionInWorldFrame = GenSwingFootTrajectory(gaitGenerator->normalizedPhase[legId],
                    //                                                   phaseSwitchFootGlobalPos.col(legId),
                    //                                                   footHoldInWorldFrame.col(legId)); // interpolation in world frame
                    // footPositionInControlFrame = GenSwingFootTrajectory(gaitGenerator->normalizedPhase[legId],
                    //                                                   phaseSwitchFootControlPos.col(legId),
                    //                                                   footHoldInControlFrame.col(legId)); // interpolation in control frame
                    // std::cout << "[swing source (control)] " << phaseSwitchFootControlPos.col(legId) <<std::endl;
                    // std::cout << "[swing target (control)] " << footHoldInControlFrame.col(legId) << std::endl;
                    // std::cout << "[swing source (world)] " << phaseSwitchFootGlobalPos.col(legId) <<std::endl;
                    // std::cout << "[swing target (world)] " << footHoldInWorldFrame.col(legId) << std::endl;
                    
                    // if (legId==2) {
                    //     datax.push_back(footPositionInControlFrame[0]);
                    //     datay1.push_back(footPositionInControlFrame[2]);
                    // } 
                    // control in world frame, get the target foot position at this phase
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
                    if (desiredStateCommand->stateDes(6, 0) > 0.01) {
                        H = robot->lowerLegLength * 0.6; // very foot height may make real robot crash!
                        // H = robot->lowerLegLength * 0.8; // 0.2*0.8 = 0.16(a1), 0.25*0.8=0.2(aliengo)
                    } else if (desiredStateCommand->stateDes(6, 0) < -0.01){
                        H = 0.1;
                    } else {
                        H = robot->lowerLegLength * 0.6; // 0.2 *0.6 = 0.12(a1), 0.15(aliengo)
                    }

                    Vec3<float> footTargetPositionInWorldFrame = robotics::math::invertRigidTransform(robot->basePosition, robotComOrientation, footTargetPosition);
                    desiredStateCommand->footTargetPositionsInWorldFrame.col(legId) = footTargetPositionInWorldFrame;
                    
                        // std::cout << "phase = " << phase <<
                        //     " (s) " << phaseSwitchFootGlobalPos.col(legId).transpose() <<
                        //     "\n --> (e) " << footTargetPosition.transpose() << std::endl;
            
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

                    foot_pos_error.col(legId) = footPositionInBaseFrame - footPositionsInBaseFrame.col(legId);
                    foot_vel_error.col(legId) = foot_vel_target - foot_vel_cur;
                    footVTarget.col(legId) = foot_vel_target;
                    footVCurrent.col(legId) = foot_vel_cur;
                    
                    stateData.wbcData.pFoot_des[legId] = robotics::math::invertRigidTransform(robot->basePosition, robotComOrientation, footPositionInBaseFrame); // in world Frame
                    stateData.wbcData.vFoot_des[legId] = stateData.baseVInWorldFrame + robotBaseR * footVelocityInBaseFrame;
                    stateData.wbcData.aFoot_des[legId] = robotBaseR * footAccInBaseFrame;
                    
                } break;
                default: {throw std::domain_error("controlParams[mode] is not correct!\n");}
            }
                
            // compute joint position & joint velocity
            robot->ComputeMotorAnglesFromFootLocalPosition(legId, footPositionInBaseFrame, jointIdx, jointAngles);
            Vec3<float> motorVelocity = robot->ComputeMotorVelocityFromFootLocalVelocity(
                legId, jointAngles, footVelocityInBaseFrame);
            // check nan value
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
        count++;
        std::map<int, Matrix<float, 5, 1>> actions;
        Matrix<float, 12, 1> kps, kds;
        kps = robot->GetMotorPositionGains();
        kds = robot->GetMotorVelocityGains();
        // std::cout << "scaleKp = " << scaleKp.transpose() << std::endl;
        
        // force contrl by PD computation
        for (int singleLegId(0); singleLegId<NumLeg; ++singleLegId) {
            bool flag;
            flag = (gaitGenerator->legState[singleLegId] == LegState::SWING) // for trot up to slope,  // in velocity mode
                        || (gaitGenerator->curLegState[singleLegId] == LegState::SWING && !gaitGenerator->allowSwitchLegState[singleLegId]); // for gait schedule mode 
            if (flag) {
                foot_forces_kin.col(singleLegId) = foot_pos_error.block<3, 1>(0, singleLegId).cwiseProduct(kps.segment(3*singleLegId,3))/4.0 + // 40, 4
                                                foot_vel_error.block<3, 1>(0, singleLegId).cwiseProduct(kds.segment(3*singleLegId,3))/1.0; // 5, 10, 2
                // if (singleLegId == 0) {
                //     vis.datax.push_back(count);
                //     vis.datay1.push_back(foot_forces_kin(2,0));
                //     vis.datay5.push_back(foot_pos_error(2,0));
                //     vis.datay2.push_back(foot_vel_error(2,0));
                //     vis.datay3.push_back(desiredFootPositionsInBaseFrame(2,0));
                //     vis.datay4.push_back(footPositionsInBaseFrame(2, 0));
                // }
                Mat3<float> jac = stateData.footJvs[singleLegId];
                Vec3<float> joint_torques = jac.lu().solve(foot_forces_kin.col(singleLegId));
                
                joint_torques = joint_torques.cwiseMax(-20).cwiseMin(20);
                // lastLegTorque.col(singleLegId) = joint_torques;
                
                actions[3*singleLegId] << 0, 0, 0, 0, joint_torques[0];
                actions[3*singleLegId+1] << 0, 0, 0, 0, joint_torques[1];
                actions[3*singleLegId+2] << 0, 0, 0, 0, joint_torques[2];
            
            }
        }
        // return actions;
        
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
                    flag = (gaitGenerator->legState[singleLegId] == LegState::SWING) // for trot up to slope,  // in velocity mode
                            || (gaitGenerator->curLegState[singleLegId] == LegState::SWING && !gaitGenerator->allowSwitchLegState[singleLegId]); // for gait schedule mode 
                } break;
            }
            
            if (flag) {
                // if (it->first % 3 == 0)
                //     std::cout << "[swing] = " << singleLegId << std::endl;
                actions[it->first] << std::get<0>(posVelId), 6 /*kps[it->first]*/, std::get<1>(posVelId), 2 /*kds[it->first]*/, actions[it->first][4];
                // actions[it->first] << std::get<0>(posVelId), kps[it->first], std::get<1>(posVelId),kds[it->first], 0;
            }
        }
        return actions;
    }
} // namespace Quadruped

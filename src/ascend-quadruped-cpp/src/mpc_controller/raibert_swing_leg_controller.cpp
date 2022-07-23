/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Swing Leg Controller
* Author: Xie Ming Cheng & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: add head comment and add some function comments and delete some test functions. @ xie_mingcheng 2021.11.22;
*       add position mode control API for foothold. @ Zhu Yijie 2021.11.24;
*/

#include "mpc_controller/raibert_swing_leg_controller.h"

using namespace Eigen;
using namespace std;
namespace Quadruped {
// The position correction coefficients in Raibert's formula.
    const Matrix<float, 3, 1> swingKp(0.03, 0.03, 0.03);

    float RaibertSwingLegController::GenParabola(float phase, float start, float mid, float end)
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

    Matrix<float, 3, 1> RaibertSwingLegController::GenSwingFootTrajectory(float inputPhase,
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

    RaibertSwingLegController::RaibertSwingLegController(Robot *robot,
                                                         OpenloopGaitGenerator *gaitGenerator,
                                                         RobotEstimator *stateEstimator,
                                                         GroundSurfaceEstimator *groundEstimator,
                                                         FootholdPlanner *footholdPlanner,
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
        // this->Reset(0);
    }

    void RaibertSwingLegController::Reset(float currentTime)
    { 
        phaseSwitchFootLocalPos = robot->state.GetFootPositionsInBaseFrame();
        phaseSwitchFootGlobalPos = robot->state.GetFootPositionsInWorldFrame();
        Matrix<float, 1, 4> footX = MatrixXf::Map(&footInitPose[0][0], 1, 4);
        Matrix<float, 1, 4> footY = MatrixXf::Map(&footInitPose[1][0], 1, 4);
        Matrix<float, 1, 4> footZ = MatrixXf::Map(&footInitPose[2][0], 1, 4);
        // footHoldInWorldFrame << footX, footY, footZ;
        // footHoldInWorldFrame(0, 0) = footHoldInWorldFrame(0, 1) - footOffset;
        // footHoldInWorldFrame(0, 3) = footHoldInWorldFrame(0, 2) - footOffset;         
        
        footHoldInWorldFrame.row(0) << 0.185f, 0.185f, -0.175f, -0.175f;
        footHoldInWorldFrame.row(1) << -0.145f, 0.145f, -0.145f, 0.145f;
        footHoldInWorldFrame.row(2) << 0.f, 0.f, 0.f, 0.f;
        footHoldInWorldFrame(0, 0) -= 0.05;
        footHoldInWorldFrame(0, 3) -= 0.05;
        footHoldInControlFrame = phaseSwitchFootLocalPos;
        
        switch (robot->controlParams["mode"]) {
            case LocomotionMode::POSITION_LOCOMOTION: {
                // footHoldInWorldFrame = phaseSwitchFootGlobalPos; //todo reset by default foot pose setting
                std::cout << "[SwingLegController Reset] phaseSwitchFootLocalPos: \n" << phaseSwitchFootLocalPos
                        << std::endl;
                std::cout << "[SwingLegController Reset] phaseSwitchFootGlobalPos: \n" << phaseSwitchFootGlobalPos
                        << std::endl;
                std::cout << "[SwingLegController Reset] footHoldInWorldFrame: \n" << footHoldInWorldFrame << std::endl;
            } break;
            case LocomotionMode::WALK_LOCOMOTION: {
                footHoldInWorldFrame = phaseSwitchFootGlobalPos; //todo reset by default foot pose setting
                std::cout << "[SwingLegController Reset] phaseSwitchFootLocalPos: \n" << phaseSwitchFootLocalPos
                        << std::endl;
                std::cout << "[SwingLegController Reset] phaseSwitchFootGlobalPos: \n" << phaseSwitchFootGlobalPos
                        << std::endl;
                std::cout << "[SwingLegController Reset] footHoldInWorldFrame: \n" << footHoldInWorldFrame << std::endl;
            } break;
            default: break;  
        }

        swingJointAnglesVelocities.clear();
    }

    void RaibertSwingLegController::Update(float currentTime)
    {
        // swingJointAnglesVelocities.clear();
        const Vec4<int>& newLegState = gaitGenerator->desiredLegState;
        const Vec4<int>& curLegState = gaitGenerator->curLegState;
        // the footHoldOffset is first init at robot.h, then update it at groundEstimator.cpp 
        Eigen::Matrix<float, 3, 1> constOffset = {robot->config->footHoldOffset, 0.f, 0.f};
        
        // Detects phase switch for each leg so we can remember the feet position at
        // the beginning of the swing phase.
        switch (robot->controlParams["mode"]) {
            case LocomotionMode::POSITION_LOCOMOTION: {
                for (int legId = 0; legId < NumLeg; ++legId) {
                    if ((newLegState(legId) == LegState::SWING || newLegState(legId) == LegState::USERDEFINED_SWING)
                        && curLegState(legId) == LegState::STANCE 
                        && !robot->stop) {
            
                        phaseSwitchFootLocalPos.col(legId) = robot->state.GetFootPositionsInBaseFrame().col(legId);
                        phaseSwitchFootGlobalPos.col(legId) = robot->state.GetFootPositionsInWorldFrame().col(legId);
                        // std::cout << "[SwingLegController::Update] phaseSwitchFootGlobalPos.col(legId): \n"
                        //         << phaseSwitchFootGlobalPos.col(legId) << std::endl;
                        // std::cout << "[SwingLegController::Update] footHoldInWorldFrame.col(legId): \n"
                        //         << footHoldInWorldFrame.col(legId) << std::endl;
                        if (legId == 0) { //update four leg footholds
                            footholdPlanner->UpdateOnce(footHoldInWorldFrame); // based on the last foothold position
                        }
                        
                        // footHoldInWorldFrame.col(legId) += constOffset;
                        // get the foothold offset of this leg by id
                        // std::cout << "offset " << footholdPlanner->GetFootholdsOffset().col(legId) <<std::endl;
                        footHoldInWorldFrame.col(legId) += footholdPlanner->GetFootholdsOffset().col(legId);
                        // cout << "[SwingLegController::Update] update footHoldInWorldFrame: \n"
                        //     << footHoldInWorldFrame.col(legId) << endl;
                    }
                }
            } break;
            case LocomotionMode::WALK_LOCOMOTION: {
                for (int legId = 0; legId < NumLeg; ++legId) {
                    if ((newLegState(legId) == SubLegState::TRUE_SWING || newLegState(legId) == LegState::USERDEFINED_SWING)
                        && curLegState(legId) == SubLegState::UNLOAD_FORCE 
                        && !robot->stop) {
                        Quat<float> robotComOrientation = robot->GetBaseOrientation();
                        Vec3<float> robotComRPY = robot->GetBaseRollPitchYaw();
                        Mat3<float> Rb = robotics::math::quaternionToRotationMatrix(robotComOrientation).transpose();
                        controlFrameOrientationSource = groundEstimator->GetControlFrameOrientation();
                        Vec3<float> groundRPY = groundEstimator->GetControlFrameRPY();
                        Mat3<float> RcSource = robotics::math::quaternionToRotationMatrix(controlFrameOrientationSource).transpose();
                        Mat3<float> Rcb = RcSource.transpose() * Rb;
                        controlFrameOriginSource = robot->GetBasePosition();
                        phaseSwitchFootLocalPos.col(legId) = robot->state.GetFootPositionsInBaseFrame().col(legId);
                        phaseSwitchFootGlobalPos.col(legId) = robot->state.GetFootPositionsInWorldFrame().col(legId);
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
                        if (
                            robot->config->isSim) {
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
                        
                        SplineInfo splineInfo;
                        splineInfo.splineType = "BSpline";
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
                    if (newLegState(legId) == LegState::SWING && newLegState(legId) != gaitGenerator->lastLegState(legId)) {
                        phaseSwitchFootLocalPos.col(legId) = robot->state.GetFootPositionsInBaseFrame().col(legId);
                    }
                }
            } break;
        }
    }

    map<int, Matrix<float, 5, 1>> RaibertSwingLegController::GetAction()
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
                         - robotics::math::TransformVecByQuat(robotics::math::quatInverse(robot->state.baseOrientation), desiredHeight);
                        //  - robotBaseR.transpose() * desiredHeight;
                    
                    // swingFootTrajectories[legId] = SwingFootTrajectory(phaseSwitchFootLocalPos.col(legId),footTargetPosition);
                    // bool flag = swingFootTrajectories[legId].GenerateTrajectoryPoint(footPositionInBaseFrame,
                    //                                                                 footVelocityInBaseFrame,
                    //                                                                 footAccInBaseFrame,
                    //                                                                 gaitGenerator->normalizedPhase(legId),
                    //                                                                 false);
                    footPositionInBaseFrame = GenSwingFootTrajectory(gaitGenerator->normalizedPhase[legId],
                                                                     phaseSwitchFootLocalPos.col(legId),
                                                                     footTargetPosition);
                } break;
                case LocomotionMode::POSITION_LOCOMOTION: {
                    footPositionInWorldFrame = GenSwingFootTrajectory(gaitGenerator->normalizedPhase[legId],
                                                                      phaseSwitchFootGlobalPos.col(legId),
                                                                      footHoldInWorldFrame.col(legId)); // interpolation in world frame
                    footPositionInBaseFrame = robotics::math::RigidTransform(robot->state.basePosition,
                                                                             robot->state.baseOrientation,
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
                    if (robot->config->isSim) {
                        flag = swingFootTrajectories[legId].GenerateTrajectoryPoint(footPositionInWorldFrame,
                                                                                        footVelocityInWorldFrame,
                                                                                        footAccInWorldFrame, 
                                                                                        gaitGenerator->normalizedPhase[legId],
                                                                                        false);
                        footPositionInBaseFrame = robotics::math::RigidTransform(robot->state.basePosition,
                                                                                robot->state.baseOrientation,
                                                                                footPositionInWorldFrame); // transfer to base frame
                        footVelocityInBaseFrame = robotics::math::RigidTransform({0.f, 0.f, 0.f},
                                                                                robot->state.baseOrientation,
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
                    // std::cout << "[SWING] foot in world frame" << robot->state.GetFootPositionsInWorldFrame() << std::endl;
                    
                    // footPositionInWorldFrame = robotics::math::invertRigidTransform(controlFrameOriginSource,
                    //                                                             controlFrameOrientationSource,
                    //                                                             footPositionInControlFrame); // transfer to world frame                    
                } break;
                default: {throw std::domain_error("controlParams[mode] is not correct!\n");}
            }
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
            // if (invalidAngleNum > 0) {
            //     printf("[warning] swing leg controll receive nan value!\n");
            // } 
        }
        count++;
        auto RLAngles = currentJointAngles.tail(3);
        map<int, Matrix<float, 5, 1>> actions;
        Matrix<float, 12, 1> kps, kds;
        kps = robot->config->motorKps;
        kds = robot->config->motorKds;
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
                // flag = (gaitGenerator->legState[singleLegId] == LegState::SWING); // for trot up to slope,  // in velocity mode
            }
            
            if (flag) {
                actions[it->first] << std::get<0>(posVelId), kps[it->first], std::get<1>(posVelId), kds[it->first], 0.f;
                // std::cout << "[SWING " << singleLegId <<", phase="<< gaitGenerator->normalizedPhase[singleLegId]<< " command = " << actions[it->first].transpose() << std::endl;
                // if (singleLegId==2) {
                //     datay.push_back(std::get<0>(posVelId));
                    // datax.push_back(count);
                //     if (it->first==9)
                //         datay1.push_back(RLAngles[0]);
                //     else if(it->first==10)
                //         datay2.push_back(RLAngles[1]);
                //     else 
                //         datay3.push_back(RLAngles[2]);
                // }
            }
        } 
        /*
        Vec3<float> tmp = footHoldInWorldFrame.rowwise().mean();
        datax.push_back(tmp[0]);
        datay1.push_back(tmp[1]);
        datay2.push_back(robot->basePosition[0]-tmp[0]);
        datay3.push_back(robot->basePosition[1]-tmp[1]);
        datay.push_back(count);
        if (count > 50000) {
            // plot
            // if (datay3.size()!=datay2.size() || datay2.size() != datay1.size()
            //     || datay3.size()*3 != datax.size()) {
            //     printf("datay size not right!\n");
            //     exit(0);
            // }
            plt::figure();
            // plt::plot(datax,datay);
            std::map<std::string,std::string> map_;
            map_["cmap"] = "viridis";
            map_["alpha"] = "0.5";
            // plt::scatter(datax, datay1, (2.0));
            plt::scatter(datay, datay2, (1.0), {{"label", "dx"}});
            plt::scatter(datay,datay3, (1.0), {{"label", "dy"}});
            // plt::scatter(dataxx,datay3, (1.0), {{"label", "knee_cur"}});
            
            // std::vector<float> dataAB, dataHip, dataKnee, dataxx;
            // for (int i=0; i+2<datax.size(); i=i+3) {
            //     dataAB.push_back(datay[i]);
            //     dataHip.push_back(datay[i+1]);
            //     dataKnee.push_back(datay[i+2]);
            //     dataxx.push_back(datax[i]);
            // }
            // plt::scatter(dataxx,dataAB, (1.0), {{"label", "abad"}});
            // plt::scatter(dataxx,dataHip, (1.0), {{"label", "hip"}});
            // plt::scatter(dataxx,dataKnee, (1.0), {{"label", "knee"}});
            // plt::scatter(dataxx, datay1, (1.0), {{"label", "abad_cur"}});
            // plt::scatter(dataxx,datay2, (1.0), {{"label", "hip_cur"}});
            // plt::scatter(dataxx,datay3, (1.0), {{"label", "knee_cur"}});
            
            plt::grid(true);
            // auto ax = plt::gac();
            plt::legend();
            plt::show();
            exit(0);
        
        }
        */
        
        return actions;
    }
} // namespace Quadruped

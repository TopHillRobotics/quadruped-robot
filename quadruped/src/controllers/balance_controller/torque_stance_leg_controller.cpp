/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Stance controller for stance foot.
* Author: Zang Yaohua & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zang Yaohua
*       add computing desired acceleration in world frame @ Zhu Yijie & Zang 2022-02-01;
*       add controlModeStr @ Zhu Linsen 2022-03-01;
*       add UpdateFRatio @ Zhu Yijie 2022-03-01;
*       add UpdateDesCommand @ Zhu Yijie 2022-04-01;
*/

#include "controllers/balance_controller/torque_stance_leg_controller.h"
#include "controllers/balance_controller/qp_torque_optimizer.h"
using namespace std;

namespace Quadruped {

    TorqueStanceLegController::TorqueStanceLegController(Robot *robot,
                                                         GaitGenerator *gaitGenerator,
                                                         StateEstimatorContainer<float>* stateEstimators,
                                                         ComAdjuster *comAdjuster,
                                                         PosePlanner *posePlanner,
                                                         FootholdPlanner *footholdPlanner,
                                                         UserParameters& userParameters,
                                                         std::string configFilepath
                                                         )
    {
        this->robot = robot;
        this->gaitGenerator = gaitGenerator;
        this->robotEstimator = stateEstimators->GetRobotEstimator();
        this->groundEstimator = stateEstimators->GetGroundEstimator();
        this->comAdjuster = comAdjuster;
        this->posePlanner = posePlanner;
        this->footholdPlanner = footholdPlanner;
        this->desiredSpeed << userParameters.desiredSpeed[0],
                            userParameters.desiredSpeed[1],
                            userParameters.desiredSpeed[2];
        this->desiredTwistingSpeed = userParameters.desiredTwistingSpeed;
        this->desiredBodyHeight = userParameters.desiredHeight;
        this->frictionCoeffs << userParameters.frictionCoeffs[0],
                                userParameters.frictionCoeffs[1],
                                userParameters.frictionCoeffs[2],
                                userParameters.frictionCoeffs[3];
        this->computeForceInWorldFrame = userParameters.computeForceInWorldFrame;
        
        this->param = YAML::LoadFile(configFilepath);
        
        Reset(0.f);
    }

    void TorqueStanceLegController::Reset(float currentTime_)
    {
        currentTime = currentTime_;

        controlModeStr = robot->GetControlMode();
        this->force_dim = param["stance_leg_params"]["force_dim"].as<int>();        
        vector<float> v = param["stance_leg_params"][controlModeStr]["KD"].as<vector<float>>();
        this->KD = Eigen::MatrixXf::Map(&v[0], 6, 1);
        v = param["stance_leg_params"][controlModeStr]["KP"].as<vector<float>>();
        this->KP = Eigen::MatrixXf::Map(&v[0], 6, 1);
        v = param["stance_leg_params"][controlModeStr]["max_ddq"].as<vector<float>>();
        this->maxDdq = Eigen::MatrixXf::Map(&v[0], 6, 1);
        v = param["stance_leg_params"][controlModeStr]["min_ddq"].as<vector<float>>();
        this->minDdq = Eigen::MatrixXf::Map(&v[0], 6, 1);
        v = param["stance_leg_params"][controlModeStr]["acc_weight"].as<vector<float>>();
        this->accWeight = Eigen::MatrixXf::Map(&v[0], 6, 1);        
    }

    void TorqueStanceLegController::Update(float currentTime_)
    {
        currentTime = currentTime_;
    }

    void TorqueStanceLegController::UpdateFRatio(Vec4<bool> &contacts, int &N, float &moveBasePhase)
    {
        moveBasePhase = 1.f;
        N=0;
        /// leg contact status  ///
        if (robot->stop) {
            printf("robot->stop");
            fMaxRatio << 10., 10., 10., 10.;
            fMinRatio << 0.01, 0.01, 0.01, 0.01;
            contacts << true, true, true, true;
            N = 4; 
            return;
        } else if (robot->controlParams["mode"] != LocomotionMode::WALK_LOCOMOTION) {
            fMaxRatio << 10., 10., 10., 10.;
            fMinRatio << 0.01, 0.01, 0.01, 0.01;
            for (int legId=0; legId<NumLeg; ++legId){
                bool flag;
                if (robot->controlParams["mode"] == LocomotionMode::VELOCITY_LOCOMOTION) {
                    flag = (gaitGenerator->desiredLegState[legId] == LegState::STANCE);
                } else {
                    flag = ((gaitGenerator->desiredLegState[legId] == LegState::STANCE && gaitGenerator->allowSwitchLegState[legId])
                            || gaitGenerator->legState[legId] == LegState::EARLY_CONTACT);
                }
                if (flag) {
                    contacts[legId] = true;
                    N++;
                } else {
                    contacts[legId] = false;
                }
            }
            return;
        } else {
            for (int legId = 0; legId < NumLeg; ++legId) {
                int desiredLegState = gaitGenerator->desiredLegState[legId];
                int detectedLegState = gaitGenerator->detectedLegState[legId];
                float phase = gaitGenerator->normalizedPhase[legId];
                
                if (detectedLegState == LegState::STANCE || detectedLegState == LegState::LOSE_CONTACT) {
                    contacts[legId] = true;
                    N++;
                    fMaxRatio[legId] = 10.0;
                    fMinRatio[legId] = 0.001;
                } else if(detectedLegState == LegState::EARLY_CONTACT) { // plan is swing, actual is stand
                        contacts[legId] = true;
                        N++;
                        float tempRatio = abs(phase-0.8);///(1.0-detectedEventTickPhase[legId]);
                        fMaxRatio[legId] = 10.0 * std::min(0.01f, tempRatio);
                        fMinRatio[legId] = 0.001;
                } else { // Swing STATE in plan
                    moveBasePhase = gaitGenerator->moveBasePhase;
                    if (desiredLegState==SubLegState::LOAD_FORCE) {
                        contacts[legId] = true;
                        N++;
                        fMaxRatio[legId] = 10.0 * std::max(0.001f, phase);
                        fMinRatio[legId] = 0.001;
                    } else if (desiredLegState==SubLegState::UNLOAD_FORCE){
                        contacts[legId] = true;
                        N++;
                        phase = phase / (3.f/4.0); // todo
                        fMaxRatio[legId] = 10.0 * std::max(0.001, 1.0-phase);
                        fMinRatio[legId] = 0.001;
                    } else if (desiredLegState==SubLegState::TRUE_SWING) {
                        contacts[legId] = false;
                        fMaxRatio[legId] = 0.002;
                        fMinRatio[legId] = 0.001;
                    }  else if (desiredLegState==SubLegState::FULL_STANCE) {
                        contacts[legId] = true;
                        N++;
                        fMaxRatio[legId] = 10.0;
                        fMinRatio[legId] = 0.001;
                    } else {
                        throw std::invalid_argument("no this leg state");
                    }
                }
            }
        }
    }

    void TorqueStanceLegController::UpdateDesCommand()
    {
        Eigen::Matrix<float, 3, 1> robotComPosition;
        Eigen::Matrix<float, 3, 1> robotComVelocity;
        Eigen::Matrix<float, 3, 1> robotComRpy;
        Eigen::Matrix<float, 3, 1> robotComRpyRate;
        Eigen::Matrix<float, 6, 1> robotQ;
        Eigen::Matrix<float, 6, 1> robotDq;

        Eigen::Matrix<float, 3, 1> desiredComPosition(0.0, 0.0, 0.0);
        Eigen::Matrix<float, 3, 1> desiredComVelocity(0.0, 0.0, 0.0);
        Eigen::Matrix<float, 3, 1> desiredComRpy(0.0, 0.0, 0.0);
        Eigen::Matrix<float, 3, 1> desiredComAngularVelocity(0.0, 0.0, 0.0);
        Eigen::Matrix<float, 6, 1> desiredQ;
        Eigen::Matrix<float, 6, 1> desiredDq;
        Eigen::Matrix<float, 6, 1> desiredDdq;
        
        UpdateFRatio(contacts, N, moveBasePhase);
        
        Eigen::Matrix<float, 3, 4> footPoseWorld = robot->GetFootPositionsInWorldFrame();
        
        Vec6<float> pose;
        Vec6<float> twist; // v, wb 
        if (robot->controlParams["mode"]==LocomotionMode::WALK_LOCOMOTION) {
            computeForceInWorldFrame = true;
            if (!robot->stop) {
                auto res = posePlanner->GetIntermediateBasePose(moveBasePhase, currentTime); //todo  in world frame;
                pose = std::get<0>(res);
                twist = std::get<1>(res);
            } else {
                auto res = posePlanner->GetIntermediateBasePose(currentTime);
                // auto res = posePlanner->GetIntermediateBasePose(1.0, currentTime); //todo  in world frame;
                pose = std::get<0>(res);
                twist = std::get<1>(res);  
            }
        } else {
            pose.head(3) = robot->GetBasePosition();
        }
        // pose[0] = max(pose[0], robot->GetBasePosition()[0]);
        
        Eigen::Matrix<int, 3, 4> jointIdxs;
        Eigen::Matrix<int, 3, 1> jointIdx;
        Eigen::Matrix<float, 3, 1> jointAngles;
        Eigen::Matrix<float, 3, 4> com2FootInWorld = footPoseWorld.colwise() - pose.head(3);
        
        Quat<float> robotComOrientation = robot->GetBaseOrientation();
        Mat3<float> Rb = robot->stateDataFlow.baseRMat;
        Quat<float> controlFrameOrientation = groundEstimator->GetControlFrameOrientation();
        Mat3<float> Rc = robot->stateDataFlow.groundRMat;
        Vec3<float> groundRPY = groundEstimator->GetControlFrameRPY();
        Mat3<float> Rcb = robot->stateDataFlow.baseRInControlFrame;
        
        /// current robot status  ///
        robotComRpy = robot->GetBaseRollPitchYaw(); // world frame
        // std::cout << "robotComRpy = " << robotComRpy << std::endl;
        robotComVelocity = robotEstimator->GetEstimatedVelocity();  // base frame
        robotComRpyRate = robot->GetBaseRollPitchYawRate();  // base frame
        // std::cout <<  "mode = " << robot->controlParams["mode"] << std::endl;
        switch (robot->controlParams["mode"]) {
            case LocomotionMode::VELOCITY_LOCOMOTION: {
                computeForceInWorldFrame = false;
                robotComPosition = {0., 0., robot->basePosition[2]}; // vel mode in base frame, height is in world frame.                
                
                if (groundEstimator->terrain.terrainType>=2) { // not horizontal plane
                    robotComPosition = robotics::math::TransformVecByQuat(robotics::math::quatInverse(controlFrameOrientation), robotComPosition);      
                    robotComPosition[0] = 0.f;
                    robotComPosition[1] = 0.f;
                    robotComVelocity = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComVelocity); // in world frame
                    robotComVelocity = robotics::math::RigidTransform({0,0,0}, controlFrameOrientation, robotComVelocity); // in control frame
                    robotComRpy = robotics::math::rotationMatrixToRPY(Rcb.transpose()); // body orientation in control frame.
                    robotComRpyRate = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComRpyRate); // in world frame
                    robotComRpyRate = robotics::math::RigidTransform({0,0,0}, controlFrameOrientation, robotComRpyRate); // in control frame
                } else { // on ground                    
                    robotComVelocity = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComVelocity); // in world frame
                    robotComRpyRate = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComRpyRate); // in world frame
                    if (!computeForceInWorldFrame) { // conrtrol frame
                        robotComRpy[2] = 0.f;
                        robotComVelocity = robotics::math::RigidTransform({0,0,0}, controlFrameOrientation, robotComVelocity); // in control frame
                        robotComRpyRate = robotics::math::RigidTransform({0,0,0}, controlFrameOrientation, robotComRpyRate); // in control frame
                    }
                }
                // robotComPosition = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComPosition); // in world frame
                // robotComPosition = robotics::math::RigidTransform({0,0,0}, controlFrameOrientation, robotComPosition); // in control frame
            } break;
            case LocomotionMode::ADVANCED_TROT: {
                if (computeForceInWorldFrame) {
                    robotComPosition = {0., 0., robot->basePosition[2]}; // vel mode in base frame, height is in world frame.
                    // robotComRpy[2] = 0.f;
                    robotComVelocity = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComVelocity); // in world frame 
                    robotComRpyRate = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComRpyRate); // in world frame
                } else { // in control frame
                    robotComPosition = {0., 0., robot->basePosition[2]}; // vel mode in base frame, height is in world frame.
                    robotComRpy[2] = 0.f;
                    if (groundEstimator->terrain.terrainType>=2) { // not horizontal plane
                        robotComPosition = robotics::math::TransformVecByQuat(robotics::math::quatInverse(controlFrameOrientation), robotComPosition);      
                        robotComPosition[0] = 0.f;
                        robotComPosition[1] = 0.f;
                        robotComPosition = {0., 0., robot->stateDataFlow.heightInControlFrame}; // vel mode in base frame, height is in world frame.
                    
                        robotComVelocity = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComVelocity); // in world frame
                        robotComVelocity = robotics::math::RigidTransform({0,0,0}, controlFrameOrientation, robotComVelocity); // in control frame
                        
                        robotComRpy = robotics::math::rotationMatrixToRPY(Rcb.transpose()); // body orientation in control frame.
                        robotComRpyRate = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComRpyRate); // in world frame
                        robotComRpyRate = robotics::math::RigidTransform({0,0,0}, controlFrameOrientation, robotComRpyRate); // in control frame
                    }
                }
                
                // robotComPosition = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComPosition); // in world frame
                // robotComPosition = robotics::math::RigidTransform({0,0,0}, controlFrameOrientation, robotComPosition); // in control frame
            } break;
            case LocomotionMode::WALK_LOCOMOTION: {
                robotComPosition = robot->GetBasePosition(); // in world frame, walk mode
                robotComVelocity = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComVelocity); // in world frame
                robotComRpyRate = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComRpyRate); // in world frame   
            } break;
            case LocomotionMode::POSITION_LOCOMOTION: {
                computeForceInWorldFrame = false;
                robotComPosition = {0., 0., robot->basePosition[2]};
            } break;
            default: {
                throw std::domain_error("no such LocomotionMode");
            }
        }
        
        robotQ << robotComPosition, robotComRpy;
        robotDq << robotComVelocity, robotComRpyRate;
        // std::cout << "[torque stance]: des command = " << desiredStateCommand->stateDes << std::endl;
        /// desired robot status  ///
        switch (robot->controlParams["mode"]) {
            case LocomotionMode::VELOCITY_LOCOMOTION: {
                desiredComPosition << 0.f, 0.f, desiredBodyHeight;
                // desiredComRpy << 0, 0, 0; // in base frame
                desiredComRpy << -groundRPY[0], 0, -groundRPY[2]; // not control roll/yaw
                /*
                desiredComVelocity = {desiredSpeed[0], desiredSpeed[1], 0.f}; // in base/control frame
                // desiredComVelocity = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, desiredComVelocity); // in world frame
                desiredComAngularVelocity = {0.f, 0.f, desiredTwistingSpeed};
                */
                desiredComVelocity = desiredStateCommand->stateDes.segment(6,3); // in base/control frame
                desiredComAngularVelocity = desiredStateCommand->stateDes.segment(9,3); // in base frame
                
                if (computeForceInWorldFrame) {
                    desiredComRpy = robotComRpy + robot->timeStep * desiredComAngularVelocity;
                    desiredComRpy[0] = 0; // roll
                    // desiredComRpy[1] = groundRPY[1]; pitch in world frame
                    desiredComVelocity = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, desiredComVelocity); // in world frame
                    // desiredComAngularVelocity
                }
                /*
                Quat<float> intermediateQuat = robotics::math::rpyToQuat(desiredComRpy);
                com2FootInWorld = footPoseWorld.colwise() - desiredComPosition;
                std::cout << "com2FootInWorld = " <<com2FootInWorld << std::endl;
                for(int i=0; i<4; ++i) {
                    Vec3<float> r = robotics::math::RigidTransform(Vec3<float>(0.,0.,0.), intermediateQuat, Vec3<float>(com2FootInWorld.col(i)));
                    robot->ComputeMotorAnglesFromFootLocalPosition(i, r, jointIdx, jointAngles);
                    desiredStateCommand->legJointq.block<3,1>(0, i) = jointAngles;
                    jointIdxs.block<3,1>(0,i) = jointIdx;
                }
                */                
            } break;
            case LocomotionMode::ADVANCED_TROT: {
                if (computeForceInWorldFrame) {
                    auto &comAdjPosInBaseFrame = comAdjuster->GetComPosInBaseFrame();
                    Vec3<float> newComPosInWorldFrame = Rb * comAdjPosInBaseFrame + robot->basePosition;
                    desiredComPosition = {newComPosInWorldFrame[0], newComPosInWorldFrame[1],
                                        desiredStateCommand->stateDes(2)}; // todo
                    
                    // desiredComPosition << 0, 0, desiredStateCommand->stateDes(2);
                    float pitch = groundRPY[1];
                    float pitchMax = 0.5;
                    if (abs(pitch) < 0.1) {
                        pitch = 0;
                    } else if (pitch > pitchMax) {
                        pitch = pitchMax;
                    } else if (pitch < -pitchMax) {
                        pitch = -pitchMax;
                    }
                    desiredComRpy << 0.f, pitch, 0.f;
                    // std::cout << "pitch = " << pitch << std::endl;
                    Eigen::Matrix<float,3,4> footPosInBaseFrame = robot->GetFootPositionsInBaseFrame();
                    float scaleFactor = 1;
                    // Eigen::Matrix<float,3,4> com2FootInWorldFrame = footPoseWorld.colwise() - robot->basePosition;
                    float footX = std::min(footPosInBaseFrame(0, 0), footPosInBaseFrame(0, 1));
                    if (footX < 0.1) {
                        scaleFactor = std::max(0.1f, footX / 0.1f);
                    }
                    // std::cout << "scaleFactor = " << scaleFactor << std::endl;
                    // is com in support polygon ?
                    // todo

                    desiredComVelocity = (scaleFactor * desiredStateCommand->stateDes.segment(6,3)); // in base
                    if (pitch < 0.1 && desiredComVelocity[2] > 0.01) {
                        desiredComPosition[2] += 0.04 * abs(pitch/pitchMax); // todo
                    } 
                    desiredComAngularVelocity = desiredStateCommand->stateDes.segment(9,3); // in base
                } else { // in control frame
                    desiredComPosition << 0.f, 0.f, desiredBodyHeight*std::abs(std::cos(groundRPY[1]));
                    desiredComPosition[2] = robotComPosition[2]*0.7 + desiredComPosition[2]*0.3;
                    desiredComRpy << -groundRPY[0], 0.f, -groundRPY[2]; // not control roll/yaw
                    desiredComVelocity = {desiredSpeed[0], desiredSpeed[1], 0.f}; // in base/control frame
                    desiredComVelocity = Rc * desiredComVelocity;
                    desiredComAngularVelocity = {0.f, 0.f, desiredTwistingSpeed};
                }      
            } break;
            case LocomotionMode::WALK_LOCOMOTION: {
                for (int i = 0; i < NumLeg; ++i) {
                    float phase = gaitGenerator->normalizedPhase[i];
                        
                    if ((contacts[i] && gaitGenerator->desiredLegState[i] == SubLegState::UNLOAD_FORCE && phase > 3.0/4) 
                        || robot->stop) {
                        Quat<float> intermediateQuat = robotics::math::rpyToQuat(Vec3<float>(pose.tail(3)));
                        Vec3<float> dr = Vec3<float>(0.f,0.f,0.01f) / 2000;
                        Vec3<float> r = robot->GetFootPositionsInBaseFrame().col(i) + dr;
                        robot->ComputeMotorAnglesFromFootLocalPosition(i, r, jointIdx, jointAngles);
                        desiredStateCommand->legJointq.block<3,1>(0, i) = jointAngles;
                        jointIdxs.block<3,1>(0,i) = jointIdx;
                    } else {
                        for(int j=0;j<3;j++) {
                            jointIdxs(j,i) = 3*i+j;
                        }
                    }
                }
                // pose in world frame
                desiredComPosition = pose.head(3);  // world frame
                desiredComRpy = pose.tail(3); // world frame
                // if (footPosInWorldFrame(0,0) > 1.95) {
                //     desiredComRpy[1] = (desiredComRpy[1] + 0.35)/2.0;
                // }
                desiredComRpy[1] = std::max(-0.35f, std::min(desiredComRpy[1], 0.35f));
                desiredComVelocity = twist.head(3); // world frame
                desiredComAngularVelocity = twist.tail(3); // world frame
                // desiredComVelocity = {desiredSpeed[0], desiredSpeed[1], 0.f}; // control frame
                // desiredComAngularVelocity = {0.f, 0.f, desiredTwistingSpeed}; // control frame
            } break;
            case LocomotionMode::POSITION_LOCOMOTION: {
                auto &comAdjPosInBaseFrame = comAdjuster->GetComPosInBaseFrame();
                desiredComPosition = {comAdjPosInBaseFrame[0], comAdjPosInBaseFrame[1],
                                    desiredBodyHeight}; // get goal com position from comAdjuster, base frame
                desiredComVelocity = {desiredSpeed[0], desiredSpeed[1], 0.f};
                // get goal rpy from footholdPlanner, in world frame
                desiredComRpy = footholdPlanner->GetDesiredComPose().tail(3);
                desiredComAngularVelocity = {0.f, 0.f, 0.f};
            } break;
        }
        desiredQ << desiredComPosition, desiredComRpy;
        //std::cout << "desiredQ" << desiredQ.transpose() << std::endl;
        //std::cout << "robotQ" << robotQ.transpose() << std::endl;
        Vec6<float> dq = desiredQ - robotQ;
        desiredDq << desiredComVelocity, desiredComAngularVelocity;
        Vec6<float> ddq = desiredDq - robotDq;
        //std::cout << "desiredDq" << desiredDq.transpose() << std::endl;
        //std::cout << "robotDq" << robotDq.transpose() << std::endl;
        
        if (computeForceInWorldFrame && robot->controlParams["mode"]!=LocomotionMode::ADVANCED_TROT) {
            // dq
            // method 1: R(dR^T-->rpy)
            Mat3<float> robotR = robotics::math::rpyToRotMat(robotComRpy).transpose();
            Mat3<float> desiredRobotRT = robotics::math::rpyToRotMat(desiredComRpy);
            Mat3<float> dR = desiredRobotRT*robotR;
            dq.tail(3) = robotR * robotics::math::rotationMatrixToRPY(dR);
            //  method 2:   (R*dR)^T --> rpy
            // dq.tail(3) = robotics::math::rotationMatrixToRPY(robotR * dR);
            
            // ddq
            // method 1 : R*((/hat(WBdes) - /hat(WBcurr))---> to skewV)
            Mat3<float> RTWdes = robotics::math::vectorToSkewMat(desiredRobotRT *  desiredComAngularVelocity);
            Mat3<float> RTWcur = robotics::math::vectorToSkewMat(robotR.transpose() *  robotComRpyRate);
            Vec3<float> dw = robotR * robotics::math::matToSkewVec(RTWdes - RTWcur);
            ddq.tail(3) =  dw;
            // method 2: do nothing
        } else if(groundEstimator->terrain.terrainType >= 2) { // not horizontal ground, computeForceInControlFrame
            // dq.tail(3) = Rcb * dq.tail(3); // todo, input should not be represent in base frame
        }

        desiredDdq = KP.cwiseProduct(dq) + KD.cwiseProduct(ddq);
        desiredDdq = desiredDdq.cwiseMin(maxDdq).cwiseMax(minDdq); // Clip
        // std::cout << "desiredDdq" << desiredDdq.transpose() << std::endl;

        desiredStateCommand->stateDes << desiredQ, desiredDq;
        desiredStateCommand->stateCur << robotQ, robotDq;
        desiredStateCommand->ddqDes.head(6) = desiredDdq;        
        // std::cout << "contact for force compute " << contacts.transpose() << std::endl;    
    }
    
    std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> TorqueStanceLegController::GetAction()
    {
        ++count;
        UpdateDesCommand();

        Vec6<float> desiredDdq = desiredStateCommand->ddqDes.head(6);
        Eigen::Matrix<float, 3, 4> contactForces;
        
        /// Compute Contact Force  ///
        Mat3<float> directionVectors = Mat3<float>::Identity(); // friction cone direction in xyz axises
        // Mat3<float> directionVectors = groundEstimator->GetAlignedDirections();
        // std::cout << "directionVectors = " << directionVectors << std::endl;      
        if (computeForceInWorldFrame) {
            contactForces << ComputeContactForce(robot, desiredDdq,  // all in world frame
                                                contacts, accWeight,
                                                directionVectors.col(2), 
                                                directionVectors.col(0),
                                                directionVectors.col(1),
                                                fMinRatio,
                                                fMaxRatio);
        } else {
            contactForces << ComputeContactForce(robot, groundEstimator, desiredDdq, contacts, accWeight); // compute the force in control/base frame        
        }
        // std::cout << "contactForces" << contactForces << std::endl;
        
        std::map<int, MotorCommand> action;
        std::map<int, float> motorTorques;
        Eigen::Matrix<float, 12, 1> kps = robot->GetMotorPositionGains();
        Eigen::Matrix<float, 12, 1> kds = robot->GetMotorVelocityGains();
        
        for (int legId = 0; legId < NumLeg; ++legId) {
            motorTorques = robot->MapContactForceToJointTorques(legId, contactForces.col(legId));    
            MotorCommand temp;
            switch (robot->controlParams["mode"]) {
                case LocomotionMode::WALK_LOCOMOTION: {
                    for (int motorId=0; motorId<3; motorId++) {    
                        if (contacts[legId]) {
                            temp = {0*desiredStateCommand->legJointq(motorId, legId), 0*kps(3*legId+motorId), 0., 0.5*kds(3*legId+motorId), motorTorques[3*legId+motorId]};
                            // if (gaitGenerator->desiredLegState[legId]==SubLegState::UNLOAD_FORCE && gaitGenerator->normalizedPhase[legId] > 3.f/4) {
                            //     std::cout << "legID=" << legId << " should lift the foot a little." << std::endl; 
                            //     temp = {desiredStateCommand->legJointq(motorId, legId), kps(3*legId+motorId), 0., kds(3*legId+motorId), 0*motorTorques[3*legId+motorId]};
                            // }
                        } else if ((N<4 && moveBasePhase<0.7) || robot->stop) {
                            temp = {0, 0, 0., kds(3*legId+motorId)*0, motorTorques[3*legId+motorId]};
                        } else { // moveBasePhase > 0.7
                            temp = {0., 0., 0., 0., 0.};
                        }       
                        action[motorId + 3*legId] = temp;
                    }   
                } break;
                default:
                    for (map<int, float>::iterator it = motorTorques.begin(); it != motorTorques.end(); ++it) {
                        temp = {0., 0., 0., 0., it->second};
                        action[it->first] = temp;
                    }
                    break;
            }
        }

        std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> actionContactForce(action, contactForces);
        return actionContactForce;
    }
} // namespace Quadruped

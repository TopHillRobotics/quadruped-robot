/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Stance controller for stance foot.
* Author: Zang Yaohua & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zang Yaohua
*/

#include "mpc_controller/torque_stance_leg_controller.h"
#include "mpc_controller/qp_torque_optimizer.h"
using namespace std;
namespace Quadruped {

    TorqueStanceLegController::TorqueStanceLegController(Robot *robot,
                                                         OpenloopGaitGenerator *gaitGenerator,
                                                         RobotEstimator *robotEstimator,
                                                         GroundSurfaceEstimator *groundEstimatorIn,
                                                         ComAdjuster *comAdjuster,
                                                         PosePlanner *posePlanner,
                                                         FootholdPlanner *footholdPlanner,
                                                         Eigen::Matrix<float, 3, 1> desiredSpeed,
                                                         float desiredTwistingSpeed,
                                                         float desiredBodyHeight,
                                                         int numLegs,
                                                         std::string configFilepath,
                                                         std::vector<float> frictionCoeffs)
    {
        this->robot = robot;
        this->gaitGenerator = gaitGenerator;
        this->robotEstimator = robotEstimator;
        groundEstimator = groundEstimatorIn;
        this->comAdjuster = comAdjuster;
        this->posePlanner = posePlanner;
        this->footholdPlanner = footholdPlanner;
        this->configFilepath = configFilepath;
        this->desiredSpeed = desiredSpeed;
        this->desiredTwistingSpeed = desiredTwistingSpeed;
        this->desiredBodyHeight = desiredBodyHeight;
        this->numLegs = numLegs;
        this->frictionCoeffs = frictionCoeffs;

        //configFilepath is "config/stance_leg_controller.yaml".
        // YAML::Node param = YAML::LoadFile(configFilepath);
        // this->force_dim = param["stance_leg_params"]["force_dim"].as<int>();
        // vector<float> v = param["stance_leg_params"]["KD"].as<vector<float>>();
        // this->KD = Eigen::MatrixXf::Map(&v[0], 6, 1);
        // v = param["stance_leg_params"]["KP"].as<vector<float>>();
        // this->KP = Eigen::MatrixXf::Map(&v[0], 6, 1);
        // v = param["stance_leg_params"]["max_ddq"].as<vector<float>>();
        // this->maxDdq = Eigen::MatrixXf::Map(&v[0], 6, 1);
        // v = param["stance_leg_params"]["min_ddq"].as<vector<float>>();
        // this->minDdq = Eigen::MatrixXf::Map(&v[0], 6, 1);
        // v = param["stance_leg_params"]["acc_weight"].as<vector<float>>();
        // this->accWeight = Eigen::MatrixXf::Map(&v[0], 6, 1);
        // currentTime = 0.f;
        Reset(0.f);
    }

    void TorqueStanceLegController::Reset(float currentTime_)
    {   
        string controlModeStr;        
        switch (robot->controlParams["mode"])
        {
            case LocomotionMode::VELOCITY_LOCOMOTION:
                controlModeStr = "velocity";
                break; 
            case LocomotionMode::POSITION_LOCOMOTION:
                controlModeStr = "position";
                break; 
            case LocomotionMode::WALK_LOCOMOTION:
                controlModeStr = "walk";
                break;                                                
            default:
                break;
        }
        YAML::Node param = YAML::LoadFile(configFilepath);
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
        currentTime = currentTime_;
    }

    void TorqueStanceLegController::Update(float currentTime_)
    {
        currentTime = currentTime_;
    }

    void TorqueStanceLegController::UpdateFRatio(Vec4<bool> &contacts, int &N, float &moveBasePhase)
    {
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
                if (gaitGenerator->desiredLegState[legId] == LegState::STANCE) {
                // if (gaitGenerator->legState[legId] == LegState::STANCE || gaitGenerator->legState[legId] == LegState::EARLY_CONTACT) {
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
    
    std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> TorqueStanceLegController::GetAction()
    {
        ++count;
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
        
        Eigen::Matrix<float, 3, 4> contactForces;
        Eigen::Matrix<bool, 4, 1> contacts;
        float moveBasePhase = 1.f;
        int N=0;
        UpdateFRatio(contacts, N, moveBasePhase);
        // std::cout << "fMaxRatio " << fMaxRatio.transpose() << std::endl;
        // std::cout << "fMinRatio " << fMinRatio.transpose() << std::endl;
        Eigen::Matrix<float, 3, 4> footPoseWorld = robot->state.GetFootPositionsInWorldFrame();
        // std::cout << "footPoseWorld = " << footPoseWorld <<std::endl;
        
        Vec6<float> pose;
        Vec6<float> twist; // v, wb
        bool computeForceInWorldFrame = false;       
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
        }
        // pose[0] = max(pose[0], robot->GetBasePosition()[0]);
        Eigen::Matrix<float, 3, 4> com2FootInWorld = footPoseWorld.colwise() - pose.head(3);
        Eigen::Matrix<float, 3, 4> footJointAngles = Eigen::Matrix<float,3,4>::Zero();
        Eigen::Matrix<int, 3, 4> jointIdxs;
        Eigen::Matrix<int, 3, 1> jointIdx;
        Eigen::Matrix<float, 3, 1> jointAngles;
        
        Quat<float> robotComOrientation = robot->GetBaseOrientation();
        Mat3<float> Rb = robotics::math::quaternionToRotationMatrix(robotComOrientation).transpose();
        Quat<float> controlFrameOrientation = groundEstimator->GetControlFrameOrientation();
        Mat3<float> Rc = robotics::math::quaternionToRotationMatrix(controlFrameOrientation).transpose();
        Vec3<float> groundRPY = groundEstimator->GetControlFrameRPY();
        Mat3<float> Rcb = Rc.transpose() * Rb;
        
        /// current robot status  ///
        robotComVelocity = robotEstimator->GetEstimatedVelocity();  // base frame
        robotComRpy = robot->GetBaseRollPitchYaw(); // world frame
        robotComRpyRate = robot->GetBaseRollPitchYawRate();  // base frame
        switch (robot->controlParams["mode"]) {
            case LocomotionMode::VELOCITY_LOCOMOTION: {
                robotComPosition = {0., 0., robot->state.basePosition[2]}; // vel mode in base frame, height is in world frame.
                robotComRpy[2] = 0.f;
                if (groundEstimator->terrain.terrainType>=2) { // not horizontal plane
                    robotComPosition = robotics::math::TransformVecByQuat(robotics::math::quatInverse(controlFrameOrientation), robotComPosition);      
                    robotComPosition[0] = 0.f;
                    robotComPosition[1] = 0.f;
                    robotComVelocity = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComVelocity); // in world frame
                    robotComVelocity = robotics::math::RigidTransform({0,0,0}, controlFrameOrientation, robotComVelocity); // in control frame
                    robotComRpy = robotics::math::rotationMatrixToRPY(Rcb.transpose()); // body orientation in control frame.
                    robotComRpyRate = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComRpyRate); // in world frame
                    robotComRpyRate = robotics::math::RigidTransform({0,0,0}, controlFrameOrientation, robotComRpyRate); // in control frame
                }
                // robotComPosition = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComPosition); // in world frame
                // robotComPosition = robotics::math::RigidTransform({0,0,0}, controlFrameOrientation, robotComPosition); // in control frame
            } break;
            case LocomotionMode::WALK_LOCOMOTION: {
                robotComPosition = robot->GetBasePosition(); // in world frame, walk mode
                robotComVelocity = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComVelocity); // in world frame
                robotComRpyRate = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComRpyRate); // in world frame   
            } break;
            default: {  // pos mode
                robotComPosition = {0., 0., robot->state.basePosition[2]};
            }
            
        }
        // std::cout << "robotComPosition" <<robotComPosition << std::endl;
        // std::cout << "robotHeightInControlFrame" <<robot->heightInControlFrame << std::endl;
        // std::cout << "groundRPY = " << groundRPY.transpose() / 3.1415 *180 <<std::endl;
        // std::cout << "robotComRpy = " << robotComRpy.transpose() / 3.1415*180 << std::endl;
        
        robotQ << robotComPosition, robotComRpy;
        robotDq << robotComVelocity, robotComRpyRate;
                
        /// desired robot status  ///
        
        switch (robot->controlParams["mode"]) {
            case LocomotionMode::VELOCITY_LOCOMOTION: {
                // float pitch = min(groundRPY[1], 0.f);
                // float pitch = groundRPY[1];
                desiredComPosition << 0.f, 0.f, desiredBodyHeight;
                desiredComRpy << -groundRPY[0], 0, -groundRPY[2]; // not control roll/yaw
                desiredComVelocity = {desiredSpeed[0], desiredSpeed[1], 0.f}; // in base/control frame
                // desiredComVelocity = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, desiredComVelocity); // in world frame
                desiredComAngularVelocity = {0.f, 0.f, desiredTwistingSpeed};
        
                /*
                Quat<float> intermediateQuat = robotics::math::rpyToQuat(desiredComRpy);
                com2FootInWorld = footPoseWorld.colwise() - desiredComPosition;
                std::cout << "com2FootInWorld = " <<com2FootInWorld << std::endl;
                for(int i=0; i<4; ++i) {
                    Vec3<float> r = robotics::math::RigidTransform(Vec3<float>(0.,0.,0.), intermediateQuat, Vec3<float>(com2FootInWorld.col(i)));
                    // std::cout << "r = " << r.transpose() << std::endl;
                    robot->ComputeMotorAnglesFromFootLocalPosition(i, r, jointIdx, jointAngles);
                    // std::cout << "angles : " << jointAngles.transpose() <<std::endl;
                    footJointAngles.block<3,1>(0, i) = jointAngles;
                    jointIdxs.block<3,1>(0,i) = jointIdx;
                }
                */                
            } break;
            case LocomotionMode::WALK_LOCOMOTION: {
                // std::cout << "base pose = " << robot->GetBasePosition().transpose() << " " << robot->GetBaseRollPitchYaw().transpose() << endl;
                // std::cout << "plan pose = " << pose.transpose() << std::endl;
                // std::cout << "moveBasePhase = " << moveBasePhase << std::endl;     
                // std::cout << "N="<< N << ", contact status: " << contacts.transpose() << std::endl;
                // std::cout << "lastLegState =  " << gaitGenerator->lastLegState.transpose() <<std::endl;
                // std::cout << "foot  force  =  " << robot->state.footForce.transpose() << std::endl;
                
                for (int i=0; i<NumLeg; ++i) {
                    float phase = gaitGenerator->normalizedPhase[i];
                        
                    if ((contacts[i] && gaitGenerator->desiredLegState[i]==SubLegState::UNLOAD_FORCE && phase > 3.0/4) 
                        || robot->stop) {
                        // printf("leg [%d] ", i);
                        // std::cout << "rcom2f = " << Vec3<float>(com2FootInWorld.col(i)).transpose() << std::endl;
                        // std::cout << "quat = " << posePlanner->quat << std::endl;
                        Quat<float> intermediateQuat = robotics::math::rpyToQuat(Vec3<float>(pose.tail(3)));
                        Vec3<float> dr = Vec3<float>(0.f,0.f,0.01f) / 2000;
                        // Vec3<float> r = robotics::math::RigidTransform(Vec3<float>(0.,0.,0.), robotComOrientation, Vec3<float>(com2FootInWorld.col(i) + dr));
                        Vec3<float> r = robot->state.GetFootPositionsInBaseFrame().col(i) + dr;
                        // std::cout << "r = " << r.transpose() << std::endl; 
                        robot->config->ComputeMotorAnglesFromFootLocalPosition(i, r, jointIdx, jointAngles);
                        // std::cout << "angles : " << jointAngles.transpose() <<std::endl;
                        footJointAngles.block<3,1>(0, i) = jointAngles;
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
                desiredComVelocity = twist.head(3); // world
                desiredComAngularVelocity = twist.tail(3); // world
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
        // std::cout << "desiredQ" << desiredQ.transpose() << std::endl;
        // std::cout << "robotQ" << robotQ.transpose() << std::endl;
        Vec6<float> dq = desiredQ - robotQ;
        desiredDq << desiredComVelocity, desiredComAngularVelocity;
        Vec6<float> ddq = desiredDq - robotDq;
        // std::cout << "desiredDq" << desiredDq.transpose() << std::endl;
        // std::cout << "robotDq" << robotDq.transpose() << std::endl;
        
        if (computeForceInWorldFrame) {
            // case1: R(dR^T-->rpy) ; case2:   (R*dR)^T --> rpy
            Mat3<float> robotR = robotics::math::rpyToRotMat(robotComRpy).transpose();
            Mat3<float> desiredRobotRT = robotics::math::rpyToRotMat(desiredComRpy);
            Mat3<float> dR = desiredRobotRT*robotR;
            dq.tail(3) = robotR * robotics::math::rotationMatrixToRPY(dR);
            // dq.tail(3) = robotics::math::rotationMatrixToRPY(robotR * dR);
            
            // case 1 : R*((/hat(WBdes) - /hat(WBcurr))---> to skewV)
            Mat3<float> RTWdes = robotics::math::vectorToSkewMat(desiredRobotRT *  desiredComAngularVelocity);
            Mat3<float> RTWcur = robotics::math::vectorToSkewMat(robotR.transpose() *  robotComRpyRate);
            Vec3<float> dw = robotR * robotics::math::matToSkewVec(RTWdes - RTWcur);
            ddq.tail(3) =  dw;
            // case 2: do nothing
        } else { // computeForceInControlFrame
            // Mat3<float> dR = robotics::math::rpyToRotMat(desiredComRpy) * robotics::math::quaternionToRotationMatrix(robotComOrientation).transpose();
            // Quat<float> quatBaseInControlFrame = robotics::math::rotationMatrixToQuaternion(dR.transpose());
            // Quat<float> quatBaseInControlFrame = robotics::math::quatProduct(robotics::math::quatInverse(controlFrameOrientation), robotComOrientation);
            // dq.tail(3) = robotics::math::quatToso3(quatBaseInControlFrame);
            // dq.tail(3) = -robotics::math::rotationMatrixToAxisAngle(dR.transpose());
            
            // Mat3<float> robotR = robotics::math::rpyToRotMat(robotComRpy).transpose();
            // Mat3<float> desiredRobotRT = robotics::math::rpyToRotMat(desiredComRpy);
            // Mat3<float> dR = desiredRobotRT*robotR;
            // dq.tail(3) = dR * robotics::math::rotationMatrixToRPY(dR);
            // dq.tail(3) = Rcb * dq.tail(3); // todo, input should not be represent in base frame

            // Mat3<float> RTWdes = robotics::math::vectorToSkewMat(dR.transpose() *  desiredComAngularVelocity);
            // Mat3<float> RTWcur = robotics::math::vectorToSkewMat(dR.transpose() *  robotComRpyRate);
            // Vec3<float> dw = dR * robotics::math::matToSkewVec(RTWdes - RTWcur);
            // ddq.tail(3) =  dw;    
        }   

        //
        desiredDdq = KP.cwiseProduct(dq) + KD.cwiseProduct(ddq);
        desiredDdq = desiredDdq.cwiseMin(maxDdq).cwiseMax(minDdq); // Clip
        // std::cout << "desiredDdq" << desiredDdq.transpose() << std::endl;
         
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
        // std::cout << "fMaxRatio " << fMaxRatio.transpose() << std::endl;
        // std::cout << "contact for force compute " << contacts.transpose() << std::endl;
        
        map<int, MotorCommand> action;
        map<int, float> motorTorques;
        Eigen::Matrix<float, 12, 1> kps = robot->config->motorKps;
        Eigen::Matrix<float, 12, 1> kds = robot->config->motorKds;
        
        // draw figure
        /*
        motorTorques = robot->state.MapContactForceToJointTorques(3, contactForces.col(3));
        datax.push_back(currentTime);
        Vec4<float> forces =  robot->state.footForce;
        // // float theta =  atan(contactForces(1,0) / (contactForces(0,0)+ 1e-5))/3.1415*180;
        // // float yaw = robotComRpy[2]/3.1415*180;
        datay1.push_back(forces[0]);
        datay2.push_back(forces[1]);
        datay3.push_back(forces[2]);
        datay4.push_back(forces[3]);
        if (count > 20000) {
            plt::figure();
            // plt::plot(datax,datay);
            std::map<std::string,std::string> map_;
            map_["cmap"] = "viridis";
            map_["alpha"] = "0.5";
            plt::plot(datax,datay1, {{"label", "cur FL"}});
            plt::plot(datax,datay2, {{"label", "cur RR"}});
            plt::plot(datax,datay3, {{"label", "cur RL"}});
            plt::plot(datax,datay4, {{"label", "cur FR"}});
            std::vector<float> xticks = {2.5f, 5.0f, 7.5f, 10.f};

            plt::xticks(xticks);
            plt::grid(true);
            plt::legend();
            plt::show();
            exit(0);
        }
        */
        
        for (int legId = 0; legId < NumLeg; ++legId) {
            motorTorques = robot->state.MapContactForceToJointTorques(legId, contactForces.col(legId));
            MotorCommand temp;
            switch (robot->controlParams["mode"]) {
                case LocomotionMode::WALK_LOCOMOTION: {
                    for (int motorId=0; motorId<3; motorId++) {    
                        if (contacts[legId]) {
                            temp = {0*footJointAngles(motorId, legId), 0*kps(3*legId+motorId), 0., 0.5*kds(3*legId+motorId), motorTorques[3*legId+motorId]};
                            // if (gaitGenerator->desiredLegState[legId]==SubLegState::UNLOAD_FORCE && gaitGenerator->normalizedPhase[legId] > 3.f/4) {
                            //     std::cout << "legID=" << legId << " should lift the foot a little." << std::endl; 
                            //     temp = {footJointAngles(motorId, legId), kps(3*legId+motorId), 0., kds(3*legId+motorId), 0*motorTorques[3*legId+motorId]};
                            // }
                        } else if ((N<4 && moveBasePhase<0.7) || robot->stop) {
                            temp = {0, 0, 0., kds(3*legId+motorId)*0, motorTorques[3*legId+motorId]};
                        } else { // moveBasePhase > 0.7
                            temp = {0., 0., 0., 0., 0.};
                        }       
                        action[jointIdxs(motorId,legId)] = temp;
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

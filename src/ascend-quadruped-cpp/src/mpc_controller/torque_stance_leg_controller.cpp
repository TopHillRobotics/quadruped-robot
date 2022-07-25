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

    qrStanceLegController::qrStanceLegController(qrRobot *robot,
                                                         qrGaitGenerator *gaitGenerator,
                                                         qrRobotEstimator *robotEstimator,
                                                         qrGroundSurfaceEstimator *groundEstimatorIn,
                                                         qrComPlanner *comPlanner,
                                                         PosePlanner *posePlanner,
                                                         qrFootholdPlanner *footholdPlanner,
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
        this->groundEstimator = groundEstimatorIn;
        this->comPlanner = comPlanner;
        this->posePlanner = posePlanner;
        this->footholdPlanner = footholdPlanner;
        this->configFilepath = configFilepath;
        this->desiredSpeed = desiredSpeed;
        this->desiredTwistingSpeed = desiredTwistingSpeed;
        this->desiredBodyHeight = desiredBodyHeight;
        this->numLegs = numLegs;
        this->frictionCoeffs = frictionCoeffs;
        Reset(0.f);
    }

    void qrStanceLegController::Reset(float currentTime_)
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

    void qrStanceLegController::Update(float currentTime_)
    {
        currentTime = currentTime_;
    }

    void qrStanceLegController::UpdateFRatio(Vec4<bool> &contacts, int &N, float &moveBasePhase)
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
    
    std::tuple<std::map<int, qrMotorCommand>, Eigen::Matrix<float, 3, 4>> qrStanceLegController::GetAction()
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
        
        Eigen::Matrix<float, 3, 4> contactForces;
        Eigen::Matrix<bool, 4, 1> contacts;
        float moveBasePhase = 1.f;
        int N=0;
        UpdateFRatio(contacts, N, moveBasePhase);
        Eigen::Matrix<float, 3, 4> footPoseWorld = robot->state.GetFootPositionsInWorldFrame();
        
        Vec6<float> pose;
        Vec6<float> twist; // v, wb
        bool computeForceInWorldFrame = false;       
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
        // std::cout << "robotComPosition" <<robotComPosition << std::endl;
        // std::cout << "robotHeightInControlFrame" <<robot->heightInControlFrame << std::endl;
        // std::cout << "groundRPY = " << groundRPY.transpose() / 3.1415 *180 <<std::endl;
        // std::cout << "robotComRpy = " << robotComRpy.transpose() / 3.1415*180 << std::endl;
        
        robotQ << robotComPosition, robotComRpy;
        robotDq << robotComVelocity, robotComRpyRate;
                
        /// desired robot status  ///
        desiredComPosition << 0.f, 0.f, desiredBodyHeight;
        desiredComRpy << -groundRPY[0], 0, -groundRPY[2]; // not control roll/yaw
        desiredComVelocity = {desiredSpeed[0], desiredSpeed[1], 0.f}; // in base/control frame
        desiredComAngularVelocity = {0.f, 0.f, desiredTwistingSpeed};
        desiredQ << desiredComPosition, desiredComRpy;
        // std::cout << "desiredQ" << desiredQ.transpose() << std::endl;
        // std::cout << "robotQ" << robotQ.transpose() << std::endl;
        Vec6<float> dq = desiredQ - robotQ;
        desiredDq << desiredComVelocity, desiredComAngularVelocity;
        Vec6<float> ddq = desiredDq - robotDq;
        // std::cout << "desiredDq" << desiredDq.transpose() << std::endl;
        // std::cout << "robotDq" << robotDq.transpose() << std::endl;

        //
        desiredDdq = KP.cwiseProduct(dq) + KD.cwiseProduct(ddq);
        desiredDdq = desiredDdq.cwiseMin(maxDdq).cwiseMax(minDdq); // Clip
        // std::cout << "desiredDdq" << desiredDdq.transpose() << std::endl;
         
        /// Compute Contact Force  ///
        Mat3<float> directionVectors = Mat3<float>::Identity(); // friction cone direction in xyz axises
        // Mat3<float> directionVectors = groundEstimator->GetAlignedDirections();
        // std::cout << "directionVectors = " << directionVectors << std::endl;      
        contactForces << ComputeContactForce(robot, groundEstimator, desiredDdq, contacts, accWeight); // compute the force in control/base frame        
        // std::cout << "contactForces" << contactForces << std::endl;
        // std::cout << "fMaxRatio " << fMaxRatio.transpose() << std::endl;
        // std::cout << "contact for force compute " << contacts.transpose() << std::endl;
        
        map<int, qrMotorCommand> action;
        map<int, float> motorTorques;
        Eigen::Matrix<float, 12, 1> kps = robot->config->motorKps;
        Eigen::Matrix<float, 12, 1> kds = robot->config->motorKds;
        
        for (int legId = 0; legId < NumLeg; ++legId) {
            motorTorques = robot->state.MapContactForceToJointTorques(legId, contactForces.col(legId));
            qrMotorCommand temp;
            for (map<int, float>::iterator it = motorTorques.begin(); it != motorTorques.end(); ++it) {
                    temp = {0., 0., 0., 0., it->second};
                    action[it->first] = temp;
            }
        }

        std::tuple<std::map<int, qrMotorCommand>, Eigen::Matrix<float, 3, 4>> actionContactForce(action, contactForces);
        return actionContactForce;
    }
} // namespace Quadruped

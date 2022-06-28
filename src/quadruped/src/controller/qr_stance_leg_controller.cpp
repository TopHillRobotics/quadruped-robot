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

#include "controller/qr_stance_leg_controller.h"

qrStanceLegController::
    qrSwingLegController(qrRobot *robot,
                         qrGaitGenerator *gaitGenerator,
                         qrRobotVelocityEstimator *robotVelocityEstimator,
                         qrGroundSurfaceEstimator *groundEstimator,
                         qrComPlanner *comPlanner,
                        //  qrPosePlanner *posePlanner,
                         qrFootholdPlanner *footholdPlanner,
                         Eigen::Matrix<float, 3, 1> desired_speed,
                         float desiredTwistingSpeed,
                         float desiredBodyHeight,
                         std::string configFilepath)
    : robot(robot),
      gaitGenerator(gaitGenerator),
      robotVelocityEstimator(robotVelocityEstimator),
      groundEstimator(groundEstimator),
      comPlanner(comPlanner),
    //   posePlanner(posePlanner),
      footholdPlanner(footholdPlanner),
      desiredSpeed(desired_speed),
      desiredTwistingSpeed(desiredTwistingSpeed),
      desiredBodyHeight(desiredBodyHeight),
      configFilepath(configFilepath)
{
    this->Reset(0.f);
    this->robotConfig = this->robot->getRobotConfig();
    this->robotState = this->robot->getRobotState();
}

void qrStanceLegController::Reset(float currentTime)
{
    std::string controlModeStr;
    switch (this->robotConfig->controlMode)
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
    this->currentTime = currentTime;
    // this->resetTime = this->robot->GetTimeSinceReset();
    // this->gaitGenerator->Reset(timeSinceReset);
    // this->stateEstimator->Reset(timeSinceReset);
    // this->groundEstimator->Reset();
    // this->comPlanner->Reset(timeSinceReset);
    // this->posePlanner->Reset(timeSinceReset);
}

void qrStanceLegController::Update(float currentTime)
{
    // if(!this->robot->stop){
    //     this->timeSinceReset = this->robot->GetTimeSinceReset() - this->resetTime;
    // }
    // this->gaitGenerator->Update(this->timeSinceReset);

    // bool switchToSwing = false;
    // if (this->robotConfig->controlMode == LocomotionMode::WALK_LOCOMOTION) {
    //     // for walk mode
    //     const Vec4<int>& newLegState = this->gaitGenerator->legState;
    //     const Vec4<int>& curLegState = this->gaitGenerator->curLegState;
    //     for(int legId =0; legId<4; legId++) {
    //         if((newLegState(legId) == LegState::SWING && curLegState(legId) == LegState::STANCE)
    //             || newLegState(legId) == LegState::USERDEFINED_SWING) {
    //             switchToSwing=true;
    //             break;
    //         }
    //     }
    // }

    // this->groundEstimator->Update();
    // this->stateEstimator->Update(this->timeSinceReset);
    // switch (this->robotConfig->controlMode) {
    //     case LocomotionMode::POSITION_LOCOMOTION: {
    //         this->comPlanner->Update(this->timeSinceReset);
    //     } break; 
    //     case LocomotionMode::WALK_LOCOMOTION: {
    //         if (switchToSwing) {
    //             this->posePlanner->Update(this->timeSinceReset);
    //             printf("update pose plan finish\n");
    //         }
    //     } break;
    //     default: break;
    // }
    this->currentTime = currentTime;
}

void qrStanceLegController::UpdateFRatio(Vec4<bool> &contacts, int &N, float &moveBasePhase)
{
    /// leg contact status  ///
    if (this->robot->stop) {
        printf("robot->stop");
        this->fMaxRatio << 10., 10., 10., 10.;
        this->fMinRatio << 0.01, 0.01, 0.01, 0.01;
        contacts << true, true, true, true;
        N = 4; 
        return;
    } else if (this->robotConfig->controlMode != LocomotionMode::WALK_LOCOMOTION) {
        this->fMaxRatio << 10., 10., 10., 10.;
        this->fMinRatio << 0.01, 0.01, 0.01, 0.01;
        for (int legId=0; legId<4; ++legId){
            if (gaitGenerator->desiredLegState[legId] == LegState::STANCE) {
                contacts[legId] = true;
                N++;
            } else {
                contacts[legId] = false;
            }
        }
        return;
    // } else {
    //     for (int legId = 0; legId < 4; ++legId) {
    //         int desiredLegState = gaitGenerator->desiredLegState[legId];
    //         int detectedLegState = gaitGenerator->detectedLegState[legId];
    //         float phase = gaitGenerator->normalizedPhase[legId];
            
    //         if (detectedLegState == LegState::STANCE || detectedLegState == LegState::LOSE_CONTACT) {
    //             contacts[legId] = true;
    //             N++;
    //             this->fMaxRatio[legId] = 10.0;
    //             this->fMinRatio[legId] = 0.001;
    //         } else if(detectedLegState == LegState::EARLY_CONTACT) { // plan is swing, actual is stand
    //                 contacts[legId] = true;
    //                 N++;
    //                 float tempRatio = abs(phase-0.8);///(1.0-detectedEventTickPhase[legId]);
    //                 this->fMaxRatio[legId] = 10.0 * std::min(0.01f, tempRatio);
    //                 this->fMinRatio[legId] = 0.001;
    //         } else { // Swing STATE in plan
    //             moveBasePhase = gaitGenerator->moveBasePhase;
    //             if (desiredLegState==SubLegState::LOAD_FORCE) {
    //                 contacts[legId] = true;
    //                 N++;
    //                 this->fMaxRatio[legId] = 10.0 * std::max(0.001f, phase);
    //                 this->fMinRatio[legId] = 0.001;
    //             } else if (desiredLegState==SubLegState::UNLOAD_FORCE){
    //                 contacts[legId] = true;
    //                 N++;
    //                 phase = phase / (3.f/4.0); // todo
    //                 this->fMaxRatio[legId] = 10.0 * std::max(0.001, 1.0-phase);
    //                 this->fMinRatio[legId] = 0.001;
    //             } else if (desiredLegState==SubLegState::TRUE_SWING) {
    //                 contacts[legId] = false;
    //                 this->fMaxRatio[legId] = 0.002;
    //                 this->fMinRatio[legId] = 0.001;
    //             }  else if (desiredLegState==SubLegState::FULL_STANCE) {
    //                 contacts[legId] = true;
    //                 N++;
    //                 this->fMaxRatio[legId] = 10.0;
    //                 this->fMinRatio[legId] = 0.001;
    //             } else {
    //                 throw std::invalid_argument("no this leg state");
    //             }
    //         }
    //     }
    // }
    }
}

std::tuple<std::map<int, qrMotorCmd>, Eigen::Matrix<float, 3, 4>> qrStanceLegController::GetAction()
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
    // Eigen::Matrix<float, 3, 4> footPoseWorld = robot->GetFootPositionsInWorldFrame();
    
    Vec6<float> pose;
    Vec6<float> twist; // v, wb
    bool computeForceInWorldFrame = false;       
    // if (this->robotConfig->controlMode == LocomotionMode::WALK_LOCOMOTION) {
    //     computeForceInWorldFrame = true;
    //     if (!robot->stop) {
    //         auto res = posePlanner->GetIntermediateBasePose(moveBasePhase, currentTime); //todo  in world frame;
    //         pose = std::get<0>(res);
    //         twist = std::get<1>(res);
    //     } else {
    //         auto res = posePlanner->GetIntermediateBasePose(currentTime);
    //         pose = std::get<0>(res);
    //         twist = std::get<1>(res);  
    //     }
    // }
    // Eigen::Matrix<float, 3, 4> com2FootInWorld = footPoseWorld.colwise() - pose.head(3);
    Eigen::Matrix<float, 3, 4> footJointAngles = Eigen::Matrix<float,3,4>::Zero();
    Eigen::Matrix<int, 3, 4> jointIdxs;
    Eigen::Matrix<int, 3, 1> jointIdx;
    Eigen::Matrix<float, 3, 1> jointAngles;
    
    Quat<float> robotComOrientation = this->robotConfig->GetBaseOrientation();
    Mat3<float> Rb = math::quaternionToRotationMatrix(robotComOrientation).transpose();
    Quat<float> controlFrameOrientation = this->groundEstimator->GetControlFrameOrientation();
    Mat3<float> Rc = math::quaternionToRotationMatrix(controlFrameOrientation).transpose();
    Vec3<float> groundRPY = this->groundEstimator->GetControlFrameRPY();
    Mat3<float> Rcb = Rc.transpose() * Rb;
    
    /// current robot status  ///
    robotComVelocity = this->robotEstimator->GetEstimatedVelocity();  // base frame
    robotComRpy = this->robotState->GetRpy(); // world frame
    robotComRpyRate = this->robotState->GetDrpy();  // base frame
    // switch (this->robotConfig->controlMode) {
    //     case LocomotionMode::VELOCITY_LOCOMOTION: {
            robotComPosition = {0., 0., this->robotConfig->GetBasePosition()[2]}; // vel mode in base frame, height is in world frame.
            robotComRpy[2] = 0.f;
            if (groundEstimator->terrain.terrainType>=2) { // not horizontal plane
                robotComPosition = math::TransformVecByQuat(robotics::math::quatInverse(controlFrameOrientation), robotComPosition);      
                robotComPosition[0] = 0.f;
                robotComPosition[1] = 0.f;
                robotComVelocity = math::invertRigidTransform({0,0,0}, robotComOrientation, robotComVelocity); // in world frame
                robotComVelocity = math::RigidTransform({0,0,0}, controlFrameOrientation, robotComVelocity); // in control frame
                robotComRpy = math::rotationMatrixToRPY(Rcb.transpose()); // body orientation in control frame.
                robotComRpyRate = math::invertRigidTransform({0,0,0}, robotComOrientation, robotComRpyRate); // in world frame
                robotComRpyRate = math::RigidTransform({0,0,0}, controlFrameOrientation, robotComRpyRate); // in control frame
            }
    //     } break;
    //     case LocomotionMode::WALK_LOCOMOTION: {
    //         robotComPosition = robotConfig->GetBasePosition(); // in world frame, walk mode
    //         robotComVelocity = math::invertRigidTransform({0,0,0}, robotComOrientation, robotComVelocity); // in world frame
    //         robotComRpyRate = math::invertRigidTransform({0,0,0}, robotComOrientation, robotComRpyRate); // in world frame   
    //     } break;
    //     default: {  // pos mode
    //         robotComPosition = {0., 0., robotConfig->GetBasePosition()[2]};
    //     }
        
    // }
    
    robotQ << robotComPosition, robotComRpy;
    robotDq << robotComVelocity, robotComRpyRate;
            
    /// desired robot status  ///
    
    // switch (this->robotConfig->controlMode) {
    //     case LocomotionMode::VELOCITY_LOCOMOTION: {
            desiredComPosition << 0.f, 0.f, desiredBodyHeight;
            desiredComRpy << -groundRPY[0], 0, -groundRPY[2]; // not control roll/yaw
            desiredComVelocity = {this->desiredSpeed[0], this->desiredSpeed[1], 0.f}; // in base/control frame
            desiredComAngularVelocity = {0.f, 0.f, desiredTwistingSpeed};        
    //     } break;
    //     case LocomotionMode::WALK_LOCOMOTION: {
    //         for (int i=0; i<NumLeg; ++i) {
    //             float phase = gaitGenerator->normalizedPhase[i];
                    
    //             if ((contacts[i] && gaitGenerator->desiredLegState[i]==SubLegState::UNLOAD_FORCE && phase > 3.0/4) 
    //                 || robot->stop) {
    //                 Quat<float> intermediateQuat = math::rpyToQuat(Vec3<float>(pose.tail(3)));
    //                 Vec3<float> dr = Vec3<float>(0.f,0.f,0.01f) / 2000;
    //                 Vec3<float> r = this->robotState->GetFootPositionsInBaseFrame().col(i) + dr;
    //                 robotConfig->FootPosition2JointAngles(i, r, jointIdx, jointAngles);
    //                 footJointAngles.block<3,1>(0, i) = jointAngles;
    //                 jointIdxs.block<3,1>(0,i) = jointIdx;
    //             } else {
    //                 for(int j=0;j<3;j++) {
    //                     jointIdxs(j,i) = 3*i+j;
    //                 }
    //             }
    //         }
    //         // pose in world frame
    //         desiredComPosition = pose.head(3);  // world frame
    //         desiredComRpy = pose.tail(3); // world frame
    //         desiredComRpy[1] = std::max(-0.35f, std::min(desiredComRpy[1], 0.35f));
    //         desiredComVelocity = twist.head(3); // world
    //         desiredComAngularVelocity = twist.tail(3); // world
    //     } break;
    //     case LocomotionMode::POSITION_LOCOMOTION: {
    //         auto &comAdjPosInBaseFrame = comPlanner->GetComPosInBaseFrame();
    //         desiredComPosition = {comAdjPosInBaseFrame[0], comAdjPosInBaseFrame[1],
    //                             desiredBodyHeight}; // get goal com position from comPlanner, base frame
    //         desiredComVelocity = {this->desiredSpeed[0], this->desiredSpeed[1], 0.f};
    //         // get goal rpy from footholdPlanner, in world frame
    //         desiredComRpy = footholdPlanner->GetDesiredComPose().tail(3);
    //         desiredComAngularVelocity = {0.f, 0.f, 0.f};
    //     } break;
    // }
    desiredQ << desiredComPosition, desiredComRpy;
    Vec6<float> dq = desiredQ - robotQ;
    desiredDq << desiredComVelocity, desiredComAngularVelocity;
    Vec6<float> ddq = desiredDq - robotDq;
    
    if (computeForceInWorldFrame) {
        // case1: R(dR^T-->rpy) ; case2:   (R*dR)^T --> rpy
        Mat3<float> robotR = math::rpyToRotMat(robotComRpy).transpose();
        Mat3<float> desiredRobotRT = math::rpyToRotMat(desiredComRpy);
        Mat3<float> dR = desiredRobotRT*robotR;
        dq.tail(3) = robotR * math::rotationMatrixToRPY(dR);
        
        // case 1 : R*((/hat(WBdes) - /hat(WBcurr))---> to skewV)
        Mat3<float> RTWdes = math::vectorToSkewMat(desiredRobotRT *  desiredComAngularVelocity);
        Mat3<float> RTWcur = math::vectorToSkewMat(robotR.transpose() *  robotComRpyRate);
        Vec3<float> dw = robotR * math::matToSkewVec(RTWdes - RTWcur);
        ddq.tail(3) =  dw;
        // case 2: do nothing
    } else { // computeForceInControlFrame
        ;
    }   

    //
    desiredDdq = this->KP.cwiseProduct(dq) + this->KD.cwiseProduct(ddq);
    desiredDdq = desiredDdq.cwiseMin(maxDdq).cwiseMax(minDdq); // Clip
        
    /// Compute Contact Force  ///
    Mat3<float> directionVectors = Mat3<float>::Identity(); // friction cone direction in xyz axises
    if (computeForceInWorldFrame) {
        contactForces << ComputeContactForce(robot, desiredDdq,  // all in world frame
                                            contacts, accWeight,
                                            directionVectors.col(2), 
                                            directionVectors.col(0),
                                            directionVectors.col(1),
                                            fMinRatio,
                                            fMaxRatio);
    } else {
        contactForces << ComputeContactForce(this->robot, 
                                             this->groundEstimator, 
                                             desiredDdq, 
                                             contacts, 
                                             accWeight); // compute the force in control/base frame        
    }
    
    std::map<int, qrMotorCmd> action;
    std::map<int, float> motorTorques;
    Eigen::Matrix<float, 12, 1> kps = this->robotConfig->GetKps();
    Eigen::Matrix<float, 12, 1> kds = this->robotConfig->GetKds();
    
    for (int legId = 0; legId < this->robotConfig->numLegs; ++legId) {
        motorTorques[legId] = this->robotState->ContactForce2JointTorque(contactForces.col(legId), legId);
        qrMotorCmd temp;
        // switch (this->robotConfig->controlMode) {
        //     case LocomotionMode::WALK_LOCOMOTION: {
        //         for (int motorId=0; motorId<3; motorId++) {    
        //             if (contacts[legId]) {
        //                 temp = {0*footJointAngles(motorId, legId), 0*kps(3*legId+motorId), 0., 0.5*kds(3*legId+motorId), motorTorques[3*legId+motorId]};
        //             } else if ((N<4 && moveBasePhase<0.7) || robot->stop) {
        //                 temp = {0, 0, 0., kds(3*legId+motorId)*0, motorTorques[3*legId+motorId]};
        //             } else { // moveBasePhase > 0.7
        //                 temp = {0., 0., 0., 0., 0.};
        //             }       
        //             action[jointIdxs(motorId,legId)] = temp;
        //         }   
        //     } break;
        //     default:
                for (std::map<int, float>::iterator it = motorTorques.begin(); it != motorTorques.end(); ++it) {
                    temp.SetCmd(0.f, 0.f, 0.f, 0.f, it->second)
                    action[it->first] = temp;
                }
        //         break;
        // }
    }

    std::tuple<std::map<int, qrMotorCmd>, Eigen::Matrix<float, 3, 4>> actionContactForce(action, contactForces);
    return actionContactForce;
}

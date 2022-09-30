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

#include "controller/qr_torque_stance_leg_controller.h"
#include "controller/qr_qp_torque_optimizer.h"
#include "controller/mpc/qr_mit_mpc_stance_leg_controller.h"

using namespace std;

extern std::unordered_map<int, std::string> modeMap;

qrStanceLegController::qrStanceLegController(qrRobot *robot,
                                            qrGaitGenerator *gaitGenerator,
                                            qrRobotEstimator *robotEstimator,
                                            qrGroundSurfaceEstimator *groundEstimatorIn,
                                            qrComPlanner *comPlanner,
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
    this->footholdPlanner = footholdPlanner;
    this->configFilepath = configFilepath;
    this->desiredSpeed = desiredSpeed;
    this->desiredTwistingSpeed = desiredTwistingSpeed;
    this->desiredBodyHeight = desiredBodyHeight;
    this->numLegs = numLegs;
    this->frictionCoeffs = frictionCoeffs;
  Reset(0.f);
}

qrStanceLegController *qrStanceLegController::createStanceController(qrRobot *robot,
                                                                     qrGaitGenerator *gaitGenerator,
                                                                     qrRobotEstimator *robotEstimator,
                                                                     qrGroundSurfaceEstimator *groundEstimatorIn,
                                                                     qrComPlanner *comPlanner,
                                                                     qrFootholdPlanner *footholdPlanner,
                                                                     Eigen::Matrix<float, 3, 1> desiredSpeed,
                                                                     float desiredTwistingSpeed,
                                                                     float desiredBodyHeight,
                                                                     int numLegs,
                                                                     std::string configFilepath,
                                                                     std::vector<float> frictionCoeffs,
                                                                     bool useMPC)
{
    if(!useMPC){
        return new qrStanceLegController(robot, gaitGenerator, robotEstimator, groundEstimatorIn,
                                         comPlanner, footholdPlanner, desiredSpeed, desiredTwistingSpeed,
                                         desiredBodyHeight, numLegs, configFilepath, frictionCoeffs);
    }
    else {
        return new qrMITConvexMPCStanceLegController(robot, gaitGenerator, robotEstimator, groundEstimatorIn,
                                                     comPlanner, footholdPlanner, desiredSpeed, desiredTwistingSpeed,
                                                     desiredBodyHeight, numLegs, configFilepath, frictionCoeffs);
    }
}

void qrStanceLegController::Reset(float currentTime_)
{   
    string controlModeStr;
    if (modeMap.count(robot->locomotionMode) > 0) {
        controlModeStr = modeMap[robot->locomotionMode];
    }
    std::cout << "locomotion mode: " + controlModeStr << std::endl;
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
    // leg contact status
    if (robot->stop) {
        printf("robot->stop");
        fMaxRatio << 10., 10., 10., 10.;
        fMinRatio << 0.01, 0.01, 0.01, 0.01;
        contacts << true, true, true, true;
        N = 4; 
        return;
    } else {
        fMaxRatio << 10., 10., 10., 10.;
        fMinRatio << 0.01, 0.01, 0.01, 0.01;
        for (int legId=0; legId<NumLeg; ++legId) {
            if (gaitGenerator->desiredLegState[legId] == LegState::STANCE) {
                contacts[legId] = true;
                N++;
            } else {
                contacts[legId] = false;
            }
        }
        return;
    } 
}

void qrStanceLegController::VelocityLocomotionProcess(Quat<float> &robotComOrientation,
                                                        Eigen::Matrix<float, 3, 1> &robotComPosition, 
                                                        Eigen::Matrix<float, 3, 1> &robotComVelocity,
                                                        Eigen::Matrix<float, 3, 1> &robotComRpy,
                                                        Eigen::Matrix<float, 3, 1> &robotComRpyRate,
                                                        Eigen::Matrix<float, 3, 1> &desiredComPosition,
                                                        Eigen::Matrix<float, 3, 1> &desiredComVelocity,
                                                        Eigen::Matrix<float, 3, 1> &desiredComRpy,
                                                        Eigen::Matrix<float, 3, 1> &desiredComAngularVelocity)
{
    Mat3<float> Rb = math::quaternionToRotationMatrix(robotComOrientation).transpose();
    Quat<float> controlFrameOrientation = groundEstimator->GetControlFrameOrientation();
    Mat3<float> Rc = math::quaternionToRotationMatrix(controlFrameOrientation).transpose();
    Vec3<float> groundRPY = groundEstimator->GetControlFrameRPY();
    Mat3<float> Rcb = Rc.transpose() * Rb;

    // velocity mode in base frame, height is in world frame.
    robotComPosition = {0., 0., robot->state.basePosition[2]}; 
    robotComRpy[2] = 0.f;
    // if not horizontal plane then adjust the orientation.
    if (groundEstimator->terrain.terrainType != TerrainType::PLANE) { 
        robotComPosition = math::TransformVecByQuat(math::quatInverse(controlFrameOrientation), robotComPosition);      
        robotComPosition[0] = 0.f;
        robotComPosition[1] = 0.f;
        // COM velocity in world frame
        robotComVelocity = math::invertRigidTransform({0,0,0}, robotComOrientation, robotComVelocity); 
        // COM velocity in control frame
        robotComVelocity = math::RigidTransform({0,0,0}, controlFrameOrientation, robotComVelocity); 
        // Body orientation in control frame.
        robotComRpy = math::rotationMatrixToRPY(Rcb.transpose());
        // In world frame
        robotComRpyRate = math::invertRigidTransform({0,0,0}, robotComOrientation, robotComRpyRate); 
        // In control frame
        robotComRpyRate = math::RigidTransform({0,0,0}, controlFrameOrientation, robotComRpyRate); 
    }
    desiredComPosition << 0.f, 0.f, desiredBodyHeight;
    // do not control roll/yaw
    desiredComRpy << -groundRPY[0], 0, -groundRPY[2]; 
    // desired COM velocity in base/control frame
    desiredComVelocity = {desiredSpeed[0], desiredSpeed[1], 0.f}; 
    desiredComAngularVelocity = {0.f, 0.f, desiredTwistingSpeed};
}

void qrStanceLegController::PositionLocomotionProcess(Eigen::Matrix<float, 3, 1> &robotComPosition,
                                                        Eigen::Matrix<float, 3, 1> &desiredComPosition,
                                                        Eigen::Matrix<float, 3, 1> &desiredComVelocity,
                                                        Eigen::Matrix<float, 3, 1> &desiredComRpy,
                                                        Eigen::Matrix<float, 3, 1> &desiredComAngularVelocity)
{
    robotComPosition = {0., 0., robot->state.basePosition[2]};

    auto &comAdjPosInBaseFrame = comPlanner->GetComPosInBaseFrame();
    // get goal com position from comPlanner in base frame
    desiredComPosition = {comAdjPosInBaseFrame[0], comAdjPosInBaseFrame[1], desiredBodyHeight}; 
    desiredComVelocity = {desiredSpeed[0], desiredSpeed[1], 0.f};
    // get goal rpy from footholdPlanner in world frame
    desiredComRpy = footholdPlanner->GetDesiredComPose().tail(3);
    desiredComAngularVelocity = {0.f, 0.f, 0.f};
}

// See the MIT paper for details:https://ieeexplore.ieee.org/document/8593885
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
    Vec6<float> twist;
    bool computeForceInWorldFrame = false;       
    Eigen::Matrix<float, 3, 4> com2FootInWorld = footPoseWorld.colwise() - pose.head(3);
    Eigen::Matrix<float, 3, 4> footJointAngles = Eigen::Matrix<float,3,4>::Zero();
    Eigen::Matrix<int, 3, 4> jointIdxs;
    Eigen::Matrix<int, 3, 1> jointIdx;
    Eigen::Matrix<float, 3, 1> jointAngles;
    
    Quat<float> robotComOrientation = robot->GetBaseOrientation();
    
    /// current robot status  ///
    // robot COM velocity in base frame
    robotComVelocity = robotEstimator->GetEstimatedVelocity();
    // robot COM rpy in world frame
    robotComRpy = robot->GetBaseRollPitchYaw(); 
    // robot COM rpy rate in base frame
    robotComRpyRate = robot->GetBaseRollPitchYawRate();  
    switch(robot->locomotionMode){
        case LocomotionMode::VELOCITY_LOCOMOTION: {
            VelocityLocomotionProcess(robotComOrientation,
                                        robotComPosition,
                                        robotComVelocity,
                                        robotComRpy,
                                        robotComRpyRate,
                                        desiredComPosition,
                                        desiredComVelocity,
                                        desiredComRpy,
                                        desiredComAngularVelocity);
        }
            break;
        case LocomotionMode::POSITION_LOCOMOTION: {
            PositionLocomotionProcess(robotComPosition,
                                        desiredComPosition,
                                        desiredComVelocity,
                                        desiredComRpy,
                                        desiredComAngularVelocity);
        }
            break;
    }

    robotQ << robotComPosition, robotComRpy;
    robotDq << robotComVelocity, robotComRpyRate;
    desiredQ << desiredComPosition, desiredComRpy;
    Vec6<float> dq = desiredQ - robotQ;
    desiredDq << desiredComVelocity, desiredComAngularVelocity;
    Vec6<float> ddq = desiredDq - robotDq;

    desiredDdq = KP.cwiseProduct(dq) + KD.cwiseProduct(ddq);
    desiredDdq = desiredDdq.cwiseMin(maxDdq).cwiseMax(minDdq);

    /// Compute Contact Force ///
    // friction cone direction in xyz axises
    Mat3<float> directionVectors = Mat3<float>::Identity(); 
    // compute the force in control/base frame
    contactForces << ComputeContactForce(robot, groundEstimator, desiredDdq, contacts, accWeight);
    
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


void qrStanceLegController::UpdateControlParameters(const Eigen::Vector3f& linSpeed, const float& angSpeed)
{
    desiredSpeed = linSpeed;
    desiredTwistingSpeed = angSpeed;
}

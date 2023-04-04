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

#include "controllers/balance_controller/qr_torque_stance_leg_controller.h"
#include "controllers/balance_controller/qr_qp_torque_optimizer.h"
using namespace std;

namespace Quadruped {

TorqueStanceLegController::TorqueStanceLegController(qrRobot *robot,
                                                     qrGaitGenerator *gaitGenerator,
                                                     qrStateEstimatorContainer* stateEstimators,
                                                     qrComAdjuster *comAdjuster,
                                                     qrPosePlanner *posePlanner,
                                                     qrFootholdPlanner *footholdPlanner,
                                                     qrUserParameters& userParameters,
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
    N = 0;

    /* Setup contact state and max and min F ratio in different robot state and locomotion mode. */
    if (robot->stop) {
        /* If robot is stopped, just set some normal values to fmax and fmin. */
        printf("[stance leg controller] Robot stopped.\n");
        fMaxRatio << 10., 10., 10., 10.;
        fMinRatio << 0.01, 0.01, 0.01, 0.01;
        contacts << true, true, true, true;
        N = 4;
        return;
    } else if (robot->controlParams["mode"] != LocomotionMode::WALK_LOCOMOTION) {
        /* The fMaxRatio and fMinRatio will be used to formulate the contraint matrix.
         * This will be set again in %ComputeConstraintMatrix matrix again.
         */
        fMaxRatio << 10., 10., 10., 10.;
        fMinRatio << 0.01, 0.01, 0.01, 0.01;
        for (int legId = 0; legId < NumLeg; ++legId){
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
                fMaxRatio[legId] = 10.0f;
                fMinRatio[legId] = 0.001f;
            } else if(detectedLegState == LegState::EARLY_CONTACT) { // plan is swing, actual is stand
                /* Plan is is swing, but actual is standing. */
                contacts[legId] = true;
                N++;
                float tempRatio = abs(phase - 0.8f);
                fMaxRatio[legId] = 10.0f * std::min(0.01f, tempRatio);
                fMinRatio[legId] = 0.001f;
            } else {
                /* Plan to Swing State. The state from stance to swing will need to unload force and vice versa in walk locomotion. */
                moveBasePhase = gaitGenerator->moveBasePhase;
                if (desiredLegState == SubLegState::LOAD_FORCE) {
                    contacts[legId] = true;
                    N++;
                    fMaxRatio[legId] = 10.0f * std::max(0.001f, phase);
                    fMinRatio[legId] = 0.001f;
                } else if (desiredLegState==SubLegState::UNLOAD_FORCE){
                    contacts[legId] = true;
                    N++;
                    phase = phase / (3.f/4.0f); // todo
                    fMaxRatio[legId] = 10.0f * std::max(0.001f, 1.0f - phase);
                    fMinRatio[legId] = 0.001f;
                } else if (desiredLegState==SubLegState::TRUE_SWING) {
                    contacts[legId] = false;
                    fMaxRatio[legId] = 0.002f;
                    fMinRatio[legId] = 0.001f;
                }  else if (desiredLegState==SubLegState::FULL_STANCE) {
                    contacts[legId] = true;
                    N++;
                    fMaxRatio[legId] = 10.0;
                    fMinRatio[legId] = 0.001f;
                } else {
                    throw std::invalid_argument("[stance leg controller] This leg state does not exist.");
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

    Vec6<float> pose, twist; /* Desired pose and twist. */

    if (robot->controlParams["mode"]==LocomotionMode::WALK_LOCOMOTION) {

        computeForceInWorldFrame = true;
        if (!robot->stop) {
            auto res = posePlanner->GetIntermediateBasePose(moveBasePhase, currentTime); //todo  in world frame;
            pose = std::get<0>(res);
            twist = std::get<1>(res);
        } else {
            auto res = posePlanner->GetIntermediateBasePose(currentTime);
            pose = std::get<0>(res);
            twist = std::get<1>(res);
        }
    } else {
        pose.head(3) = robot->GetBasePosition();
    }

    Eigen::Matrix<int, 3, 4> jointIdxs;
    Eigen::Matrix<int, 3, 1> jointIdx;
    Eigen::Matrix<float, 3, 1> jointAngles;
    Eigen::Matrix<float, 3, 4> com2FootInWorld = footPoseWorld.colwise() - pose.head(3);

    /* Calculate the rotation matrix from base frame to control frame. */
    Quat<float> robotComOrientation = robot->GetBaseOrientation();
    Mat3<float> Rb = robot->stateDataFlow.baseRMat;
    Quat<float> controlFrameOrientation = groundEstimator->GetControlFrameOrientation();
    Mat3<float> Rc = robot->stateDataFlow.groundRMat;
    Vec3<float> groundRPY = groundEstimator->GetControlFrameRPY();
    Mat3<float> Rcb = robot->stateDataFlow.baseRInControlFrame;

    /* Current robot status on CoM. */
    robotComRpy = robot->GetBaseRollPitchYaw(); /* robotComRpy is in world frame. */
    robotComVelocity = robotEstimator->GetEstimatedVelocity(); /* robotComVelocity is in base frame */
    robotComRpyRate = robot->GetBaseRollPitchYawRate(); /* robotComRpyRate is in base frame */

    /* Setup current robot states according to locomotion mode. */
    switch (robot->controlParams["mode"]) {
    case LocomotionMode::VELOCITY_LOCOMOTION:
        /* Be carefure that VELOCITY_LOCOMOTION in base frame, but robot height is in world frame. */
        computeForceInWorldFrame = false;

        robotComPosition = {0.0f, 0.0f, robot->basePosition[2]};

        if (groundEstimator->terrain.terrainType >= 2) { /* The terrain is not a horizontal plane. */

            /* Transform the CoM position into control frame. */
            robotComPosition = robotics::math::TransformVecByQuat(robotics::math::quatInverse(controlFrameOrientation), robotComPosition);
            robotComPosition[0] = 0.0f;
            robotComPosition[1] = 0.0f;

            /* Transform the CoM velocity into control frame.
             * First transform into world, then into control frame.
             */
            robotComVelocity = robotics::math::invertRigidTransform({0, 0, 0}, robotComOrientation, robotComVelocity);
            robotComVelocity = robotics::math::RigidTransform({0, 0, 0}, controlFrameOrientation, robotComVelocity);

            /* Transform the CoM roll, pitch, yaw and their change rate into control frame.
             * First transform into world, then into control frame.
             */
            robotComRpy = robotics::math::rotationMatrixToRPY(Rcb.transpose());
            robotComRpyRate = robotics::math::invertRigidTransform({0, 0, 0}, robotComOrientation, robotComRpyRate);
            robotComRpyRate = robotics::math::RigidTransform({0, 0, 0}, controlFrameOrientation, robotComRpyRate);
        } else {
            /* The only difference is that the CoM position in world frame is as same as that in control frame. */
            robotComVelocity = robotics::math::invertRigidTransform({0, 0, 0}, robotComOrientation, robotComVelocity);
            robotComRpyRate = robotics::math::invertRigidTransform({0, 0, 0}, robotComOrientation, robotComRpyRate);
            if (!computeForceInWorldFrame) { // conrtrol frame
                robotComRpy[2] = 0.0f;
                robotComVelocity = robotics::math::RigidTransform({0, 0, 0}, controlFrameOrientation, robotComVelocity);
                robotComRpyRate = robotics::math::RigidTransform({0, 0, 0}, controlFrameOrientation, robotComRpyRate);
            }
        }
        break;

    case LocomotionMode::ADVANCED_TROT:
        if (computeForceInWorldFrame) {
            /* Recommand to set computeForceInWorldFrame to true. */
            /* Transform all CoM states into world frame. */
            robotComPosition = {0., 0., robot->basePosition[2]};
            robotComVelocity = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComVelocity);
            robotComRpyRate = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComRpyRate);
        } else {
            /* Transform all CoM states into control frame. */
            robotComPosition = {0., 0., robot->basePosition[2]};
            robotComRpy[2] = 0.f;
            if (groundEstimator->terrain.terrainType>=2) {
                robotComPosition = robotics::math::TransformVecByQuat(robotics::math::quatInverse(controlFrameOrientation), robotComPosition);
                robotComPosition[0] = 0.f;
                robotComPosition[1] = 0.f;
                robotComPosition = {0., 0., robot->stateDataFlow.heightInControlFrame};

                robotComVelocity = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComVelocity);
                robotComVelocity = robotics::math::RigidTransform({0,0,0}, controlFrameOrientation, robotComVelocity);

                robotComRpy = robotics::math::rotationMatrixToRPY(Rcb.transpose());
                robotComRpyRate = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComRpyRate);
                robotComRpyRate = robotics::math::RigidTransform({0,0,0}, controlFrameOrientation, robotComRpyRate);
            }
        }
        break;

    case LocomotionMode::WALK_LOCOMOTION:
        /* Transform all CoM states into world frame. */
        robotComPosition = robot->GetBasePosition();
        robotComVelocity = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComVelocity); // in world frame
        robotComRpyRate = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComRpyRate); // in world frame
        break;

    case LocomotionMode::POSITION_LOCOMOTION:
        /* Transform CoM position into base frame.
         * POSITION_LOCOMOTION does not need other transformations.
         * Be careful that setting robotComPosition[2] to a value in world frame is just for calculation
         */
        computeForceInWorldFrame = false;
        robotComPosition = {0., 0., robot->basePosition[2]};
        break;

    default:
        throw std::domain_error("no such LocomotionMode");
    }

    /* Setup pose and twist vector. */
    robotQ << robotComPosition, robotComRpy;
    robotDq << robotComVelocity, robotComRpyRate;

    /* Setup desired robot pose, twist according to different mode. */
    switch (robot->controlParams["mode"]) {
    case LocomotionMode::VELOCITY_LOCOMOTION:

        desiredComPosition << 0.f, 0.f, desiredBodyHeight;

        /* If quadruped has a roll or yaw, then make it back to zero position.
         * Because VELOCITY_LOCOMOTION uses force balance, we just use it tortting on the ground.
         * So the pitch is set to zero.
         */
        desiredComRpy << -groundRPY[0], 0, -groundRPY[2];

        /* Get desired twist in base frame. */
        desiredComVelocity = desiredStateCommand->stateDes.segment(6,3);
        desiredComAngularVelocity = desiredStateCommand->stateDes.segment(9,3);

        if (computeForceInWorldFrame) {
            /* Transform into world frame. */
            desiredComRpy = robotComRpy + robot->timeStep * desiredComAngularVelocity;
            desiredComRpy[0] = 0;
            desiredComVelocity = robotics::math::invertRigidTransform({0, 0, 0}, robotComOrientation, desiredComVelocity); // in world frame
        }
        break;

    case LocomotionMode::ADVANCED_TROT:
        if (computeForceInWorldFrame) {
            /* MPC uses comAdjuster to get desired CoM position and transform it into world frame. */
            auto &comAdjPosInBaseFrame = comAdjuster->GetComPosInBaseFrame();
            Vec3<float> newComPosInWorldFrame = Rb * comAdjPosInBaseFrame + robot->basePosition;
            desiredComPosition = {newComPosInWorldFrame[0], newComPosInWorldFrame[1], desiredStateCommand->stateDes(2)};

            /* Clip the pitch value. */
            float pitch = groundRPY[1];
            float pitchMax = 0.5f;
            if (abs(pitch) < 0.1f) {
                pitch = 0;
            } else if (pitch > pitchMax) {
                pitch = pitchMax;
            } else if (pitch < -pitchMax) {
                pitch = -pitchMax;
            }

            /* MPC controls pitch to go onto the stair. Just set others to zero. */
            desiredComRpy << 0.f, pitch, 0.f;

            Eigen::Matrix<float, 3, 4> footPosInBaseFrame = robot->GetFootPositionsInBaseFrame();
            float scaleFactor = 1;

            /* If the former two legs are close to the body, the velocity should be slow down
             * by multiplying a factor related to the foot position. It's experiential.
             * Users can adjust this part.
             */
            float footX = std::min(footPosInBaseFrame(0, 0), footPosInBaseFrame(0, 1));
            if (footX < 0.1f)
                scaleFactor = std::max(0.1f, footX / 0.1f);

            /* The linear and angular velocity are actually in base frame.
             * But this does not has any effects if using same frame in KP/KD calculation.
             */
            desiredComVelocity = (scaleFactor * desiredStateCommand->stateDes.segment(6, 3));

            if (pitch < 0.1f && desiredComVelocity[2] > 0.01f) {
                /* When the quadruped has a velocity on Z axis, the height part should add a compensation
                 * because stateDes(2) is a constant. Will adjusting this in the future.
                 */
                desiredComPosition[2] += 0.04f * abs(pitch/pitchMax);
            }
            desiredComAngularVelocity = desiredStateCommand->stateDes.segment(9, 3);
        } else {
            /* Transform related CoM states into control frame. */
            desiredComPosition << 0.f, 0.f, desiredBodyHeight * std::abs(std::cos(groundRPY[1]));

            /* CoM orientation shouldn't change too quickly, so using a linear filter. */
            desiredComPosition[2] = robotComPosition[2] * 0.7 + desiredComPosition[2] * 0.3;
            desiredComRpy << -groundRPY[0], 0.f, -groundRPY[2];
            desiredComVelocity = {desiredSpeed[0], desiredSpeed[1], 0.f};
            desiredComVelocity = Rc * desiredComVelocity;
            desiredComAngularVelocity = {0.f, 0.f, desiredTwistingSpeed};
        }
        break;

    case LocomotionMode::WALK_LOCOMOTION:
        for (int i = 0; i < NumLeg; ++i) {

            float phase = gaitGenerator->normalizedPhase[i];

            if ((contacts[i] && gaitGenerator->desiredLegState[i] == SubLegState::UNLOAD_FORCE && phase > 3.0f / 4)
                || robot->stop) {
                Quat<float> intermediateQuat = robotics::math::rpyToQuat(Vec3<float>(pose.tail(3)));
                Vec3<float> dr = Vec3<float>(0.f, 0.f, 0.01f) / 2000;
                Vec3<float> r = robot->GetFootPositionsInBaseFrame().col(i) + dr;
                robot->ComputeMotorAnglesFromFootLocalPosition(i, r, jointIdx, jointAngles);
                desiredStateCommand->legJointq.block<3, 1>(0, i) = jointAngles;
                jointIdxs.block<3, 1>(0, i) = jointIdx;
            } else {
                for(int j = 0; j < 3; j++) {
                    jointIdxs(j, i) = 3 * i + j;
                }
            }
        }

        /* Poses are in world frame. */
        desiredComPosition = pose.head(3);
        desiredComRpy = pose.tail(3);

        /* Clip the pitch. */
        desiredComRpy[1] = std::max(-0.35f, std::min(desiredComRpy[1], 0.35f));

        /* Twists are in world frame. */
        desiredComVelocity = twist.head(3);
        desiredComAngularVelocity = twist.tail(3);

        break;

    case LocomotionMode::POSITION_LOCOMOTION:
        /* POSITION_LOCOMOTION uses comAdjuster to get desired CoM position in base frame. */
        auto &comAdjPosInBaseFrame = comAdjuster->GetComPosInBaseFrame();

        desiredComPosition = {comAdjPosInBaseFrame[0], comAdjPosInBaseFrame[1], desiredBodyHeight};
        desiredComVelocity = {desiredSpeed[0], desiredSpeed[1], 0.f};

        /* All sets to zero. Will consider to adjust footholdPlanner for better results.
         * Can set roll pitch yaw and angular velocity if needed.
         */
        desiredComRpy = footholdPlanner->GetDesiredComPose().tail(3);
        desiredComAngularVelocity = {0.f, 0.f, 0.f};
        break;
    }

    /* Calculate change on position and velocity. */
    desiredQ << desiredComPosition, desiredComRpy;
    Vec6<float> dq = desiredQ - robotQ;
    desiredDq << desiredComVelocity, desiredComAngularVelocity;
    Vec6<float> ddq = desiredDq - robotDq;

    if (computeForceInWorldFrame && robot->controlParams["mode"] != LocomotionMode::ADVANCED_TROT) {
        /* Another method to calculate change on roll pitch yaw: R(dR^T-->rpy). */
        Mat3<float> robotR = robotics::math::rpyToRotMat(robotComRpy).transpose();
        Mat3<float> desiredRobotRT = robotics::math::rpyToRotMat(desiredComRpy);
        Mat3<float> dR = desiredRobotRT*robotR;
        dq.tail(3) = robotR * robotics::math::rotationMatrixToRPY(dR);

        /* Another method to calculate change on angular velocity : R*((/hat(WBdes) - /hat(WBcurr))---> to skewV). */
        Mat3<float> RTWdes = robotics::math::vectorToSkewMat(desiredRobotRT *  desiredComAngularVelocity);
        Mat3<float> RTWcur = robotics::math::vectorToSkewMat(robotR.transpose() *  robotComRpyRate);
        Vec3<float> dw = robotR * robotics::math::matToSkewVec(RTWdes - RTWcur);
        ddq.tail(3) =  dw;
    }

    /* Using KP/KD to calculate desired acceleration and clip. */
    desiredDdq = KP.cwiseProduct(dq) + KD.cwiseProduct(ddq);
    desiredDdq = desiredDdq.cwiseMin(maxDdq).cwiseMax(minDdq);

    desiredStateCommand->stateCur << robotQ, robotDq;
    desiredStateCommand->stateDes << desiredQ, desiredDq;
    desiredStateCommand->ddqDes.head(6) = desiredDdq;
}
    

std::tuple<std::map<int, qrMotorCommand>, Eigen::Matrix<float, 3, 4>> TorqueStanceLegController::GetAction()
{
    ++count;

    /* Update desired linear and angular velocity and acceleration. */
    UpdateDesCommand();

    Vec6<float> desiredDdq = desiredStateCommand->ddqDes.head(6);

    Eigen::Matrix<float, 3, 4> contactForces;

    /* Compute contact force. */
    Mat3<float> directionVectors = Mat3<float>::Identity();

    /* Position and Velocity locomotion will not compute force in world frame. */
    if (computeForceInWorldFrame) {
        contactForces << ComputeContactForce(robot, desiredDdq, contacts, accWeight,
                                             directionVectors.col(2), directionVectors.col(0), directionVectors.col(1),
                                             fMinRatio, fMaxRatio);
    } else {
        contactForces << ComputeContactForce(robot, groundEstimator, desiredDdq, contacts, accWeight);
    }

    std::map<int, qrMotorCommand> action;
    std::map<int, float> motorTorques;
    Eigen::Matrix<float, 12, 1> kps = robot->GetMotorKps();
    Eigen::Matrix<float, 12, 1> kds = robot->GetMotorKdp();

    for (int legId = 0; legId < NumLeg; ++legId) {
        motorTorques = robot->MapContactForceToJointTorques(legId, contactForces.col(legId));
        qrMotorCommand temp;
        switch (robot->controlParams["mode"]) {
        case LocomotionMode::WALK_LOCOMOTION:
            /* When stance or need unloading force but base is still moving, keep exerting torque to the motor.
             * Currently set the threshold to 0.7.
             * Else the leg will unload the force.
             */
            for (int motorId = 0; motorId < 3; motorId++) {
                if (contacts[legId]) {
                    temp = { 0.0 * desiredStateCommand->legJointq(motorId, legId),
                             0.0 * kps(3 * legId + motorId),
                             0.0,
                             0.5 * kds(3 * legId + motorId),
                             motorTorques[3 * legId + motorId]};
                } else if ((N < 4 && moveBasePhase < 0.7) || robot->stop) {
                    temp = {0, 0, 0., kds(3 * legId + motorId) * 0.0, motorTorques[3 * legId + motorId]};
                } else {
                    /* When moveBasePhase > 0.7 */
                    temp = {0., 0., 0., 0., 0.};
                }
                action[motorId + 3 * legId] = temp;
            }
            break;
        default:
            /* Trotting with force balance. */
            for (map<int, float>::iterator it = motorTorques.begin(); it != motorTorques.end(); ++it) {
                temp = {0., 0., 0., 0., it->second};
                action[it->first] = temp;
            }
            break;
        }
    }

    std::tuple<std::map<int, qrMotorCommand>, Eigen::Matrix<float, 3, 4>> actionContactForce(action, contactForces);
    return actionContactForce;
}

} // namespace Quadruped

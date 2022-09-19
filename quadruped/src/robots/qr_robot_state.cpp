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

#include "robots/qr_robot_state.h"

static bool firstObservation = true;

qrRobotState::qrRobotState()
{

    baseOrientation << 1.f, 0.f, 0.f, 0.f;
    baseRollPitchYaw << 0.f, 0.f, 0.f;
    baseRollPitchYawRate << 0.f, 0.f, 0.f;
    motorVelocities = Eigen::Matrix<float, 12, 1>::Zero();
    footForce << 0.f, 0.f, 0.f, 0.f;
    footContact << 1, 1, 1, 1;
    baseVelocity << 0.0, 0.0, 0.0;
}

qrRobotState::qrRobotState(qrRobotConfig* config): qrRobotState()
{
    SetRobotConfig(config);
}

void qrRobotState::Update()
{
    if(firstObservation){
        yawOffset = imu.rpy[2]; // when firstly Updating, record current yaw offset
        firstObservation = false;
    }
    baseRollPitchYaw << imu.rpy[0], imu.rpy[1], CalibrateYaw();
    baseOrientation = math::rpyToQuat(baseRollPitchYaw);
    baseRollPitchYawRate << imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2];
    for (int motorId = 0; motorId < qrRobotConfig::numMotors; motorId++) {
        motorAngles[motorId] = motorState[motorId].q;
        motorVelocities[motorId] = motorState[motorId].dq;
    }
    for (int footId = 0; footId < qrRobotConfig::numLegs; footId++) {
        footContact[footId] = footForce[footId] > 5 ? true : false;
    }
}

void qrRobotState::SetRobotConfig(qrRobotConfig *config)
{
    this->config = config;
}


float qrRobotState::CalibrateYaw()
{
    float calibratedYaw = imu.rpy[2] - yawOffset;
    if (calibratedYaw >= M_PI) {
        calibratedYaw -= (2 * M_PI);
    } else if (calibratedYaw <= -M_PI) {
        calibratedYaw += (2 * M_PI);
    }
    return calibratedYaw;
}

Eigen::Matrix<float, 3, 4> qrRobotState::GetFootPositionsInBaseFrame()
{
    return config->FootPositionsInBaseFrame(this->motorAngles);
}

Eigen::Matrix<float, 3, 4> qrRobotState::GetFootPositionsInWorldFrame(bool useInput, Vec3<float> basePositionIn, Quat<float> baseOrientationIn)
{
    Eigen::Matrix<float, 3, 4> footPositionsInBaseFrame = GetFootPositionsInBaseFrame(); // base to  world frame
    if (!useInput) {
        return math::invertRigidTransform(basePosition, baseOrientation, footPositionsInBaseFrame);
    } else {
        return math::invertRigidTransform(basePositionIn, baseOrientationIn, footPositionsInBaseFrame);
    }
}

Eigen::Matrix<float, 3, 3> qrRobotState::ComputeJacobian(int legId)
{
    Eigen::Matrix<float, 3, 1> legMotorAngles;
    legMotorAngles << this->motorAngles.block(legId * 3, 0, 3, 1);
    return config->AnalyticalLegJacobian(legMotorAngles, legId);
}

std::map<int, float> qrRobotState::MapContactForceToJointTorques(int legId, Eigen::Matrix<float, 3, 1> contractForce)
{
    Eigen::Matrix<float, 3, 3> jv = ComputeJacobian(legId);
    Eigen::Matrix<float, 3, 1> motorTorquesPerLeg = jv.transpose() * contractForce; // TODO TEST
    std::map<int, float> motorTorquesDict;
    for (int torqueIndex = 0; torqueIndex < motorTorquesPerLeg.size(); torqueIndex++) {
        int jointIndex = torqueIndex + legId * qrRobotConfig::dofPerLeg;
        motorTorquesDict[jointIndex] = motorTorquesPerLeg[torqueIndex];
    }
    return motorTorquesDict;
}

#include "robots/qr_robot_state.h"

static bool firstObservation = true;

RobotState::RobotState(qrRobotConfig*& config):config(config)
{
    Eigen::Matrix<float, 3, 1> basePosition = {0.f, 0.f, config->bodyHeight};
    baseOrientation << 1.f, 0.f, 0.f, 0.f;
    baseRollPitchYaw << 0.f, 0.f, 0.f;
    baseRollPitchYawRate << 0.f, 0.f, 0.f;
    motorVelocities = Eigen::Matrix<float, 12, 1>::Zero();
    footForce << 0.f, 0.f, 0.f, 0.f;
    footContact << 1, 1, 1, 1;
    std::cout <<" robot state initialized" << std::endl;
}

void RobotState::Update()
{
    if(firstObservation){
        yawOffset = imu.rpy[2]; // todo
        firstObservation = false;
    }
    baseRollPitchYaw << imu.rpy[0], imu.rpy[1], CalibrateYaw();
    baseOrientation = robotics::math::rpyToQuat(baseRollPitchYaw);
    baseRollPitchYawRate << imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2];
    for (int motorId = 0; motorId < qrRobotConfig::numMotors; motorId++) {
        motorAngles[motorId] = motorState[motorId].q;
        motorVelocities[motorId] = motorState[motorId].dq;
    }
    for (int footId = 0; footId < qrRobotConfig::numLegs; footId++) {
        footContact[footId] = footForce[footId] > 5 ? true : false;
    }
}


float RobotState::CalibrateYaw()
{
    float calibratedYaw = imu.rpy[2] - yawOffset;
    if (calibratedYaw >= M_PI) {
        calibratedYaw -= (2 * M_PI);
    } else if (calibratedYaw <= -M_PI) {
        calibratedYaw += (2 * M_PI);
    }
    return calibratedYaw;
}

Eigen::Matrix<float, 3, 4> RobotState::GetFootPositionsInBaseFrame()
{
    return config->FootPositionsInBaseFrame(this->motorAngles);
}

Eigen::Matrix<float, 3, 4> RobotState::GetFootPositionsInWorldFrame(bool useInput, Vec3<float> basePositionIn, Quat<float> baseOrientationIn)
{
    Eigen::Matrix<float, 3, 4> footPositionsInBaseFrame = GetFootPositionsInBaseFrame(); // base to  world frame
    if (!useInput) {
        return robotics::math::invertRigidTransform(basePosition, baseOrientation, footPositionsInBaseFrame);
    } else {
        return robotics::math::invertRigidTransform(basePositionIn, baseOrientationIn, footPositionsInBaseFrame);
    }
}

Eigen::Matrix<float, 3, 3> RobotState::ComputeJacobian(int legId)
{
    Eigen::Matrix<float, 3, 1> legMotorAngles;
    legMotorAngles << this->motorAngles.block(legId * 3, 0, 3, 1);
    return config->AnalyticalLegJacobian(legMotorAngles, legId);
}

std::map<int, float> RobotState::MapContactForceToJointTorques(int legId, Eigen::Matrix<float, 3, 1> contractForce)
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

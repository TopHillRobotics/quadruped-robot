#include "robot/qr_robot_state.h"
#include "common/qr_se3.h"

qrRobotState::qrRobotState(){
    deltaTime = 0.001f;
}

qrRobotState::qrRobotState(qrRobotConfig *robotConfig)
{
    deltaTime = 0.001f;
    this->robotConfig = robotConfig;
    basePosition = {0.f, 0.f, robotConfig->bodyHeight};
}

void qrRobotState::setTimeStamp(uint32_t tick){
    if(lastStamp != 0){
        deltaTime = (tick - lastStamp) / 1000.0f;
    }
    lastStamp = tick;
}

void qrRobotState::SetRobotConfig(qrRobotConfig *robotConfig)
{
    this->robotConfig = robotConfig;
}

Vec3<float> qrRobotState::GetDq(unsigned int legId){
    Vec3<float> result;
    result << dq[legId * 3], dq[legId * 3 + 1], dq[legId * 3 + 2];
    return result;
}

Vec3<float> qrRobotState::GetQ(unsigned int legId){
    Vec3<float> result;
    result << q[legId * 3], q[legId * 3 + 1], q[legId * 3 + 2];
    return result;
}

Mat3x4<float> qrRobotState::GetFootPositionInBaseFrame()
{
    return robotConfig->JointAngles2FootPositionInBaseFrame(this->q);
}

Vec3<float> qrRobotState::GetRpy()
{
    return this->imu.CalibratedYawRpy();
}

Vec3<float> qrRobotState::GetDrpy()
{
    return this->imu.gyroscope;
}

Vec4<float> qrRobotState::GetBaseOrientation()
{
    return math::Rpy2Quat(this->imu.CalibratedYawRpy());
}

Mat3<float> qrRobotState::GetJacobian(int legId)
{
    Vec3<float> legQ;
    legQ << q.segment(legId * 3, 3);
    return robotConfig->AnalyticalLegJacobian(legQ, legId);
}


Vec3<float> qrRobotState::ContactForce2JointTorque(Vec3<float> contractForce, int legId)
{
    Mat3<float> j = GetJacobian(legId);
    return j.transpose() * contractForce;
}

void qrRobotState::operator=(const qrRobotState &robotState)
{
    this->imu       = robotState.imu;
    this->footForce = robotState.footForce;
    this->q         = robotState.q;
    this->dq        = robotState.dq;
    this->tau       = robotState.tau;
}

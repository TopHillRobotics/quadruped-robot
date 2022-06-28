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

Eigen::Matrix<float, 3, 1> qrRobotState::GetDq(unsigned int legId){
  Eigen::Matrix<float, 3, 1> result;
  result << dq[legId * 3], dq[legId * 3 + 1], dq[legId * 3 + 2];
  return result;
}

Eigen::Matrix<float, 3, 1> qrRobotState::GetQ(unsigned int legId){
  Eigen::Matrix<float, 3, 1> result;
  result << q[legId * 3], q[legId * 3 + 1], q[legId * 3 + 2];
  return result;
}

Eigen::Matrix<float, 3, 4> qrRobotState::GetFootPositionInBaseFrame()
{
  return robotConfig->JointAngles2FootPositionInBaseFrame(this->q);
}

Eigen::Matrix<float, 3, 1> qrRobotState::GetRpy()
{
  return this->imu.CalibratedYawRpy();
}

Eigen::Matrix<float, 3, 1> qrRobotState::GetDrpy()
{
  return this->imu.gyroscope;
}

Eigen::Matrix<float, 4, 1> qrRobotState::GetBaseOrientation()
{
  return math::Rpy2Quat(this->imu.CalibratedYawRpy());
}

Eigen::Matrix<float, 3, 3> qrRobotState::GetJacobian(int legId)
{
  Eigen::Matrix<float, 3, 1> legQ;
  legQ << q.segment(legId * 3, 3);
  return robotConfig->AnalyticalLegJacobian(legQ, legId);
}


Eigen::Matrix<float, 3, 1> qrRobotState::ContactForce2JointTorque(Eigen::Matrix<float, 3, 1> contractForce, int legId)
{
  Eigen::Matrix<float, 3, 3> j = GetJacobian(legId);
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

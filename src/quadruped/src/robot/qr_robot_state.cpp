#include "robot/qr_robot_state.h"

qrRobotState::qrRobotState(){
  deltaTime = 0.001f;
}

void qrRobotState::setTimeStamp(uint32_t tick){
  if(lastStamp != 0){
    deltaTime = (tick - lastStamp) / 1000.0f;
  }
  lastStamp = tick;
}

Eigen::Matrix<float, 3, 1> qrRobotState::getDq(unsigned int legId){
  Eigen::Matrix<float, 3, 1> result;
  result << dq[legId * 3], dq[legId * 3 + 1], dq[legId * 3 + 2];
  return result;
}

Eigen::Matrix<float, 3, 1> qrRobotState::getQ(unsigned int legId){
  Eigen::Matrix<float, 3, 1> result;
  result << q[legId * 3], q[legId * 3 + 1], q[legId * 3 + 2];
  return result;
}

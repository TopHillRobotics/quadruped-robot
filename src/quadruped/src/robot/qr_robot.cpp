#include "robot/qr_robot.h"
#include "common/qr_se3.h"

qrRobot::qrRobot()
{
  stop = false;
  config = nullptr;
  robotState = new qrRobotState();
}

qrRobot::qrRobot(std::string path)
{
  stop = false;
  config = new qrRobotConfig(path);
  robotState = new qrRobotState();
}

void qrRobot::LoadConfig(std::string path)
{
  if(config == nullptr)
    config = new qrRobotConfig(path);
  else {
    config->Load(path);
  }
}

qrRobot::~qrRobot()
{
  delete config;
}

std::array<qrMotorCmd, 12> qrRobot::GetCmd()
{
  return cmds;
}

void qrRobot::SetCmd(const Eigen::MatrixXf &motorCmdValues, MotorMode mode)
{
  switch(mode){
  case POSITION:
    SetAngleCmd(motorCmdValues);
    break;
  case TORQUE:
    SetTorqueCmd(motorCmdValues);
    break;
  case HYBRID:
    SetHybridCmd(motorCmdValues);
    break;
  }
}

void qrRobot::SetAngleCmd(const Eigen::Matrix<float, 12, 1> &qValues)
{
  for (unsigned int i = 0; i < qrRobotConfig::numMotor; i++) {
    cmds[i].SetCmd(qValues[i], config->motorKps[i], 0.0f, config->motorKds[i], 0);
  }
}

void qrRobot::SetTorqueCmd(const Eigen::Matrix<float, 12, 1> &tauValues)
{
  for (unsigned int i = 0; i < qrRobotConfig::numMotor; i++) {
    cmds[i].SetCmd(0.0f, 0.0f, 0.0f, 0.0f, tauValues[i]);
  }
}

void qrRobot::SetHybridCmd(const Eigen::Matrix<float, 5, 12> &cmdValues)
{
  for (unsigned int i = 0; i < qrRobotConfig::numMotor; i++) {
    cmds[i].SetCmd(cmdValues(0, i), cmdValues(1, i), cmdValues(2, i), cmdValues(3, i), cmdValues(4, i));
  }
}



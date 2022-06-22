#include "robot/qr_robot.h"
#include "common/qr_se3.h"

qrRobot::qrRobot()
{
  config = nullptr;
}

qrRobot::qrRobot(std::string path)
{
  config = new qrRobotConfig(path);
}

void qrRobot::loadConfig(std::string path)
{
  if(config == nullptr)
    config = new qrRobotConfig(path);
  else {
    config->load(path);
  }
}

qrRobot::~qrRobot()
{
  delete config;
}

void qrRobot::update()
{
  rpy         = robotState.imu.calibratedYawRpy();
  orientation = Math::rpy2Quat(rpy);
  drpy        = robotState.imu.acc;
  for(unsigned int i = 0; i < qrRobotConfig::numMotor; i++){
    motorq[i]  = robotState.motors[i].q;
    motordq[i] = robotState.motors[i].dq;
  }

  for(unsigned i = 0; i < qrRobotConfig::numLegs; i++){
    if(robotState.footForce[i] > 5.0f){
      footContact[i] = true;
    } else {
      footContact[i] = false;
    }
  }
}

std::array<qrMotorCmd, 12> qrRobot::getCmd()
{
  return cmds;
}

void qrRobot::setCmd(const Eigen::MatrixXf &motorCmdValues, MotorMode mode)
{
  switch(mode){
  case POSITION:
    setAngleCmd(motorCmdValues);
    break;
  case TORQUE:
    setTorqueCmd(motorCmdValues);
    break;
  case HYBRID:
    setHybridCmd(motorCmdValues);
    break;
  }
}

void qrRobot::setAngleCmd(const Eigen::Matrix<float, 12, 1> &qValues)
{
  for (unsigned int i = 0; i < qrRobotConfig::numMotor; i++) {
    cmds[i].SetCmd(qValues[i], config->motorKps[i], 0.0f, config->motorKds[i], 0);
  }
}

void qrRobot::setTorqueCmd(const Eigen::Matrix<float, 12, 1> &tauValues)
{
  for (unsigned int i = 0; i < qrRobotConfig::numMotor; i++) {
    cmds[i].SetCmd(0.0f, 0.0f, 0.0f, 0.0f, tauValues[i]);
  }
}

void qrRobot::setHybridCmd(const Eigen::Matrix<float, 5, 12> &cmdValues)
{
  for (unsigned int i = 0; i < qrRobotConfig::numMotor; i++) {
    cmds[i].SetCmd(cmdValues(0, i), cmdValues(1, i), cmdValues(2, i), cmdValues(3, i), cmdValues(4, i));
  }
}



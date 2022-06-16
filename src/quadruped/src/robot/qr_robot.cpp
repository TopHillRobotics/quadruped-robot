#include "robot/qr_robot.h"
#include "common/se3.h"

qrRobot::qrRobot()
{
  config = nullptr;
  cmds   = nullptr;
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
  delete cmds;
}

void qrRobot::update()
{
  // TODO: fill this parent function
  rpy = robotState.imu.calibratedYawRpy();
}

std::vector<qrMotorCmd> qrRobot::getCmd()
{
  return *cmds;
}

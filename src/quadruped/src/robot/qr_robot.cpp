#include "robot/qr_robot.h"
#include "common/qr_se3.h"

qrRobot::qrRobot()
{
    stop = false;
    timeStep = 0.001f;
    robotConfig = nullptr;
}

qrRobot::qrRobot(std::string path)
{
    stop = false;
    timeStep = 0.001f;
    robotConfig = new qrRobotConfig(path);
    robotState.SetRobotConfig(robotConfig);
}

void qrRobot::LoadConfig(std::string path)
{
    if(robotConfig == nullptr)
      robotConfig = new qrRobotConfig(path);
    else {
      robotConfig->Load(path);
    }
}

qrRobot::~qrRobot()
{
    delete robotConfig;
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

void qrRobot::SetCmd(const std::array<qrMotorCmd, 12> cmd)
{
    for(unsigned int i = 0; i < 12; i++){
        this->cmds[i] = cmd[i];
    }
}

void qrRobot::SetAngleCmd(const Vec12<float> &qValues)
{
    for (unsigned int i = 0; i < qrRobotConfig::numMotor; i++) {
        cmds[i].SetCmd(qValues[i], robotConfig->motorKps[i], 0.0f, robotConfig->motorKds[i], 0);
    }
}

void qrRobot::SetTorqueCmd(const Vec12<float> &tauValues)
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

void qrRobot::ApplyAction(const std::array<qrMotorCmd, 12> cmd)
{
  SetCmd(cmd);
  SendCmd();
}

void qrRobot::ApplyAction(const Eigen::MatrixXf &motorCmdValues, MotorMode mode)
{
  SetCmd(motorCmdValues, mode);
  SendCmd();
}



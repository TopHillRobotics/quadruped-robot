#include "robot/qr_robot_qrRobotA1Sim.h"
#include "common/qr_se3.h"

qrRobotqrRobotA1Sim::qrRobotqrRobotA1Sim(ros::NodeHandle &nhIn, std::string configFilePath):qrRobot(configFilePath)
{
  imuSub = nh.subscribe("/trunk_imu", 1, &qrRobotA1Sim::ImuCallback, this);
  jointStateSub[0]  = nh.subscribe("a1_gazebo/FR_hip_controller/state", 1, &qrRobotA1Sim::FRhipCallback, this);
  jointStateSub[1]  = nh.subscribe("a1_gazebo/FR_thigh_controller/state", 1, &qrRobotA1Sim::FRthighCallback, this);
  jointStateSub[2]  = nh.subscribe("a1_gazebo/FR_calf_controller/state", 1, &qrRobotA1Sim::FRcalfCallback, this);
  jointStateSub[3]  = nh.subscribe("a1_gazebo/FL_hip_controller/state", 1, &qrRobotA1Sim::FLhipCallback, this);
  jointStateSub[4]  = nh.subscribe("a1_gazebo/FL_thigh_controller/state", 1, &qrRobotA1Sim::FLthighCallback, this);
  jointStateSub[5]  = nh.subscribe("a1_gazebo/FL_calf_controller/state", 1, &qrRobotA1Sim::FLcalfCallback, this);
  jointStateSub[6]  = nh.subscribe("a1_gazebo/RR_hip_controller/state", 1, &qrRobotA1Sim::RRhipCallback, this);
  jointStateSub[7]  = nh.subscribe("a1_gazebo/RR_thigh_controller/state", 1, &qrRobotA1Sim::RRthighCallback, this);
  jointStateSub[8]  = nh.subscribe("a1_gazebo/RR_calf_controller/state", 1, &qrRobotA1Sim::RRcalfCallback, this);
  jointStateSub[9]  = nh.subscribe("a1_gazebo/RL_hip_controller/state", 1, &qrRobotA1Sim::RLhipCallback, this);
  jointStateSub[10] = nh.subscribe("a1_gazebo/RL_thigh_controller/state", 1, &qrRobotA1Sim::RLthighCallback, this);
  jointStateSub[11] = nh.subscribe("a1_gazebo/RL_calf_controller/state", 1, &qrRobotA1Sim::RLcalfCallback, this);
  footForceSub[0]   = nh.subscribe("/visual/FR_foot_contact/the_force", 1, &qrRobotA1Sim::FRfootCallback, this);
  footForceSub[1]   = nh.subscribe("/visual/FL_foot_contact/the_force", 1, &qrRobotA1Sim::FLfootCallback, this);
  footForceSub[2]   = nh.subscribe("/visual/RR_foot_contact/the_force", 1, &qrRobotA1Sim::RRfootCallback, this);
  footForceSub[3]   = nh.subscribe("/visual/RL_foot_contact/the_force", 1, &qrRobotA1Sim::RLfootCallback, this);

  jointCmdPub[0]  = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/FR_hip_controller/command", 1);
  jointCmdPub[1]  = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/FR_thigh_controller/command", 1);
  jointCmdPub[2]  = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/FR_calf_controller/command", 1);
  jointCmdPub[3]  = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/FL_hip_controller/command", 1);
  jointCmdPub[4]  = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/FL_thigh_controller/command", 1);
  jointCmdPub[5]  = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/FL_calf_controller/command", 1);
  jointCmdPub[6]  = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/RR_hip_controller/command", 1);
  jointCmdPub[7]  = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/RR_thigh_controller/command", 1);
  jointCmdPub[8]  = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/RR_calf_controller/command", 1);
  jointCmdPub[9]  = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/RL_hip_controller/command", 1);
  jointCmdPub[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/RL_thigh_controller/command", 1);
  jointCmdPub[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/RL_calf_controller/command", 1);

  usleep(300000); // must wait 300ms, to get first state

  robotConfig->yawOffset = robotStateBuffer.imu.rpy[2]; // todo

  this->ResetTimer();

  std::cout << "-------A1Sim init Complete-------" << std::endl;
}

void qrRobotA1Sim::ImuCallback(const sensor_msgs::Imu &msg)
{
  qrRobotStateBuffer.imu.quaternion[0] = msg.orientation.w;
  qrRobotStateBuffer.imu.quaternion[1] = msg.orientation.x;
  qrRobotStateBuffer.imu.quaternion[2] = msg.orientation.y;
  qrRobotStateBuffer.imu.quaternion[3] = msg.orientation.z;

  Eigen::Matrix<float, 4, 1> quaternion = {qrRobotStateBuffer.imu.quaternion[0],
                                           qrRobotStateBuffer.imu.quaternion[1],
                                           qrRobotStateBuffer.imu.quaternion[2],
                                           qrRobotStateBuffer.imu.quaternion[3]};

  Eigen::Matrix<float, 3, 1> rpy = robotics::math::quatToRPY(quaternion);

  qrRobotStateBuffer.imu.rpy[0] = rpy[0];
  qrRobotStateBuffer.imu.rpy[1] = rpy[1];
  qrRobotStateBuffer.imu.rpy[2] = rpy[2];

  qrRobotStateBuffer.imu.gyroscope[0] = msg.angular_velocity.x;
  qrRobotStateBuffer.imu.gyroscope[1] = msg.angular_velocity.y;
  qrRobotStateBuffer.imu.gyroscope[2] = msg.angular_velocity.z;

  qrRobotStateBuffer.imu.acc[0] = msg.linear_acceleration.x;
  qrRobotStateBuffer.imu.acc[1] = msg.linear_acceleration.y;
  qrRobotStateBuffer.imu.acc[2] = msg.linear_acceleration.z;
}

void qrRobotA1Sim::FRhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    qrRobotStateBuffer.motorState[0].q = msg.q;
    qrRobotStateBuffer.motorState[0].dq = msg.dq;
}

void qrRobotA1Sim::FRthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    qrRobotStateBuffer.motorState[1].q = msg.q;
    qrRobotStateBuffer.motorState[1].dq = msg.dq;
}

void qrRobotA1Sim::FRcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    qrRobotStateBuffer.motorState[2].q = msg.q;
    qrRobotStateBuffer.motorState[2].dq = msg.dq;
}

void qrRobotA1Sim::FLhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    qrRobotStateBuffer.motorState[3].q = msg.q;
    qrRobotStateBuffer.motorState[3].dq = msg.dq;
}

void qrRobotA1Sim::FLthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    qrRobotStateBuffer.motorState[4].q = msg.q;
    qrRobotStateBuffer.motorState[4].dq = msg.dq;
}

void qrRobotA1Sim::FLcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    qrRobotStateBuffer.motorState[5].q = msg.q;
    qrRobotStateBuffer.motorState[5].dq = msg.dq;
}

void qrRobotA1Sim::RRhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    qrRobotStateBuffer.motorState[6].q = msg.q;
    qrRobotStateBuffer.motorState[6].dq = msg.dq;
}

void qrRobotA1Sim::RRthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    qrRobotStateBuffer.motorState[7].q = msg.q;
    qrRobotStateBuffer.motorState[7].dq = msg.dq;
}

void qrRobotA1Sim::RRcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    qrRobotStateBuffer.motorState[8].q = msg.q;
    qrRobotStateBuffer.motorState[8].dq = msg.dq;
}

void qrRobotA1Sim::RLhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    qrRobotStateBuffer.motorState[9].q = msg.q;
    qrRobotStateBuffer.motorState[9].dq = msg.dq;
}

void qrRobotA1Sim::RLthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    qrRobotStateBuffer.motorState[10].q = msg.q;
    qrRobotStateBuffer.motorState[10].dq = msg.dq;
}

void qrRobotA1Sim::RLcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    qrRobotStateBuffer.motorState[11].q = msg.q;
    qrRobotStateBuffer.motorState[11].dq = msg.dq;
}

void qrRobotA1Sim::FRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    qrRobotStateBuffer.footForce[0] = msg.wrench.force.z;
}

void qrRobotA1Sim::FLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    qrRobotStateBuffer.footForce[1] = msg.wrench.force.z;
}

void qrRobotA1Sim::RRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    qrRobotStateBuffer.footForce[2] = msg.wrench.force.z;
}

void qrRobotA1Sim::RLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    qrRobotStateBuffer.footForce[3] = msg.wrench.force.z;
}

void qrRobotA1Sim::Observation()
{
  this->robotState = robotStateBuffer;
}

void qrRobotA1Sim::SendCmd()
{
  for(int motorId = 0; motorId < 12; motorId++) {
    std::array<float, 5> cmd = cmds[motorId].ToArray();
    lowCmd.motorCmd[motorId].mode = 0x0A;
    lowCmd.motorCmd[motorId].q    = cmd[0];
    lowCmd.motorCmd[motorId].Kp   = cmd[1];
    lowCmd.motorCmd[motorId].dq   = cmd[2];
    lowCmd.motorCmd[motorId].Kd   = cmd[3];
    lowCmd.motorCmd[motorId].tau  = cmd[4];
  }
  for(int m = 0; m < 12; m++){
    jointCmdPub[m].publish(lowCmd.motorCmd[m]);
  }
}

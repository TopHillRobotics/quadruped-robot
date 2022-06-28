#include "robot/qr_robot_a1sim.h"
#include "common/qr_se3.h"

qrRobotA1Sim::qrRobotA1Sim(ros::NodeHandle &nhIn, std::string configFilePath):qrRobot(configFilePath), nh(nhIn)
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

  robotConfig->SetYawOffset(robotStateBuffer.imu.rpy[2]);

  ResetStartTime();

  std::cout << "-------A1Sim init Complete-------" << std::endl;
}

void qrRobotA1Sim::ImuCallback(const sensor_msgs::Imu &msg)
{
  robotStateBuffer.imu.quaternion[0] = msg.orientation.w;
  robotStateBuffer.imu.quaternion[1] = msg.orientation.x;
  robotStateBuffer.imu.quaternion[2] = msg.orientation.y;
  robotStateBuffer.imu.quaternion[3] = msg.orientation.z;

  Eigen::Matrix<float, 4, 1> quaternion = {robotStateBuffer.imu.quaternion[0],
                                           robotStateBuffer.imu.quaternion[1],
                                           robotStateBuffer.imu.quaternion[2],
                                           robotStateBuffer.imu.quaternion[3]};

  Eigen::Matrix<float, 3, 1> rpy = math::Quat2Rpy(quaternion);

  robotStateBuffer.imu.rpy[0] = rpy[0];
  robotStateBuffer.imu.rpy[1] = rpy[1];
  robotStateBuffer.imu.rpy[2] = rpy[2];

  robotStateBuffer.imu.gyroscope[0] = msg.angular_velocity.x;
  robotStateBuffer.imu.gyroscope[1] = msg.angular_velocity.y;
  robotStateBuffer.imu.gyroscope[2] = msg.angular_velocity.z;

  robotStateBuffer.imu.acc[0] = msg.linear_acceleration.x;
  robotStateBuffer.imu.acc[1] = msg.linear_acceleration.y;
  robotStateBuffer.imu.acc[2] = msg.linear_acceleration.z;
}

void qrRobotA1Sim::FRhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    robotStateBuffer.q[0] = msg.q;
    robotStateBuffer.dq[0] = msg.dq;
}

void qrRobotA1Sim::FRthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    robotStateBuffer.q[1] = msg.q;
    robotStateBuffer.dq[1] = msg.dq;
}

void qrRobotA1Sim::FRcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    robotStateBuffer.q[2] = msg.q;
    robotStateBuffer.dq[2] = msg.dq;
}

void qrRobotA1Sim::FLhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    robotStateBuffer.q[3] = msg.q;
    robotStateBuffer.dq[3] = msg.dq;
}

void qrRobotA1Sim::FLthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    robotStateBuffer.q[4]= msg.q;
    robotStateBuffer.dq[4] = msg.dq;
}

void qrRobotA1Sim::FLcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    robotStateBuffer.q[5] = msg.q;
    robotStateBuffer.dq[5] = msg.dq;
}

void qrRobotA1Sim::RRhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    robotStateBuffer.q[6] = msg.q;
    robotStateBuffer.dq[6] = msg.dq;
}

void qrRobotA1Sim::RRthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    robotStateBuffer.q[7]= msg.q;
    robotStateBuffer.dq[7] = msg.dq;
}

void qrRobotA1Sim::RRcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    robotStateBuffer.q[8] = msg.q;
    robotStateBuffer.dq[8] = msg.dq;
}

void qrRobotA1Sim::RLhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    robotStateBuffer.q[9]= msg.q;
    robotStateBuffer.dq[9] = msg.dq;
}

void qrRobotA1Sim::RLthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    robotStateBuffer.q[10]= msg.q;
    robotStateBuffer.dq[10]= msg.dq;
}

void qrRobotA1Sim::RLcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    robotStateBuffer.q[11] = msg.q;
    robotStateBuffer.dq[11] = msg.dq;
}

void qrRobotA1Sim::FRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    robotStateBuffer.footForce[0] = msg.wrench.force.z;
}

void qrRobotA1Sim::FLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    robotStateBuffer.footForce[1] = msg.wrench.force.z;
}

void qrRobotA1Sim::RRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    robotStateBuffer.footForce[2] = msg.wrench.force.z;
}

void qrRobotA1Sim::RLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    robotStateBuffer.footForce[3] = msg.wrench.force.z;
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

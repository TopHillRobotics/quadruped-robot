#ifndef QR_ROBOT_A1SIM_H
#define QR_ROBOT_A1SIM_H

#include <ros/ros.h>
#include <iostream>
#include <array>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>

#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"

#include "qr_robot.h"
#include "qr_robot_state.h"

/**
 * @brief The qrRobotA1Sim class is used in gazebo simulation
 */
class qrRobotA1Sim: public qrRobot
{
public:
  qrRobotA1Sim(ros::NodeHandle &nhIn, std::string configFilePath);

  ~qrRobotA1Sim();

  void Observation();

  void SendCmd();

  void ImuCallback(const sensor_msgs::Imu &msg);

  void FRhipCallback(const unitree_legged_msgs::MotorState &msg);

  void FRthighCallback(const unitree_legged_msgs::MotorState &msg);

  void FRcalfCallback(const unitree_legged_msgs::MotorState &msg);

  void FLhipCallback(const unitree_legged_msgs::MotorState &msg);

  void FLthighCallback(const unitree_legged_msgs::MotorState &msg);

  void FLcalfCallback(const unitree_legged_msgs::MotorState &msg);

  void RRhipCallback(const unitree_legged_msgs::MotorState &msg);

  void RRthighCallback(const unitree_legged_msgs::MotorState &msg);

  void RRcalfCallback(const unitree_legged_msgs::MotorState &msg);

  void RLhipCallback(const unitree_legged_msgs::MotorState &msg);

  void RLthighCallback(const unitree_legged_msgs::MotorState &msg);

  void RLcalfCallback(const unitree_legged_msgs::MotorState &msg);

  void FRfootCallback(const geometry_msgs::WrenchStamped &msg);

  void FLfootCallback(const geometry_msgs::WrenchStamped &msg);

  void RRfootCallback(const geometry_msgs::WrenchStamped &msg);

  void RLfootCallback(const geometry_msgs::WrenchStamped &msg);

private:

  /**
   * @brief node handler of ROS
   */
  ros::NodeHandle &nodeHandler;

  /**
   * @brief state from ROS firstly save date to this buffer
   */
  qrRobotState robotStateBuffer;

  /**
   * @brief commands of unitree A1
   */
  unitree_legged_msgs::LowCmd lowCmd;

  /**
   * @brief ros publisher of joint commands
   */
  ros::Publisher jointCmdPub[12];

  /**
   * @brief ros subscriber of joint state
   */
  ros::Subscriber jointStateSub[12];

  /**
   * @brief ros subscriber of foot force
   */
  ros::Subscriber footForceSub[4];

  /**
   * @brief ros subscriber of IMU
   */
  ros::Subscriber imuSub;
};

#endif // QR_ROBOT_A1SIM_H

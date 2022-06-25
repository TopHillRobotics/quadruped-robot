// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef QR_ROBOT_H
#define QR_ROBOT_H

#include <array>
#include <unordered_map>

#include "qr_robot_config.h"
#include "qr_motor_cmd.h"
#include "qr_robot_state.h"
#include "common/qr_types.h"
/**
 *  @brief a base class for all robot classes.It stores runtime status and data of the robot.
 */
class qrRobot
{
public:

  /**
   *  @brief Constructor of the class
   */
  qrRobot();

  qrRobot(std::string path);

  /**
   *  @brief Destructor of the class
   */
  virtual ~qrRobot();


  /**
   * @brief load config from path
   * @param path to config file
   * @see qrRobotConfig
   */
  void LoadConfig(std::string path);

  /**
   *  @brief Update the state of the robot.
   */
  virtual void Update();

  /**
   *  @brief Receive robot state and store information to robotState
   */
  virtual void Observation()=0;

  /**
   * @brief send command to motors according to vector qrMotorCmd. Depends on type of robot
   */
  virtual void SendCmd()=0;

  /**
   * @brief get the motor command to be executed
   * @return result of motor command
   */
  std::array<qrMotorCmd, 12> GetCmd();

  /**
   * @brief setCmd config the cmds
   * @param motorCmdValues: values needed to config cmds
   * @param mode: control mode the the motorCmdValues
   */
  void SetCmd(const Eigen::MatrixXf &motorCmdValues, MotorMode mode);

  /**
   * @brief set target angle values to cmds
   * @param qValues: target value matrix
   */
  void SetAngleCmd(const Eigen::Matrix<float, 12, 1> &qValues);

  /**
   * @brief set target torque values to cmds
   * @param cmdValues: target value matrix
   */
  void SetTorqueCmd(const Eigen::Matrix<float, 12, 1> &tauValues);

  /**
   * @brief set target torque values to cmds
   * @param cmdValues: target value matrix
   */
  void SetHybridCmd(const Eigen::Matrix<float, 5, 12> &cmdValues);

  /**
   * @brief get config of the robot
   * @return config of the robot
   */
  qrRobotConfig* getRobotConfig();

  /**
   * @brief get state of the robot
   * @return state of the robot
   */
  qrRobotState* getRobotState();

protected:

  /**
   * @brief stores the static config of the robot
   */
  qrRobotConfig* config;

  /**
   * @brief stores the command that will execute at each motor
   */
  std::array<qrMotorCmd, 12> cmds;

  /**
   * @brief robot state from observation
   */
  qrRobotState* robotState;

  /**
   * @brief robot base position in world frame
   */
  Eigen::Matrix<float, 3, 1> position;

  /**
   * @brief robot base orientation in world frame
   */
  Eigen::Matrix<float, 4, 1> orientation;

  /**
   * @brief robot calibrated rpy in world frame
   */
  Eigen::Matrix<float, 3, 1> rpy;

  // TODO: check what it is
  /**
   * @brief acceleration of robot
   */
  Eigen::Matrix<float, 3, 1> drpy;

  // TODO: check what it is
  /**
   * @brief linear acceleration of robot
   */
  Eigen::Matrix<float, 3, 1> linearAcc;

  /**
   * @brief angles of 12 motors
   */
  Eigen::Matrix<float, 12, 1> motorq;

  /**
   * @brief velocity of 12 motors
   */
  Eigen::Matrix<float, 12, 1> motordq;

  /**
   * @brief the contact status of 4 foot
   */
  Eigen::Matrix<bool, 4, 1> footContact;

};

#endif // QR_ROBOT_H

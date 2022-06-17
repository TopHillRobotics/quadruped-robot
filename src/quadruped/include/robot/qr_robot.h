// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: Xinyu Zhang   email: tophill.robotics@gmail.com

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

#include <vector>
#include <unordered_map>
#include "qr_robot_config.h"
#include "qr_motor_cmd.h"
#include "qr_robotState.h"

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

  void loadConfig(std::string path);
  /**
   *  @brief Destructor of the class
   */
  virtual ~qrRobot();

  /**
   *  @brief Receive robot state and store information to robotState
   */
  virtual void observation()=0;

  /**
   *  @brief Update the state of the robot.
   */
  virtual void update();

  /**
   * @brief send command to motors according to vector qrMotorCmd. Depends on type of robot
   */
  virtual void sendCmd()=0;

  /**
   * @brief get the motor command to be executed
   * @return result of motor command
   */
  std::vector<qrMotorCmd> getCmd();

protected:

  /**
   * @brief stores the static config of the robot
   */
  qrRobotConfig* config;

  /**
   * @brief stores the command that will execute at each motor
   */
  std::vector<qrMotorCmd> *cmds;

  /**
   * @brief robot state from observation
   */
  qrRobotState robotState;

  /**
   * @brief number of motors
   */
  static const int numMotor  = 12;

  /**
   * @brief number of legs
   */
  static const int numLegs   = 4;

  /**
   * @brief DOF of each leg
   */
  static const int dofPerLeg = 3;

  /**
   * @brief robot base position in world frame
   */
  Eigen::Matrix<float, 3, 1> position;

  /**
   * @brief robot base orientation in world frame
   */
  Eigen::Matrix<float, 4, 1> orientation;

  /**
   * @brief robot rpy in world frame
   */
  Eigen::Matrix<float, 3, 1> rpy; //yaw calibrated robot rpy in world frame

  // TODO: check what it is
  /**
   * @brief change rate of rpy
   */
  Eigen::Matrix<float, 3, 1> drpy; //robot rpy rate in base frame

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

  /**
   * @brief mapping of different control modes
   */
  std::unordered_map<int, std::string> modeMap = {{0, "velocity"}, {1, "position"}, {2, "walk"}};
};

#endif // QR_ROBOT_H

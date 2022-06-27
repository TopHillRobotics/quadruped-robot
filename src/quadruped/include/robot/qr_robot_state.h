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

#ifndef QR_ROBOT_STATE_H
#define QR_ROBOT_STATE_H

#include <iostream>
#include <array>
#include <Eigen/Dense>

#include "robot/qr_robot_config.h"

enum Joint{
  FRhip, FRthigh, FRcalf, \
  FLhip, FLthigh, FLcalf, \
  RRhip, RRthigh, RRcalf, \
  RLhip, RLthigh, RLcalf
};

// TODO: check initialization
/**
 * @brief The qrIMU struct saves data from imu
 */
struct qrIMU
{

  /**
   * @brief normalized quaternion  (w,x,y,z)
   */
  Eigen::Matrix<float, 4, 1> quaternion;

  /**
   * @brief angular velocity （unit: rad/s)
   */
  Eigen::Matrix<float, 3, 1> gyroscope;

  // TODO: discuss
  /**
   * @brief acceleration  (unit: m/(s2) )
   */
  Eigen::Matrix<float, 3, 1> acc;

  /**
   * @brief euler angle（unit: rad)
   */
  Eigen::Matrix<float, 3, 1> rpy;

  /**
   * @brief calibrate the yaw when out [-PI, PI]
   * @return rpy with calibrated yaw
   */
  Eigen::Matrix<float, 3, 1> CalibratedYawRpy(){
    Eigen::Matrix<float, 3, 1> rpyCalibrate = rpy;
    if(rpyCalibrate(2, 0) >= float(M_PI)){
      rpyCalibrate(2, 0) -= float(2 * M_PI);
    }
    else if (rpyCalibrate(2, 0) <= -float(M_PI)) {
      rpyCalibrate(2, 0) += float(2 * M_PI);
    }
    return rpyCalibrate;
  }
};

/**
 * @brief The qrLegState struct stores observed data every iteration
 */
class qrRobotState
{

public:

  /**
   * @brief constructor of qrRobotState
   */
  qrRobotState();

  /**
   * @brief constructor of qrRobotState
   * @param robotConfig used to calculate current state
   */
  qrRobotState(qrRobotConfig* robotConfig);

  /**
   * @brief imu related date
   */
  qrIMU imu;

  /**
   * @brief current angle (unit: radian)
   */
  Eigen::Matrix<float, 12, 1> q;

  /**
   * @brief current velocity (unit: radian/second)
   */
  Eigen::Matrix<float, 12, 1> dq;

  /**
   * @brief current estimated output torque (unit: N.m)
   */
  Eigen::Matrix<float, 12, 1> tau;
  /**
   * @brief time between this currentStamp and last stamp;
   *        at the first time, delta time will set 0.001 as default
   */
  float deltaTime;

  /**
   * @brief set current time stamp from hardware
   * @param tick: time stamp
   */
  void setTimeStamp(uint32_t tick);

  /**
   * @brief get a vector of motor velocities
   * @param legId: which leg's velocity
   * @return vector of motor velocities
   */
  Eigen::Matrix<float, 3, 1> GetDq(unsigned int legId);


  /**
   * @brief get a vector of motor velocities
   * @param legId: which leg's velocity
   * @return vector of motor velocities
   */
  Eigen::Matrix<float, 3, 1> GetQ(unsigned int legId);

  /**
   * @brief return foot position accroding to base frame
   * @return foot position in base frame
   */
  Eigen::Matrix<float, 3, 4> GetFootPositionInBaseFrame();

  /**
   * @brief get roll pitch yall of robot body
   * @return roll, pitch, yaw
   */
  Eigen::Matrix<float, 3, 1> GetRpy();

  /**
   * @brief get velocity of roll, pitch, yaw
   * @return velocity of roll, pitch, yaw
   */
  Eigen::Matrix<float, 3, 1> GetDrpy();

  /**
   * @brief return orientation of robot body
   * @return orientation of robot body
   */
  Eigen::Matrix<float, 3, 1> GetBaseOrientation();

  /**
   * @brief get current jacobian of leg legId
   * @param legId: which leg to calculate
   * @return current Jacobian
   */
  Eigen::Matrix<float, 3, 3> GetJacobian(int legId);

  /**
   * @brief convert contact force of one leg to joint torque
   * @param contractForce: contact force of the leg
   * @param legId: which leg to convert
   * @return joint torques, totally 3 joints
   */
  Eigen::Matrix<float, 3, 1> ContactForce2JointTorque(Eigen::Matrix<float, 3, 1> contractForce, int legId);

  inline Eigen::Matrix<bool, 4, 1> GetFootContacts() const
  {
      return footContact;
  }

private:

  /**
   * @brief the smallest time interval of each loop, which means max frequence is 1000HZ
   */
  static constexpr float leastDeltaTime = 0.001f;

  /**
   * @see robotConfig
   */
  qrRobotConfig* robotConfig;

  /**
   * @brief last time stamp
   */
  uint32_t lastStamp = 0;

  /**
   * @brief force on foot
   */
  Eigen::Matrix<float, 4, 1> footForce;

  // TODO: get and set
  /**
   * @brief the contact status of 4 foot
   */
  Eigen::Matrix<bool, 4, 1> footContact;
};

#endif // QR_ROBOT_STATE_H

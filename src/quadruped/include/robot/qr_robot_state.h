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
  Eigen::Matrix<float, 3, 1> q;

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
 * @brief The qrMotor struct saves data from motor
 */
struct qrMotor
{
  /**
   * @brief current angle (unit: radian)
   */
  float q;

  /**
   * @brief current velocity (unit: radian/second)
   */
  float dq;

  /**
   * @brief current estimated output torque (unit: N.m)
   */
  float tau;
};

/**
 * @brief The qrLegState struct stores observed data every iteration
 */
struct qrRobotState
{

  qrRobotState(){
    deltaTime = 0.001f;
  }
  /**
   * @brief imu related date
   */
  qrIMU imu;

  /**
   * @brief force on foot
   */
  Eigen::Matrix<float, 4, 1> footForce;

  // TODO: get and set
  /**
   * @brief the contact status of 4 foot
   */
  Eigen::Matrix<bool, 4, 1> footContact;

  /**
   * @brief states of 12 motors
   */
  std::array<qrMotor, 12> motors;

  /**
   * @brief time between this currentStamp and last stamp;
   *        at the first time, delta time will set 0.001 as default
   */
  float deltaTime;

  /**
   * @brief set current time stamp from hardware
   * @param tick: time stamp
   */
  void setTimeStamp(uint32_t tick){
    if(lastStamp != 0){
      deltaTime = (tick - lastStamp) / 1000.0f;
    }
    lastStamp = tick;
  }

  /**
   * @brief get a vector of motor velocities
   * @param legId: which leg's velocity
   * @return vector of motor velocities
   */
  Eigen::Matrix<float, 3, 1> getDq(unsigned int legId){
    Eigen::Matrix<float, 3, 1> dq;
    dq << motors[legId * 3].dq, motors[legId * 3 + 1].dq, motors[legId * 3 + 2].dq;
    return dq;
  }


  /**
   * @brief get a vector of motor velocities
   * @param legId: which leg's velocity
   * @return vector of motor velocities
   */
  Eigen::Matrix<float, 3, 1> getQ(unsigned int legId){
    Eigen::Matrix<float, 3, 1> dq;
    dq << motors[legId * 3].q, motors[legId * 3 + 1].q, motors[legId * 3 + 2].q;
    return dq;
  }

private:

  /**
   * @brief the smallest time interval of each loop, which means max frequence is 1000HZ
   */
  static constexpr float leastDeltaTime = 0.001f;

  /**
   * @brief last time stamp
   */
  uint32_t lastStamp = 0;
};

#endif // QR_ROBOT_STATE_H

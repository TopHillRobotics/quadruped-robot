// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

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

#ifndef QR_ROBOT_VELOCITY_ESTIMATOR_H
#define QR_ROBOT_VELOCITY_ESTIMATOR_H

#include <Eigen/Dense>

#include "qr_tinyekf.h"
#include "qr_moving_window_filter.h"
#include "robot/qr_robot.h"
#include "robot/qr_robot_state.h"


/**
 * @brief The RobotVelocityEstimator class estimate the velocity of the robot
 */
class RobotVelocityEstimator {

public:

  /**
   * @brief RobotVelocityEstimator
   * @param robot
   * @param accelerometerVariance: noise estimation for accelerometer reading
   * @param sensorVariance: noise estimation for motor velocity reading.
   * @param initialVariance: covariance estimation of initial state.
   * @param filerWindowSize: window size of filter
   */
  RobotVelocityEstimator(qrRobot *robot,
                         float accelerometerVariance = 0.1f,
                         float sensorVariance = 0.1f,
                         unsigned int filerWindowSize = 120);

  // TODO: check wheter filter should be reset
  /**
   * @brief reset the moving window filter and estimated velocity
   * @param windowSize: window size to be reset
   */
  void Reset(unsigned int windowSize);

  /** @brief get com velocity expressed in base frame. */
  const Eigen::Vector3f &GetEstimatedVelocity() const
  {
      return estimatedVelocity;
  }

  /**
   * @brief Estimate the velocity
   */
  void Estimate();

private:

  /**
   * @see qrRobotState
   */
  qrRobotState *robotState;

  /**
   * @see qrRobotConfig
   */
  qrRobotConfig * robotConfig;

  /**
   * @brief the velocity result of estimation
   */
  Eigen::Vector3f estimatedVelocity;

  /**
   * @brief velocity moving window filter along X axis
   */
  qrMovingWindowFilter velocityFilterX;

  /**
   * @brief velocity moving window filter along Y axis
   */
  qrMovingWindowFilter velocityFilterY;

  /**
   * @brief velocity moving window filter along Z axis
   */
  qrMovingWindowFilter velocityFilterZ;

  /**
   * @brief kalman filter
   */
  qrTinyEKF *filter;

  Eigen::Matrix<float, 3, 1> movingWindowUpdate(float x, float y, float z);

};

#endif // QR_ROBOT_VELOCITY_ESTIMATOR_H

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

#include <deque>
#include <numeric>

#include "tinyekf/TinyEKF.h"

#include "common/qr_se3.h"
#include "robots/qr_robot.h"
#include "planner/qr_gait_generator.h"
#include "state_estimator/qr_filter.h"


/** @brief Initiates the velocity estimator.
 *  See filterpy documentation in the link below for more details.
 *  https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html
 *
 * @param robot: the robot class for velocity estimation.
 * @param accelerometer_variance: noise estimation for accelerometer reading.
 * @param sensor_variance: noise estimation for motor velocity reading.
 * @param initial_covariance: covariance estimation of initial state.
 */
class qrRobotVelocityEstimator {
public:
    /**
     * @brief Estimates base velocity of A1 robot.
     * The velocity estimator consists of 2 parts:
     * 1) A state estimator for CoM velocity.
     *
     * Two sources of information are used:
     * The integrated reading of accelerometer and the velocity estimation from
     * contact legs. The readings are fused together using a Kalman Filter.
     *
     * 2) A moving average filter to smooth out velocity readings
     */
    qrRobotVelocityEstimator(qrRobot *robot,
                             qrGaitGenerator *gaitGeneratorIn,
                             float accelerometerVarianceIn = 0.1f,
                             float sensorVarianceIn = 0.1f,
                             float initialVarianceIn = 0.1f,
                             int movingWindowFilterSizeIn = 120);

    void Reset(float currentTime);

    /** @brief compute the time period between two adjacent imu message */
    float ComputeDeltaTime(const float tick);

    /** @brief Estimate the velocity */
    void Update(float currentTime);

    /** @brief get com velocity expressed in base frame. */
    const Vec3<float> &GetEstimatedVelocity() const
    {
        return estimatedVelocity;
    }

    const Vec3<float> &GetEstimatedAngularVelocity() const
    {
        return estimatedAngularVelocity;
    }

private:
    qrRobot *robot;
    qrGaitGenerator *gaitGenerator;
    int windowSize;
    float initialVariance;
    float lastTimestamp;
    Vec3<float> estimatedVelocity; // expressed in base frame
    Vec3<float> estimatedAngularVelocity;
    qrMovingWindowFilter velocityFilterX;
    qrMovingWindowFilter velocityFilterY;
    qrMovingWindowFilter velocityFilterZ;

    TinyEKF *filter;
};

#endif //ASCEND_QUADRUPED_CPP_ROBOT_VELOCITY_ESTIMATOR_H

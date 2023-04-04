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


#include "utils/qr_se3.h"
#include "robots/qr_robot.h"
#include "gait/qr_openloop_gait_generator.h"
#include "estimators/qr_moving_window_filter.hpp"

/* this is a external lib, but using some variables defined in qr_filter.hpp*/
#include "TinyEKF.h"


namespace Quadruped {

/** @brief Initiates the velocity estimator.
 *  See filterpy documentation in the link below for more details.
 *  https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html.
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
     * @param robot: the robot class for velocity estimation.
     * @param gaitGeneratorIn: generate desired gait schedule for locomotion.
     * @param userParametersIn: parameters for kalman filter and moving window algorithm.
     */
    qrRobotVelocityEstimator(qrRobot *robot,
                           qrGaitGenerator *gaitGeneratorIn,
                           qrUserParameters *userParametersIn);

    /**
     * @brief Reset the robot velocity estimator.
     * @param currentTime: time since the timer started
     */
    void Reset(float currentTime);

    /**
     * @brief Compute the time period between two adjacent imu message.
     */
    float ComputeDeltaTime(uint32_t tick);

    /**
     * @brief Estimate the velocity in base frame.
     */
    void Update(float currentTime);

    /**
     * @brief Getter method of member estimatedVelocity.
     */
    const Vec3<float> &GetEstimatedVelocity() const {
        return estimatedVelocity;
    };

    /**
     * @brief Getter method of member estimatedAngularVelocity.
     */
    const Vec3<float> &GetEstimatedAngularVelocity() const {
        return estimatedAngularVelocity;
    };

private:

    /**
     * @brief The robot class for the velocity estimation.
     */
    qrRobot *robot;

    /**
     * @brief Generate desired gait schedule for locomotion.
     */
    qrGaitGenerator *gaitGenerator;

    /**
     * @brief The window size of moving window algorithm.
     */
    int windowSize;

    /**
     * @brief The initila variance of kalman filter.
     */
    float initialVariance;

    /**
     * @brief Time stamp when last loop ended.
     */
    uint32_t lastTimestamp;

    /**
     * @brief Estimated velocity in base frame.
     */
    Vec3<float> estimatedVelocity;

    /**
     * @brief Estimated angular velocity in base frame.
     */
    Vec3<float> estimatedAngularVelocity;

    /**
     * @brief Moving window filter in x-axis for the kalman filter.
     */
    qrMovingWindowFilter<double, 1> velocityFilterX;

    /**
     * @brief Moving window filter in y-axis for the kalman filter.
     */
    qrMovingWindowFilter<double, 1> velocityFilterY;

    /**
     * @brief Moving window filter in z-axis for the kalman filter.
     */
    qrMovingWindowFilter<double, 1> velocityFilterZ;

    /**
     * @brief Moving window filter for liner acceleration.
     */
    qrMovingWindowFilter<float, 3> AccFilter;
    
    /**
     * @brief Kalman filter for the velocity estimation.
     */
    TinyEKF<3,3> *filter;

};

} // namespace Quadruped

#endif // QR_ROBOT_VELOCITY_ESTIMATOR_H

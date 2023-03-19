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

#ifndef QR_ROBOT_ESTIMATOR_H
#define QR_ROBOT_ESTIMATOR_H

#include "utils/qr_se3.h"
#include "robots/qr_robot.h"
#include "estimators/qr_robot_pose_estimator.h"
#include "estimators/qr_robot_velocity_estimator.h"
#include "estimators/qr_ground_surface_estimator.h"


/* CMU paramaters */
#define PROCESS_NOISE_PIMU 0.01
#define PROCESS_NOISE_VIMU 0.01
#define PROCESS_NOISE_PFOOT 0.01 
#define SENSOR_NOISE_PIMU_REL_FOOT 0.001
#define SENSOR_NOISE_VIMU_REL_FOOT 0.1
#define SENSOR_NOISE_ZFOOT 0.001
#define STATE_SIZE  18
#define MEAS_SIZE  28

namespace Quadruped {

/**
 * @brief estimate robot state including pose and velocity
 */
class qrRobotEstimator : public qrBaseStateEstimator {

public:

    /**
     * @brief Estimate robot state including pose and velocity.
     * @param robotIn: the robot class for the estimation.
     * @param gaitGeneratorIn: generate desired gait by the gait scheduler.
     * @param groundEstimatorIn: estimate the 3D plane where the feet contact.
     * @param userParametersIn: stores estimation configure for the estimator.
     */
    qrRobotEstimator(qrRobot *robotIn,
                   qrGaitGenerator *gaitGeneratorIn,
                   qrGroundSurfaceEstimator *groundEstimatorIn,
                   qrUserParameters *userParametersIn);

    /**
     * @brief Reset the estimator.
     * @param currentTime: currunt time since the timer started.
     */
    void Reset(float currentTime);

    /**
     * @brief Compute the time intervial for each loop.
     * @param tick: real robot state used for compution time for each loop.
     */
    float ComputeDeltaTime(uint32_t tick);

    /**
     * @brief Compute and update the zero moment point.
     * @return a vector of zero moment point.
     */
    Vec3<float> ComputeZMP();

    /**
     * @brief Update the robot kinematics state.
     * @param currentTime: time since the timer started.
     */
    void Update(float currentTime);

    /**
     * @brief Initialize the state of Filter.
     */
    void CMUInitState();

    /**
     * @brief Update foot contact state and contact force by Filter.
     * @param currentTime: time since the timer started.
     */
    void CMUUpdate(double currentTime);

    /**
     * @brief Getter mthod of member estimatedVelocity.
     */
    inline const Vec3<float> &GetEstimatedVelocity() const {
        return estimatedVelocity;
    };

    /**
     * @brief Getter mthod of member estimatedAngularVelocity.
     */
    inline const Vec3<float> &GetEstimatedAngularVelocity() const {
        return estimatedAngularVelocity;
    };

    /**
     * @brief Getter mthod of member estimatedPosition.
     */
    inline const Vec3<float> &GetEstimatedPosition() const {
        return estimatedPosition;
    };

    /**
     * @brief Getter mthod of member estimatedRPY.
     */
    inline const Vec3<float> &GetEstimatedRPY() {
        return estimatedRPY;
    };

private:

    /**
     * @brief The robot class for estimation.
     */
    qrRobot *robot;

    /**
     * @brief Velocity estimator for robot.
     */
    qrRobotVelocityEstimator velocityEstimator;

    /**
     * @brief Pose estimator for robot.
     */
    qrRobotPoseEstimator poseEstimator;

    /**
     * @brief Time since reset the timer.
     */
    float timeSinceReset;

    /**
     * @brief Estimated position in world frame.
     */
    Vec3<float> estimatedPosition;

    /**
     * @brief Estimated row-pitch-yaw in world frame.
     */
    Vec3<float> estimatedRPY;

    /**
     * @brief Estimated velocity in base frame.
     */
    Vec3<float> estimatedVelocity;

    /**
     * @brief Estimated velocity in base frame on last updation.
     */
    Vec3<float> lastEstimatedVelocity;

    /**
     * @brief Estimated angular velocity in world frame.
     */
    Vec3<float> estimatedAngularVelocity;

    /**
     * @brief Last tick that real robot state used for compution time for each loop.
     */
    uint32_t lastTimestamp;

    /**
     * @brief Estimation state.
     * 0 1 2 pos 3 4 5 vel 6 7 8 foot pos FL 9 10 11 foot pos FR 12 13 14 foot pos RL 15 16 17 foot pos RR
     */
    Eigen::Matrix<double, STATE_SIZE, 1> x; // estimation state

    /**
     * @brief Estimation state after process update.
     */
    Eigen::Matrix<double, STATE_SIZE, 1> xbar; // estimation state after process update

    /**
     * @brief Estimation state covariance.
     */
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P; // estimation state covariance

    /**
     * @brief Estimation state covariance after process update.
     */
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar; // estimation state covariance after process update

    /**
     * @brief Estimation state transition.
     */
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> A; // estimation state transition

    /**
     * @brief Estimation state transition.
     */
    Eigen::Matrix<double, STATE_SIZE, 3> B; // estimation state transition

    /**
     * @brief Estimation state transition noise.
     */
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q; // estimation state transition noise

    /**
     * @brief Observation.
     * observation
     * 0 1 2   FL pos residual
     * 3 4 5   FR pos residual
     * 6 7 8   RL pos residual
     * 9 10 11 RR pos residual
     * 12 13 14 vel residual from FL
     * 15 16 17 vel residual from FR
     * 18 19 20 vel residual from RL
     * 21 22 23 vel residual from RR
     * 24 25 26 27 foot height
     */
    Eigen::Matrix<double, MEAS_SIZE, 1> y;

    /**
     * @brief Estimated observation.
     */
    Eigen::Matrix<double, MEAS_SIZE, 1> yhat;

    /**
     * @brief Estimated observation.
     */
    Eigen::Matrix<double, MEAS_SIZE, 1> error_y;

    /**
     * @brief S^-1*error_y.
     */
    Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y;

    /**
     * @brief Estimation state observation.
     */
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> C;

    /**
     * @brief S^-1*C.
     */
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> SC;

    /**
     * @brief Estimation state observation noise
     */
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R;

    /**
     * @brief 3x3 identity.
     */
    Eigen::Matrix<double, 3, 3> eye3;

    /**
     * @brief Innovation (or pre-fit residual) covariance.
     */
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S;

    /**
     * @brief Kalman gain.
     */
    Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> K;

    /**
     * @brief If assume the ground is flat.
     */
    bool assume_flat_ground = true;

    /**
     * @brief The maximum sliding friction.
     */
    double smooth_foot_force[4];

    /**
     * @brief Estimated contact states.
     */
    double estimated_contacts[4];


};

} // Namespace Quadruped

#endif // QR_ROBOT_ESTIMATOR_H

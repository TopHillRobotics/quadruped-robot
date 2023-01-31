/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhu Yijie
* Create: 2021-11-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*         add ZMP computation function. @ Zhu Yijie 2022.04.01
*         add state-coupled EKF from CMU code version . @ Zhu Yijie 2022.04.15
*/

#ifndef ASCEND_QUADRUPED_CPP_ROBOT_ESTIMATOR_H
#define ASCEND_QUADRUPED_CPP_ROBOT_ESTIMATOR_H

#include "utils/se3.h"
#include "robots/robot.h"
// #include "inekf_cpp_interface.h"
#include "estimators/robot_pose_estimator.h"
#include "estimators/robot_velocity_estimator.h"
#include "estimators/ground_estimator.h"
// cmu paramaters
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
    class RobotEstimator : public BaseEstimator {
    public:
        RobotEstimator(Robot *robotIn,
                       GaitGenerator *gaitGeneratorIn,
                       GroundSurfaceEstimator *groundEstimatorIn,
                       UserParameters *userParametersIn);

        void Reset(float currentTime);

        float ComputeDeltaTime(const LowState *robotState);
        
        Vec3<float> ComputeZMP();

        void Update(float currentTime);

        void CMUInitState();

        void CMUUpdate(double currentTime);

        /** @brief get com velocity expressed in base frame. */
        inline const Vec3<float> &GetEstimatedVelocity() const
        {
            return estimatedVelocity;
        }

        inline const Vec3<float> &GetEstimatedAngularVelocity() const
        {
            return estimatedAngularVelocity;
        }

        inline const Vec3<float> &GetEstimatedPosition() const
        {
            return estimatedPosition;
        }

        inline const Vec3<float> &GetEstimatedRPY()
        {
            // return inekf_.getRotation().cast<float>();
            return estimatedRPY;
        }

    private:
        Robot *robot;
        RobotVelocityEstimator velocityEstimator;
        RobotPoseEstimator poseEstimator;

        float timeSinceReset;
        Vec3<float> estimatedPosition;
        Vec3<float> estimatedRPY;
        Vec3<float> estimatedVelocity;
        Vec3<float> lastEstimatedVelocity;
        Vec3<float> estimatedAngularVelocity;
        // inekf::RobotState estimatedState; 
        // inekf::INEKFInterface inekf_;
        // IMU imuData;
        float lastTimestamp;

        // CMU 
        // state
        // 0 1 2 pos 3 4 5 vel 6 7 8 foot pos FL 9 10 11 foot pos FR 12 13 14 foot pos RL 15 16 17 foot pos RR
        Eigen::Matrix<double, STATE_SIZE, 1> x; // estimation state
        Eigen::Matrix<double, STATE_SIZE, 1> xbar; // estimation state after process update
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P; // estimation state covariance
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar; // estimation state covariance after process update
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> A; // estimation state transition
        Eigen::Matrix<double, STATE_SIZE, 3> B; // estimation state transition
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q; // estimation state transition noise

        // observation
        // 0 1 2   FL pos residual
        // 3 4 5   FR pos residual
        // 6 7 8   RL pos residual
        // 9 10 11 RR pos residual
        // 12 13 14 vel residual from FL
        // 15 16 17 vel residual from FR
        // 18 19 20 vel residual from RL
        // 21 22 23 vel residual from RR
        // 24 25 26 27 foot height
        Eigen::Matrix<double, MEAS_SIZE, 1> y; //  observation
        Eigen::Matrix<double, MEAS_SIZE, 1> yhat; // estimated observation
        Eigen::Matrix<double, MEAS_SIZE, 1> error_y; // estimated observation
        Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y; // S^-1*error_y
        Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> C; // estimation state observation
        Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> SC; // S^-1*C
        Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R; // estimation state observation noise
        // helper matrices
        Eigen::Matrix<double, 3, 3> eye3; // 3x3 identity
        Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S; // Innovation (or pre-fit residual) covariance
        Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> K; // kalman gain

        bool assume_flat_ground = true;
        // variables to process foot force
        double smooth_foot_force[4];
        double estimated_contacts[4];

        int count = 0;
    };


} // namespace Quadruped

#endif//ASCEND_QUADRUPED_CPP_ROBOT_ESTIMATOR_H

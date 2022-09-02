/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhu Yijie
* Create: 2021-11-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef QR_ROBOT_ESTIMATOR_H
#define QR_ROBOT_ESTIMATOR_H

#include "common/qr_se3.h"
#include "robots/qr_robot.h"
// #include "inekf_cpp_interface.h"
#include "state_estimator/qr_robot_pose_estimator.h"
#include "state_estimator/qr_robot_velocity_estimator.h"
#include "state_estimator/qr_ground_estimator.h"

/**
 * @brief estimate robot state including pose and velocity
 */
class qrRobotEstimator {
public:
    qrRobotEstimator(qrRobot *robotIn,
                    qrGaitGenerator *gaitGeneratorIn,
                    qrGroundSurfaceEstimator *groundEstimatorIn);

    void Reset(float currentTime);

    float ComputeDeltaTime(const LowState *robotState);

    void Update(float currentTime);

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
    qrRobot *robot;
    qrRobotVelocityEstimator velocityEstimator;
    qrRobotPoseEstimator poseEstimator;

    float timeSinceReset;
    Vec3<float> estimatedPosition;
    Vec3<float> estimatedRPY;
    Vec3<float> estimatedVelocity;
    Vec3<float> estimatedAngularVelocity;
    // inekf::qrRobotState estimatedState; 
    // inekf::INEKFInterface inekf_;
    // IMU imuData;
    float lastTimestamp;

};

#endif//ASCEND_QUADRUPED_CPP_ROBOT_ESTIMATOR_H

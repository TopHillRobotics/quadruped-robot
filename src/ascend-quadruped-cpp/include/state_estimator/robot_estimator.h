/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhu Yijie
* Create: 2021-11-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_ROBOT_ESTIMATOR_H
#define ASCEND_QUADRUPED_CPP_ROBOT_ESTIMATOR_H

#include "utils/se3.h"
#include "robots/robot.h"
// #include "inekf_cpp_interface.h"
#include "state_estimator/robot_pose_estimator.h"
#include "state_estimator/robot_velocity_estimator.h"
#include "state_estimator/ground_estimator.h"

namespace Quadruped {
    /**
     * @brief estimate robot state including pose and velocity
     */
    class RobotEstimator {
    public:
        RobotEstimator(Robot *robotIn,
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
        Robot *robot;
        RobotVelocityEstimator velocityEstimator;
        RobotPoseEstimator poseEstimator;

        float timeSinceReset;
        Vec3<float> estimatedPosition;
        Vec3<float> estimatedRPY;
        Vec3<float> estimatedVelocity;
        Vec3<float> estimatedAngularVelocity;
        // inekf::RobotState estimatedState; 
        // inekf::INEKFInterface inekf_;
        // IMU imuData;
        float lastTimestamp;

    };

} // namespace Quadruped

#endif//ASCEND_QUADRUPED_CPP_ROBOT_ESTIMATOR_H

/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhu Yijie
* Create: 2021-11-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_ROBOT_POSE_ESTIMATOR_H
#define ASCEND_QUADRUPED_CPP_ROBOT_POSE_ESTIMATOR_H

#include "robots/robot.h"
#include "mpc_controller/qr_gait_generator.h"
#include "state_estimator/robot_velocity_estimator.h"
#include "state_estimator/ground_estimator.h"

namespace Quadruped {
    /**
     * @brief estimates robot pose, currently the code implemented position in x-axis and y-axis.
     */
    class RobotPoseEstimator {
    public:
        RobotPoseEstimator(Robot *robotIn,
                           qrGaitGenerator *gaitGeneratorIn,
                           qrGroundSurfaceEstimator *groundEstimatorIn,
                           RobotVelocityEstimator *velocityEstimator);

        void Reset(float currentTime);

        float ComputeDeltaTime(const LowState *robotState);

        void Update(float currentTime);

        /** 
         * @brief caculate position of foot toe in base frame, then represent this vector in world frame, 
                  is not absolute height of body in world frame. 
        */
        float EstimateRobotHeight();

        void ComputePose(float deltaTime);

        const Vec6<float> &GetEstimatedPose() const
        {
            return estimatedPose;
        }

    private:
        Robot *robot;
        float lastTimestamp;
        Vec6<float> estimatedPose;
        qrGaitGenerator *gaitGenerator;
        qrGroundSurfaceEstimator *groundEstimator;
        RobotVelocityEstimator *velocityEstimator;

    };

} // namespace Quadruped

#endif//ASCEND_QUADRUPED_CPP_ROBOT_VELOCITY_ESTIMATOR_H
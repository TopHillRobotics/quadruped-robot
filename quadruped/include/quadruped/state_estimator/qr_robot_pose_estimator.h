/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhu Yijie
* Create: 2021-11-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef QR_ROBOT_POSE_ESTIMATOR_H
#define QR_ROBOT_POSE_ESTIMATOR_H

#include "robots/qr_robot.h"
#include "planner/qr_gait_generator.h"
#include "state_estimator/qr_robot_velocity_estimator.h"
#include "state_estimator/qr_ground_estimator.h"


/**
 * @brief estimates robot pose, currently the code implemented position in x-axis and y-axis.
 */
class qrRobotPoseEstimator {
public:
    qrRobotPoseEstimator(qrRobot *robotIn,
                        qrGaitGenerator *gaitGeneratorIn,
                        qrGroundSurfaceEstimator *groundEstimatorIn,
                        qrRobotVelocityEstimator *velocityEstimator);

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
    qrRobot *robot;
    float lastTimestamp;
    Vec6<float> estimatedPose;
    qrGaitGenerator *gaitGenerator;
    qrGroundSurfaceEstimator *groundEstimator;
    qrRobotVelocityEstimator *velocityEstimator;

};

#endif //QR_ROBOT_POSE_ESTIMATOR_H


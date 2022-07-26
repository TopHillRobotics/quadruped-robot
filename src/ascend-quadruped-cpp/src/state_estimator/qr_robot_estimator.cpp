/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhu Yijie
* Create: 2021-11-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#include "state_estimator/qr_robot_estimator.h"

namespace Quadruped {
    qrRobotEstimator::qrRobotEstimator(qrRobot *robotIn,
                                   qrGaitGenerator *gaitGeneratorIn,
                                   qrGroundSurfaceEstimator *groundEstimatorIn)
        : robot(robotIn), velocityEstimator(robotIn, gaitGeneratorIn),
          poseEstimator(robotIn, gaitGeneratorIn, groundEstimatorIn, &velocityEstimator)
    {        
        estimatedVelocity = velocityEstimator.GetEstimatedVelocity();
        estimatedAngularVelocity = velocityEstimator.GetEstimatedAngularVelocity();
        const Vec6<float> &pose = poseEstimator.GetEstimatedPose();
        estimatedPosition = pose.head(3);
        estimatedRPY = pose.tail(3);
        std::cout << "estimatedPosition = " << estimatedPosition.transpose() << std::endl;
        lastTimestamp = 0.0;
    }

    void qrRobotEstimator::Reset(float currentTime)
    {
        timeSinceReset = robot->GetTimeSinceReset();
        velocityEstimator.Reset(currentTime);
        poseEstimator.Reset(currentTime);
        estimatedVelocity = velocityEstimator.GetEstimatedVelocity();
        estimatedAngularVelocity = velocityEstimator.GetEstimatedAngularVelocity();
        const Vec6<float> &pose = poseEstimator.GetEstimatedPose();
        estimatedPosition = pose.head(3);
        estimatedRPY = pose.tail(3);
        lastTimestamp = 0.0;
        std::cout << "reset pos= " << estimatedPosition.transpose() << std::endl;
    }

    float qrRobotEstimator::ComputeDeltaTime(const LowState *robotState)
    {
        float deltaTime;
        if (std::abs(lastTimestamp) < 1e-5) {
            // First timestamp received, return an estimated delta_time.
            deltaTime = robot->timeStep;
        } else {
            deltaTime = (robotState->tick - lastTimestamp) / 1000.;
        }
        lastTimestamp = robotState->tick;
        return deltaTime;
    }

    void qrRobotEstimator::Update(float currentTime)
    {
        velocityEstimator.Update(currentTime);
        poseEstimator.Update(currentTime);

        estimatedVelocity = velocityEstimator.GetEstimatedVelocity();
        estimatedAngularVelocity = velocityEstimator.GetEstimatedAngularVelocity();
        const Vec6<float> &pose = poseEstimator.GetEstimatedPose();
        estimatedPosition = pose.head(3);
        estimatedRPY = pose.tail(3);
    }
} // namespace Quadruped

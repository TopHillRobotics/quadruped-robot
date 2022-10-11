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


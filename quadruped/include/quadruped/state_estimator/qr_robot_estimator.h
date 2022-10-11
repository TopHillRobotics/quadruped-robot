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

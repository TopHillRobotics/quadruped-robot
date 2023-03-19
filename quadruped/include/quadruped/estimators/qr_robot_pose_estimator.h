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
#include "gait/qr_openloop_gait_generator.h"
#include "estimators/qr_robot_velocity_estimator.h"
#include "estimators/qr_ground_surface_estimator.h"


namespace Quadruped {

/**
 * @brief estimates robot pose, currently the code implemented position in x-axis and y-axis.
 */
class qrRobotPoseEstimator {

public:

    /**
     * @brief Constructor of robot pose estimator.
     * @param robotIn: the robot class for pose estimation.
     * @param gaitGeneratorIn: generate desired gait schedule for locomotion.
     * @param groundEstimatorIn: estimate the 3D plane where the feet contact.
     * @param velocityEstimator: the estimator class for velocity estimation.
     */
    qrRobotPoseEstimator(qrRobot *robotIn,
                       qrGaitGenerator *gaitGeneratorIn,
                       qrGroundSurfaceEstimator *groundEstimatorIn,
                       qrRobotVelocityEstimator *velocityEstimator);

    /**
     * @brief Reset estimater
     * @param currentTime: time since the timer started.
     */
    void Reset(float currentTime);

    /**
     * @brief Compute the time intervial for each loop.
     * @param tick: real robot state used for compution time for each loop.
     * @return time intervial for each loop.
     */
    float ComputeDeltaTime(uint32_t tick);

    /**
     * @brief Update the pose of robot.
     * @param currentTime: time since the timer started.
     */
    void Update(float currentTime);

    /** 
     * @brief Caculate position of foot toe in control frame, then represent this vector in world frame,
     * is not absolute height of body in world frame.
     * @return estimated robot height.
    */
    float EstimateRobotHeight();

    /**
     * @brief Simply compute the odometry and update the robot pose(x,y,yaw).
     * @param deltaTime:time intervial for each loop.
     */
    void ComputePose(float deltaTime);

    /**
     * @brief Getter method of member estimatedPose.
     */
    const Vec6<float> &GetEstimatedPose() const {
        return estimatedPose;
    };

private:

    /**
     * @brief The robot class for pose estimation.
     */
    qrRobot *robot;

    /**
     * @brief Last time stamp when one loop finished.
     */
    uint32_t lastTimestamp;

    /**
     * @brief Esitimaed pose.
     */
    Vec6<float> estimatedPose;

    /**
     * @brief Generate desired gait schedule for locomotion.
     */
    qrGaitGenerator *gaitGenerator;

    /**
     * @brief Estimate the 3D plane where the feet contact.
     */
    qrGroundSurfaceEstimator *groundEstimator;

    /**
     * @brief Velocity estimatior class for pose estimation.
     */
    qrRobotVelocityEstimator *velocityEstimator;

};

} // Namespace Quadruped

#endif // QR_ROBOT_POSE_ESTIMATOR_H

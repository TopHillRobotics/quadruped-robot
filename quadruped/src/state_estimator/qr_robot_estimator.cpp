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

#include "state_estimator/qr_robot_estimator.h"

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

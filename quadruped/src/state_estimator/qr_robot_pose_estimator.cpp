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

#include "state_estimator/qr_robot_pose_estimator.h"

qrRobotPoseEstimator::qrRobotPoseEstimator(qrRobot *robotIn,
                                        qrGaitGenerator *gaitGeneratorIn,
                                        qrGroundSurfaceEstimator *groundEstimatorIn,
                                        qrRobotVelocityEstimator *velocityEstimatorIn)
    : robot(robotIn), gaitGenerator(gaitGeneratorIn), groundEstimator(groundEstimatorIn), velocityEstimator(velocityEstimatorIn)
{
    lastTimestamp = 0.f;
    estimatedPose << robot->GetBasePosition(), robot->GetBaseRollPitchYaw();
}

void qrRobotPoseEstimator::Reset(float currentTime)
{
    lastTimestamp = 0.f;
    estimatedPose << robot->GetBasePosition(), robot->GetBaseRollPitchYaw();
}

float qrRobotPoseEstimator::ComputeDeltaTime(const LowState *robotState)
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

void qrRobotPoseEstimator::Update(float currentTime)
{
    const qrRobotState &robotState = robot->state;
    const LowState state = robot->lowstate;
    // Propagate current state estimate with new accelerometer reading."""
    float deltaTime = ComputeDeltaTime(&state);
    float height = EstimateRobotHeight();
    estimatedPose[2] = height;
    robot->state.basePosition[2] = height;
    ComputePose(deltaTime);
    // case 2 : in simulation case
    if (robot->config->isSim) {
        estimatedPose[0] = robot->gazeboBasePosition[0];
        estimatedPose[1] = robot->gazeboBasePosition[1];
        // estimatedPose[2] = robot->gazeboBasePosition[2];
        robot->state.basePosition[0] = estimatedPose[0];
        robot->state.basePosition[1] = estimatedPose[1];
        // robot->basePosition[2] = estimatedPose[2];
    }
}

float qrRobotPoseEstimator::EstimateRobotHeight()
{
    Quat<float> baseOrientation;
    Eigen::Matrix<float, 3, 3> rotMat;
    Eigen::Matrix<float, 3, 4> footPositions;
    Eigen::Matrix<float, 3, 4> footPositionsWorldFrame, footPositionsControlFrame;
    Vec4<float> usefulHeights;

    Eigen::Matrix<int, 1, 4> contacts;
    for (int legId = 0; legId < 4; legId++) {
        int desLegState = gaitGenerator->desiredLegState[legId];
        bool flag = (desLegState == LegState::STANCE);
        if (flag) {
            contacts[legId] = true;
        } else {
            contacts[legId] = false;
        }
    }
    if (contacts.sum() == 0) {
        // All foot in air, no way to estimate
        return robot->config->bodyHeight;
    } else {
        baseOrientation = robot->GetBaseOrientation();
        rotMat = math::quaternionToRotationMatrix(baseOrientation).transpose();
        footPositions = robot->state.GetFootPositionsInBaseFrame();
        footPositionsWorldFrame = rotMat * footPositions;
        Mat3<float> groundOrientationMat = groundEstimator->GetAlignedDirections();
        footPositionsControlFrame = groundOrientationMat.transpose() * footPositionsWorldFrame;
        Vec4<float> heightsInControlFrame = -footPositionsControlFrame.block<1, 4>(2, 0).cwiseProduct(contacts.cast<float>());
        robot->heightInControlFrame = heightsInControlFrame.sum() / contacts.sum();
        usefulHeights = -footPositionsWorldFrame.block<1, 4>(2, 0).cwiseProduct(contacts.cast<float>());
        return usefulHeights.sum() / contacts.sum();
    }
}

void qrRobotPoseEstimator::ComputePose(float deltaTime)
{
    // currentTime = 0;//ros::Time::now();
    const Vec3<float> &estimatedVelocity = velocityEstimator->GetEstimatedVelocity();
    const Vec3<float> &baseRollPitchYawRate = velocityEstimator->GetEstimatedAngularVelocity();
    const Vec3<float> &baseRollPitchYaw = robot->GetBaseRollPitchYaw();
    float vX = estimatedVelocity[0];
    float vY = estimatedVelocity[1];
    float vTheta = baseRollPitchYawRate[2];
    float x = estimatedPose[0];
    float y = estimatedPose[1];
    float theta = estimatedPose[5];
    // compute odometry in a typical way given the velocities of the robot
    float deltaT = deltaTime; //(currentTime - lastTime).toSec();
    float deltaX = (vX * cos(theta) - vY * sin(theta)) * deltaT;
    float deltaY = (vX * sin(theta) + vY * cos(theta)) * deltaT;
    float deltaTheta = vTheta * deltaT;

    x += deltaX * 1.1f; // sensor error, 1.1 is empirical factor
    y += deltaY * 1.1f;
    theta += deltaTheta;
    // update robot position
    estimatedPose[0] = x;
    estimatedPose[1] = y;
    estimatedPose[5] = theta;

    robot->state.basePosition[0] = x;
    robot->state.basePosition[1] = y;
}

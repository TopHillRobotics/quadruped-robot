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

#include "estimators/qr_robot_pose_estimator.h"


namespace Quadruped {

qrRobotPoseEstimator::qrRobotPoseEstimator(
    qrRobot *robotIn,
    qrGaitGenerator *gaitGeneratorIn,
    qrGroundSurfaceEstimator *groundEstimatorIn,
    qrRobotVelocityEstimator *velocityEstimatorIn):

    robot(robotIn),
    gaitGenerator(gaitGeneratorIn),
    groundEstimator(groundEstimatorIn),
    velocityEstimator(velocityEstimatorIn)
{
    lastTimestamp = 0;
    estimatedPose << robot->GetBasePosition(), robot->GetBaseRollPitchYaw();
}


void qrRobotPoseEstimator::Reset(float currentTime)
{
    lastTimestamp = 0;
    estimatedPose << robot->GetBasePosition(), robot->GetBaseRollPitchYaw();
    robot->stateDataFlow.heightInControlFrame = robot->bodyHeight;
}


float qrRobotPoseEstimator::ComputeDeltaTime(uint32_t tick)
{
    float deltaTime;
    if ((double)lastTimestamp < 1e-5) {
        /* First timestamp received, return an estimated delta_time. */
        deltaTime = robot->timeStep;
    } else {
        deltaTime = (tick - lastTimestamp) / 1000.;
    }
    lastTimestamp = tick;
    return deltaTime;
}


void qrRobotPoseEstimator::Update(float currentTime)
{
    uint32_t tick = robot->GetTick();
    // Propagate current state estimate with new accelerometer reading.
    float deltaTime = ComputeDeltaTime(tick);
    float height = EstimateRobotHeight();
    estimatedPose[2] = height;
    robot->basePosition[2] = height;
    ComputePose(deltaTime);
    
    // case 2 : in simulation case, // todo
    if (robot->isSim) {
        estimatedPose[0] = robot->gazeboBasePosition[0];
        estimatedPose[1] = robot->gazeboBasePosition[1];
        // estimatedPose[2] = robot->gazeboBasePosition[2];
        robot->basePosition[0] = estimatedPose[0];
        robot->basePosition[1] = estimatedPose[1];
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

    /* update desired contact state with desired state */
    Eigen::Matrix<int, 1, 4> contacts;
    for (int legId = 0; legId < 4; legId++) {
        int desLegState = gaitGenerator->desiredLegState[legId];
        int legState = gaitGenerator->detectedLegState[legId];
        bool flag;
        if (robot->controlParams["mode"] == LocomotionMode::WALK_LOCOMOTION) {
            flag = (desLegState!=SubLegState::TRUE_SWING 
                    || desLegState == LegState::STANCE
                    || (desLegState==SubLegState::TRUE_SWING && legState == LegState::EARLY_CONTACT));
        } else {
            flag = (desLegState == LegState::STANCE);
        }
        if (flag) {
            contacts[legId] = true;
        } else {
            contacts[legId] = false;
        }
    }

    if (contacts.sum() == 0) {
        // All foot in air, no way to estimate
        return robot->bodyHeight;
    } else {
        /* compute robot height by averaging the height of feet position */
        baseOrientation = robot->GetBaseOrientation();
        rotMat = robot->stateDataFlow.baseRMat;
        footPositions = robot->GetFootPositionsInBaseFrame();
        footPositionsWorldFrame = rotMat * footPositions;
        Mat3<float> groundOrientationMat = groundEstimator->GetAlignedDirections();
        footPositionsControlFrame = groundOrientationMat.transpose() * footPositionsWorldFrame;
        Vec4<float> heightsInControlFrame = -footPositionsControlFrame.block<1, 4>(2, 0).cwiseProduct(contacts.cast<float>());
        robot->stateDataFlow.heightInControlFrame = heightsInControlFrame.sum() / contacts.sum();
        usefulHeights = -footPositionsWorldFrame.block<1, 4>(2, 0).cwiseProduct(contacts.cast<float>());
        return usefulHeights.sum() / contacts.sum();
    }
}


void qrRobotPoseEstimator::ComputePose(float deltaTime)
{
    const Vec3<float> &estimatedVelocity = velocityEstimator->GetEstimatedVelocity();
    const Vec3<float> &baseRollPitchYawRate = velocityEstimator->GetEstimatedAngularVelocity();
    const Vec3<float> &baseRollPitchYaw = robot->GetBaseRollPitchYaw();
    float vX = estimatedVelocity[0];
    float vY = estimatedVelocity[1];
    float vZ = estimatedVelocity[2];
    float vTheta = baseRollPitchYawRate[2];
    float x = estimatedPose[0];
    float y = estimatedPose[1];
    float theta = estimatedPose[5];
     /* compute odometry in a typical way given the velocities of the robot */
    float deltaT = deltaTime; //(currentTime - lastTime).toSec();
    float deltaX = (vX * cos(theta) - vY * sin(theta)) * deltaT;
    float deltaY = (vX * sin(theta) + vY * cos(theta)) * deltaT;
    float deltaTheta = vTheta * deltaT;

    x += deltaX;// * 1.1f; // sensor error, 1.1 is empirical factor
    y += deltaY;// * 1.1f;
    robot->absoluteHight += vZ*deltaT;
    theta += deltaTheta;
    /* update robot position */
    estimatedPose[0] = x;
    estimatedPose[1] = y;
    estimatedPose[5] = theta;

    robot->basePosition[0] = x;
    robot->basePosition[1] = y;
}

} // Namespace Quadruped

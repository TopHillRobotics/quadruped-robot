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

#include "estimators/qr_robot_velocity_estimator.h"

namespace Quadruped {

qrRobotVelocityEstimator::qrRobotVelocityEstimator(
    qrRobot *robotIn,
    qrGaitGenerator *gaitGeneratorIn,
    qrUserParameters *userParametersIn):

    robot(robotIn),
    gaitGenerator(gaitGeneratorIn)
{
    initialVariance = userParametersIn->initialVariance;
    lastTimestamp = 0;
    estimatedVelocity << 0.f, 0.f, 0.f;
    estimatedAngularVelocity << 0.f, 0.f, 0.f;
    windowSize = userParametersIn->movingWindowFilterSize;
    filter = new TinyEKF<3,3>(0.f, initialVariance, userParametersIn->accelerometerVariance, userParametersIn->sensorVariance);
    velocityFilterX = qrMovingWindowFilter<double, 1>(windowSize);
    velocityFilterY = qrMovingWindowFilter<double, 1>(windowSize);
    velocityFilterZ = qrMovingWindowFilter<double, 1>(windowSize);
    AccFilter = qrMovingWindowFilter<float, 3>(20);
}


void qrRobotVelocityEstimator::Reset(float currentTime)
{
    velocityFilterX.Reset();
    velocityFilterY.Reset();
    velocityFilterZ.Reset();
    AccFilter.Reset();
    lastTimestamp = 0;
    estimatedVelocity << 0.f, 0.f, 0.f;
    estimatedAngularVelocity << 0.f, 0.f, 0.f;
}


float qrRobotVelocityEstimator::ComputeDeltaTime(uint32_t tick)
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


void qrRobotVelocityEstimator::Update(float currentTime)
{
    Vec3<float> filtedAcc = AccFilter.CalculateAverage(robot->stateDataFlow.baseLinearAcceleration);
    robot->stateDataFlow.baseLinearAcceleration = filtedAcc; // todo

    Vec3<float> rpyRate = robot->GetBaseRollPitchYawRate();
    // Propagate current state estimate with new accelerometer reading."""
    float deltaTime = ComputeDeltaTime(robot->GetTick());
    Vec3<float> sensorAcc = robot->baseAccInBaseFrame; //todo
    Quat<float> baseOrientation = robot->GetBaseOrientation(); // w,x,y,z
    Mat3<float> rotMat = robotics::math::quaternionToRotationMatrix(baseOrientation).transpose();
    Vec3<float> calibratedAcc = rotMat * sensorAcc;
    calibratedAcc[2] -= 9.81;
    double deltaV[3] = {calibratedAcc[0] * deltaTime, calibratedAcc[1] * deltaTime, calibratedAcc[2] * deltaTime};
    // Correct estimation using contact legs
    std::vector<Vec3<float>> observedVelocities;
    const Vec4<bool>& footContact = robot->GetFootContact();
    Eigen::Matrix<float,3,4> footPInBaseFrame = robot->GetFootPositionsInBaseFrame();
    Eigen::Matrix<float,3,4> footVInBaseFrame = robot->stateDataFlow.footVelocitiesInBaseFrame; // foot relative to body
    /* compute observed foot vleocity in base frame and get its average */
    for (int legId = 0; legId < 4; ++legId) {
        // if (footContact[leg_id] && gaitGenerator->desiredLegState[legId] == LegState::STANCE) {
        if (footContact[legId]) {
            /* W_v_B + R_wb * B_v_BF + cross(W_omega, W_r_BF) = W_v_F = 0;
               corss(R(v1), R(v2)) = R * cross(v1, v2), where R is rotation matrix(det(R)=1) */
            Vec3<float> vB = footVInBaseFrame.col(legId) + robotics::math::vectorToSkewMat(rpyRate)*footPInBaseFrame.col(legId);
            observedVelocities.push_back(-rotMat * vB);
        }
    }
    int num = observedVelocities.size();
    if (num > 0) {
        Vec3<float> MeanObservedVelocity = Vec3<float>::Zero();
        for (auto &v: observedVelocities) {
            MeanObservedVelocity += v;    // std::mean(observedVelocities);
        }
        MeanObservedVelocity /= num;
        // filter.Update(deltaTime, MeanObservedVelocity);
        /* the fitler is kalman filter, the deltav is a variable for state stransition function,
                   while MeanObservedVelocity is the observed quantity, then the KF fuse them*/
        double z[3] = {MeanObservedVelocity[0], MeanObservedVelocity[1], MeanObservedVelocity[2]};
        filter->step(deltaV, z);
    } else {
        double z[3] = {estimatedVelocity[0], estimatedVelocity[1], estimatedVelocity[2]};
        filter->step(deltaV, z);
    }
     /* the results of KF will move into moving window algorithm for smooth*/
    float x[3] = {(float)filter->getX(0), (float)filter->getX(1), (float)filter->getX(2)};
    float vx = velocityFilterX.CalculateAverage(x[0]);
    float vy = velocityFilterY.CalculateAverage(x[1]);
    float vz = velocityFilterZ.CalculateAverage(x[2]);
    estimatedVelocity << vx, vy, vz; // world frame
    robot->stateDataFlow.baseVInWorldFrame = estimatedVelocity;
    estimatedVelocity = rotMat.transpose() * estimatedVelocity; // base frame
    estimatedAngularVelocity = robot->GetBaseRollPitchYawRate(); // base frame
    robot->stateDataFlow.baseWInWorldFrame = rotMat*estimatedAngularVelocity;
    robot->baseVelocityInBaseFrame = estimatedVelocity;
}

} // Namespace Quadruped


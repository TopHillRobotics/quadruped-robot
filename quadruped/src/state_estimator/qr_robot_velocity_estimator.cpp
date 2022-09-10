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

#include "state_estimator/qr_robot_velocity_estimator.h"

qrRobotVelocityEstimator::qrRobotVelocityEstimator(qrRobot *robotIn,
                                                qrGaitGenerator *gaitGeneratorIn,
                                                float accelerometerVarianceIn,
                                                float sensorVarianceIn,
                                                float initialVarianceIn,
                                                int movingWindowFilterSizeIn)
{
    robot = robotIn;
    gaitGenerator = gaitGeneratorIn;
    initialVariance = initialVarianceIn;
    lastTimestamp = 0.f;
    estimatedVelocity << 0.f, 0.f, 0.f;
    estimatedAngularVelocity << 0.f, 0.f, 0.f;
    windowSize = movingWindowFilterSizeIn;
    filter = new TinyEKF(0.f, initialVariance, accelerometerVarianceIn, sensorVarianceIn);
    velocityFilterX = qrMovingWindowFilter(windowSize);
    velocityFilterY = qrMovingWindowFilter(windowSize);
    velocityFilterZ = qrMovingWindowFilter(windowSize);
}

void qrRobotVelocityEstimator::Reset(float currentTime)
{
    // filter->Reset(0., initialVariance);
    velocityFilterX = qrMovingWindowFilter(windowSize);
    velocityFilterY = qrMovingWindowFilter(windowSize);
    velocityFilterZ = qrMovingWindowFilter(windowSize);

    lastTimestamp = 0.f;
    estimatedVelocity << 0.f, 0.f, 0.f;
    estimatedAngularVelocity << 0.f, 0.f, 0.f;
}

float qrRobotVelocityEstimator::ComputeDeltaTime(const LowState *robotState)
{
    float deltaTime = 0.001;
    if (std::abs(lastTimestamp) < 1e-5) {
        // First timestamp received, return an estimated delta_time.
        deltaTime = robot->timeStep;
    } else {
        deltaTime = (robotState->tick - lastTimestamp) / 1000.;
    }
    lastTimestamp = robotState->tick;
    return deltaTime;
}

void qrRobotVelocityEstimator::Update(float currentTime)
{
    const qrRobotState &state = robot->state;
    // Propagate current state estimate with new accelerometer reading."""
    const LowState lowstate = robot->lowstate;
    float deltaTime = ComputeDeltaTime(&lowstate);
    const auto &acc = state.imu.accelerometer;
    Vec3<float> sensorAcc(acc[0], acc[1], acc[2]);
    Quat<float> baseOrientation = robot->GetBaseOrientation(); // w,x,y,z
    Mat3<float> rotMat = math::quaternionToRotationMatrix(baseOrientation).transpose(); // todo
    Vec3<float> calibratedAcc = rotMat * sensorAcc;
    calibratedAcc[2] -= 9.81;
    double deltaV[3] = {calibratedAcc[0] * deltaTime, calibratedAcc[1] * deltaTime, calibratedAcc[2] * deltaTime};
    // filter.Predict(deltaTime, calibratedAcc * deltaTime);

    // Correct estimation using contact legs
    std::vector<Vec3<float>> observedVelocities;
    auto footContact(robot->GetFootContacts());
    for (int leg_id = 0; leg_id < 4; ++leg_id) {
        if (footContact[leg_id]) {
            Mat3<float> jacobian = robot->state.ComputeJacobian(leg_id);
            // Only pick the jacobian related to joint motors
            Vec3<float> jointVelocities = robot->GetMotorVelocities().segment(leg_id * 3, 3);
            Vec3<float> legVelocityInBaseFrame = -(jacobian * jointVelocities);
            observedVelocities.push_back(rotMat * legVelocityInBaseFrame);
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
        double z[3] = {MeanObservedVelocity[0], MeanObservedVelocity[1], MeanObservedVelocity[2]};
        filter->step(deltaV, z);
    } else {
        double z[3] = {estimatedVelocity[0], estimatedVelocity[1], estimatedVelocity[2]};
        filter->step(deltaV, z);
    }
    float x[3] = {(float)filter->getX(0), (float)filter->getX(1), (float)filter->getX(2)};
    float vx = velocityFilterX.CalculateAverage(x[0]);
    float vy = velocityFilterY.CalculateAverage(x[1]);
    float vz = velocityFilterZ.CalculateAverage(x[2]);
    estimatedVelocity << vx, vy, vz; // world frame
    estimatedVelocity = rotMat.transpose() * estimatedVelocity; // base frame
    estimatedAngularVelocity = robot->GetBaseRollPitchYawRate();
    robot->state.baseVelocity = estimatedVelocity;
}


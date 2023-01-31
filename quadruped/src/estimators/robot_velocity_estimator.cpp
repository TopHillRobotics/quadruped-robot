/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhu Yijie
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*         modify the relationship between base velocity in world frame and motor velocities. @ Zhu Yijie 2022.05.12
*/

#include "estimators/robot_velocity_estimator.h"

namespace Quadruped {
    RobotVelocityEstimator::RobotVelocityEstimator(Robot *robotIn,
                                                   GaitGenerator *gaitGeneratorIn,
                                                   UserParameters *userParametersIn)
        : robot(robotIn), gaitGenerator(gaitGeneratorIn)
    {
        initialVariance = userParametersIn->initialVariance;
        lastTimestamp = 0.f;
        estimatedVelocity << 0.f, 0.f, 0.f;
        estimatedAngularVelocity << 0.f, 0.f, 0.f;
        windowSize = userParametersIn->movingWindowFilterSize;
        filter = new TinyEKF<3,3>(0.f, initialVariance, userParametersIn->accelerometerVariance, userParametersIn->sensorVariance);
        velocityFilterX = MovingWindowFilter<double, 1>(windowSize);
        velocityFilterY = MovingWindowFilter<double, 1>(windowSize);
        velocityFilterZ = MovingWindowFilter<double, 1>(windowSize);
        AccFilter = MovingWindowFilter<float, 3>(20);
    }

    void RobotVelocityEstimator::Reset(float currentTime)
    {
        // filter->Reset(0., initialVariance);
        velocityFilterX.Reset();
        velocityFilterY.Reset();
        velocityFilterZ.Reset();
        AccFilter.Reset();
        lastTimestamp = 0.f;
        estimatedVelocity << 0.f, 0.f, 0.f;
        estimatedAngularVelocity << 0.f, 0.f, 0.f;
        // robot->stateDataFlow.baseVInWorldFrame
        // robot->stateDataFlow.baseWInWorldFrame
        // robot->baseVelocityInBaseFrame
    }

    float RobotVelocityEstimator::ComputeDeltaTime(const LowState *robotState)
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

    void RobotVelocityEstimator::Update(float currentTime)
    {
        Vec3<float> filtedAcc = AccFilter.CalculateAverage(robot->stateDataFlow.baseLinearAcceleration);
        robot->stateDataFlow.baseLinearAcceleration = filtedAcc; // todo
        
        Vec3<float> rpyRate = robot->GetBaseRollPitchYawRate(); 
        const LowState &robotState = robot->lowState;
        // Propagate current state estimate with new accelerometer reading."""
        float deltaTime = ComputeDeltaTime(&robotState);
        const auto &acc = robotState.imu.accelerometer;
        Vec3<float> sensorAcc(acc[0], acc[1], acc[2]);
        Quat<float> baseOrientation = robot->GetBaseOrientation(); // w,x,y,z
        Mat3<float> rotMat = robotics::math::quaternionToRotationMatrix(baseOrientation).transpose();
        Vec3<float> calibratedAcc = rotMat * sensorAcc;
        calibratedAcc[2] -= 9.81;
        double deltaV[3] = {calibratedAcc[0] * deltaTime, calibratedAcc[1] * deltaTime, calibratedAcc[2] * deltaTime};
        // Correct estimation using contact legs
        std::vector<Vec3<float>> observedVelocities;
        const Vec4<bool>& footContact = robot->GetFootContacts();
        Eigen::Matrix<float,3,4> footPInBaseFrame = robot->GetFootPositionsInBaseFrame();
        Eigen::Matrix<float,3,4> footVInBaseFrame = robot->stateDataFlow.footVelocitiesInBaseFrame; // foot relative to body
        for (int legId = 0; legId < 4; ++legId) {
            // if (footContact[leg_id] && gaitGenerator->desiredLegState[legId] == LegState::STANCE) {
            if (footContact[legId]) {
                // W_v_B + R_wb * B_v_BF + cross(W_omega, W_r_BF) = W_v_F = 0;
                // corss(R(v1), R(v2)) = R * cross(v1, v2), where R is rotation matrix(det(R)=1)
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
        robot->stateDataFlow.baseVInWorldFrame = estimatedVelocity;
        estimatedVelocity = rotMat.transpose() * estimatedVelocity; // base frame
        estimatedAngularVelocity = robot->GetBaseRollPitchYawRate(); // base frame
        robot->stateDataFlow.baseWInWorldFrame = rotMat*estimatedAngularVelocity;
        robot->baseVelocityInBaseFrame = estimatedVelocity;
    }

} //namespace Quadruped


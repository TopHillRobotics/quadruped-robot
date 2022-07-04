#include "estimator/qr_robot_velocity_estimator.h"
#include "common/qr_se3.h"
#include "common/qr_math.h"

qrRobotVelocityEstimator::qrRobotVelocityEstimator(
    qrRobot *robot, float accelerometerVariance, float sensorVariance, unsigned int filerWindowSize)
{
    this->robotState  = robot->GetRobotState();
    this->robotConfig = robot->GetRobotConfig();
    estimatedVelocity << 0.f, 0.f, 0.f;
    velocityFilterX = qrMovingWindowFilter(filerWindowSize);
    velocityFilterY = qrMovingWindowFilter(filerWindowSize);
    velocityFilterZ = qrMovingWindowFilter(filerWindowSize);
    filter = new qrTinyEKF(0.f, accelerometerVariance, sensorVariance);
}

void qrRobotVelocityEstimator::Reset(unsigned int windowSize)
{
    velocityFilterX = qrMovingWindowFilter(windowSize);
    velocityFilterY = qrMovingWindowFilter(windowSize);
    velocityFilterZ = qrMovingWindowFilter(windowSize);
    estimatedVelocity << 0.f, 0.f, 0.f;
}

void qrRobotVelocityEstimator::Estimate()
{
    const Vec3<float> acc = robotState->imu.acc;
    Mat3<float> rotMat = math::Rpy2RotMat(robotState->imu.rpy);
    Vec3<float> calibratedAcc = rotMat * acc;
    // TODO: discuss this
    calibratedAcc[2] -= 9.81f;
    Vec3<float> deltaV = robotState->deltaTime * calibratedAcc;

    std::vector<Vec3<float>> observedVelocities;
    for(unsigned int legId = 0; legId < qrRobotConfig::numLegs; legId++){
      if(robotState->GetFootContact()[legId]){
        Vec3<float> jointVelocityInBaseFrame =
            -robotConfig->JointVelocity2FootVelocity(robotState->GetQ(legId), robotState->GetDq(legId), int(legId));
        observedVelocities.push_back(rotMat.transpose() * jointVelocityInBaseFrame);// in world frame
      }
    }

    if(observedVelocities.size() > 0){
      Vec3<float> z = mean(observedVelocities);
      filter->step(deltaV, z);
    } else {
      filter->step(deltaV, estimatedVelocity);// if no contact, then the speed is last speed
    }
    estimatedVelocity = movingWindowUpdate(float(filter->getX(0)), float(filter->getX(1)), float(filter->getX(2)));
}

Vec3<float> qrRobotVelocityEstimator::movingWindowUpdate(float x, float y, float z)
{
    float vx = velocityFilterX.Average(x);
    float vy = velocityFilterY.Average(y);
    float vz = velocityFilterZ.Average(z);
    Vec3<float> v;
    v << vx, vy, vz;
    return v;
}

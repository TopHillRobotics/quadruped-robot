#include "estimator/qr_robot_velocity_estimator.h"
#include "common/qr_se3.h"
#include "common/qr_math.h"

RobotVelocityEstimator::RobotVelocityEstimator(
    qrRobot *robot, float accelerometerVariance, float sensorVariance, unsigned int filerWindowSize)
{
  this->robotState  = robot->getRobotState();
  this->robotConfig = robot->getRobotConfig();
  estimatedVelocity << 0.f, 0.f, 0.f;
  velocityFilterX = qrMovingWindowFilter(filerWindowSize);
  velocityFilterY = qrMovingWindowFilter(filerWindowSize);
  velocityFilterZ = qrMovingWindowFilter(filerWindowSize);
  filter = new qrTinyEKF(0.f, accelerometerVariance, sensorVariance);
}

void RobotVelocityEstimator::Reset(unsigned int windowSize)
{
  velocityFilterX = qrMovingWindowFilter(windowSize);
  velocityFilterY = qrMovingWindowFilter(windowSize);
  velocityFilterZ = qrMovingWindowFilter(windowSize);
  estimatedVelocity << 0.f, 0.f, 0.f;
}

void RobotVelocityEstimator::Estimate()
{
  const Eigen::Matrix<float, 3, 1> acc = robotState->imu.acc;
  Eigen::Matrix<float, 3, 3> rotMat = Math::rpy2RotMat(robotState->imu.rpy);
  Eigen::Matrix<float, 3, 1> calibratedAcc = rotMat * acc;
  // TODO: discuss this
  calibratedAcc[2] -= 9.81f;
  Eigen::Matrix<float, 3, 1> deltaV = robotState->deltaTime * calibratedAcc;

  std::vector<Eigen::Matrix<float, 3, 1>> observedVelocities;
  for(unsigned int legId = 0; legId < qrRobotConfig::numLegs; legId++){
    if(robotState->footContact[legId]){
      Eigen::Matrix<float, 3, 1> jointVelocityInBaseFrame =
          -robotConfig->JointVelocity2FootVelocity(robotState->getQ(legId), robotState->getDq(legId), int(legId));
      observedVelocities.push_back(rotMat.transpose() * jointVelocityInBaseFrame);// in world frame
    }
  }

  if(observedVelocities.size() > 0){
    Eigen::Matrix<float, 3, 1> z = mean(observedVelocities);
    filter->step(deltaV, z);
  } else {
    filter->step(deltaV, estimatedVelocity);// if no contact, then the speed is last speed
  }
  estimatedVelocity = movingWindowUpdate(float(filter->getX(0)), float(filter->getX(1)), float(filter->getX(2)));
}

Eigen::Matrix<float, 3, 1> RobotVelocityEstimator::movingWindowUpdate(float x, float y, float z)
{
  float vx = velocityFilterX.Average(x);
  float vy = velocityFilterY.Average(y);
  float vz = velocityFilterZ.Average(z);
  Eigen::Matrix<float, 3, 1> v;
  v << vx, vy, vz;
  return v;
}

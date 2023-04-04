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

#include "estimators/qr_robot_estimator.h"

namespace Quadruped {

qrRobotEstimator::qrRobotEstimator(qrRobot *robotIn,
                               qrGaitGenerator *gaitGeneratorIn,
                               qrGroundSurfaceEstimator *groundEstimatorIn,
                               qrUserParameters *userParametersIn):
    robot(robotIn),
    velocityEstimator(robotIn, gaitGeneratorIn, userParametersIn),
    poseEstimator(robotIn, gaitGeneratorIn, groundEstimatorIn, &velocityEstimator)
{
    estimatedVelocity = velocityEstimator.GetEstimatedVelocity();
    estimatedAngularVelocity = velocityEstimator.GetEstimatedAngularVelocity();
    const Vec6<float> &pose = poseEstimator.GetEstimatedPose();
    estimatedPosition = pose.head(3);
    estimatedRPY = pose.tail(3);
    std::cout << "estimatedPosition = " << estimatedPosition.transpose() << std::endl;
    lastTimestamp = 0;
    std::cout << "init robotEstimator finish\n" << std::endl;
    // CMUInitState();
}


void qrRobotEstimator::Reset(float currentTime)
{
    timeSinceReset = currentTime;
    velocityEstimator.Reset(currentTime);
    poseEstimator.Reset(currentTime);
    estimatedVelocity = velocityEstimator.GetEstimatedVelocity();
    lastEstimatedVelocity.setZero();
    estimatedAngularVelocity = velocityEstimator.GetEstimatedAngularVelocity();
    const Vec6<float> &pose = poseEstimator.GetEstimatedPose();
    estimatedPosition = pose.head(3);
    estimatedRPY = pose.tail(3);
    lastTimestamp = 0;
    std::cout << "reset pos= " << estimatedPosition.transpose() << std::endl;
}


float qrRobotEstimator::ComputeDeltaTime(uint32_t tick)
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


void qrRobotEstimator::Update(float currentTime)
{
    velocityEstimator.Update(currentTime);
    poseEstimator.Update(currentTime);
    
    estimatedVelocity = velocityEstimator.GetEstimatedVelocity();
    estimatedAngularVelocity = velocityEstimator.GetEstimatedAngularVelocity();
    const Vec6<float> &pose = poseEstimator.GetEstimatedPose();
    estimatedPosition = pose.head(3);
    estimatedRPY = pose.tail(3);
    ComputeZMP();

}


/** @brief Cart-On-Table Model: x_zmp = x_m-(z_m*dtdt_x_m)/(dtdt_z_m+g) */
Vec3<float> qrRobotEstimator::ComputeZMP()
{
    Vec3<float> basePos = robot->GetBasePosition();
    Vec3<float>& baseAcc = robot->stateDataFlow.baseLinearAcceleration;
    float filterF = 0.5;
    baseAcc = filterF*baseAcc + 1000* (1-filterF) * (estimatedVelocity- lastEstimatedVelocity);
    robot->stateDataFlow.zmp[0] = basePos[0] - basePos[2]*baseAcc[0]/(baseAcc[2]);
    robot->stateDataFlow.zmp[1] = basePos[1] - basePos[2]*baseAcc[1]/(baseAcc[2]); // baseAcc[2] receive from IMU already contains g value.
    lastEstimatedVelocity = estimatedVelocity;
    return robot->stateDataFlow.zmp;
}


void qrRobotEstimator::CMUInitState()
{
    // constructor
    eye3.setIdentity();
    // C is fixed
    C.setZero();
    for (int i = 0; i < NumLeg; ++i) {
        C.block<3,3>(i*3,0) = -eye3;     // -pos
        C.block<3,3>(i*3,6+i*3) = eye3;  //foot pos
        C.block<3,3>(NumLeg*3+i*3,3) = eye3;  // vel
        C(NumLeg*6+i,6+i*3+2) = 1;  // height z of foot
    }

    /* Q R are fixed */
    Q.setIdentity();
    /* position transition */
    Q.block<3,3>(0,0) = 0.01*eye3;
    /* velocity transition */
    Q.block<3,3>(3,3) = 0.01*eye3;
    for (int i = 0; i < NumLeg; ++i) {
        Q.block<3,3>(6+i*3,6+i*3) = PROCESS_NOISE_PFOOT*eye3;  // foot position transition
    }
    R.setIdentity();
    for (int i = 0; i < NumLeg; ++i) {
        R.block<3,3>(i*3,i*3) = SENSOR_NOISE_PIMU_REL_FOOT*eye3;                     // fk estimation
        R.block<3,3>(NumLeg*3+i*3,NumLeg*3+i*3) = SENSOR_NOISE_VIMU_REL_FOOT*eye3;   // vel estimation
        R(NumLeg*6+i,NumLeg*6+i) = SENSOR_NOISE_ZFOOT;                               // height z estimation
    }

    A.setIdentity();
    B.setZero();

    /* change R according to this flag, if we do not assume the robot moves on flat ground,
     then we cannot infer height z using this way */
    if (assume_flat_ground == false) {
        for (int i = 0; i < NumLeg; ++i) {
            R(NumLeg*6+i,NumLeg*6+i) = 1e5; // height z estimation not reliable
        }
    }

    P.setIdentity();
    P = P * 3;

    /* set initial value of x */
    x.setZero();
    x.segment<3>(0) = Eigen::Vector3d(0, 0, 0.27);
    Mat3<double> root_rot_mat = robot->stateDataFlow.baseRMat.cast<double>();
    Eigen::Matrix<double,3,4> foot_pos_rel = robot->GetFootPositionsInBaseFrame().cast<double>();
    for (int i = 0; i < NumLeg; ++i) {
        Eigen::Vector3d fk_pos = foot_pos_rel.block<3, 1>(0, i);
        x.segment<3>(6 + i * 3) = root_rot_mat * fk_pos + x.segment<3>(0);
    }
}


/* todo, the paramaters are to be tuned.
 have tested, the velocity amplitude is larger than ours. */
void qrRobotEstimator::CMUUpdate(double dt)
{
    /* update A B using latest dt */
    A.block<3, 3>(0, 3) = dt * eye3;
    B.block<3, 3>(3, 0) = dt * eye3;

    /* control input u is Ra + ag */
    Vec3<double> sensorAcc = robot->stateDataFlow.baseLinearAcceleration.cast<double>();
    Mat3<double> root_rot_mat = robot->stateDataFlow.baseRMat.cast<double>();
    Eigen::Vector3d u = root_rot_mat * sensorAcc + Eigen::Vector3d(0, 0, -9.81);
    Vec4<float> foot_force = robot->GetFootForce();
    // contact estimation, do something very simple first
    // if (state.movement_mode == 0) {  // stand
    //     for (int i = 0; i < NumLeg; ++i) estimated_contacts[i] = 1.0;
    // } else {  // walk
    /* probability of the contact state
         * when the foot force surpass 80 N, the estimated state is contact */
        for (int i = 0; i < NumLeg; ++i) {
            estimated_contacts[i] = std::min(std::max((foot_force(i) + 20.0) / (100.0 - 0.0), 0.0), 1.0);
        }
    // }

    // update Q
    Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt / 20.0 * eye3; //IMU P 
    Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * 9.8 / 20.0 * eye3; // IMU V
     /* update Q R for legs not in contact */
    for (int i = 0; i < NumLeg; ++i) {
        Q.block<3, 3>(6 + i * 3, 6 + i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3)
                                                    * dt * PROCESS_NOISE_PFOOT * eye3;  // foot position transition
        // for estimated_contacts[i] == 1, Q = 0.002
        // for estimated_contacts[i] == 0, Q = 1001*Q

        R.block<3, 3>(i * 3, i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_PIMU_REL_FOOT * eye3;    // fk estimation
        R.block<3, 3>(NumLeg * 3 + i * 3, NumLeg * 3 + i * 3)
            = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_VIMU_REL_FOOT * eye3; // vel estimation
        if (assume_flat_ground) {
            R(NumLeg * 6 + i, NumLeg * 6 + i)
                = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_ZFOOT;       // height z estimation
        }
    }

    // process update
    /* A and B are translate matrixs */
    xbar = A*x + B*u ;

    /* Pbar means state predict covariance martrix */
    Pbar = A * P * A.transpose() + Q;

    // measurement construction
    yhat = C*xbar;
    // leg_v = (-J_rf*av-skew(omega)*p_rf);
    // r((i-1)*3+1:(i-1)*3+3) = body_v - R_er*leg_v;
    // actual measurement
    Eigen::Matrix<double,3,4> foot_pos_rel = robot->GetFootPositionsInBaseFrame().cast<double>();
    Eigen::Matrix<double,3,4> foot_vel_rel = robot->stateDataFlow.footVelocitiesInBaseFrame.cast<double>();
    Vec3<double> imu_ang_vel = robot->GetBaseRollPitchYawRate().cast<double>();
    /* update the observation variable in kalman filter */
    for (int i=0; i<NumLeg; ++i) {
        Eigen::Vector3d fk_pos = foot_pos_rel.block<3,1>(0,i); // in base frame
        y.block<3,1>(i*3,0) = root_rot_mat*fk_pos;   // fk estimation
        Eigen::Vector3d leg_v = -foot_vel_rel.block<3,1>(0,i) - robotics::math::vectorToSkewMat(imu_ang_vel)*fk_pos;
        y.block<3,1>(NumLeg*3+i*3,0) =
                (1.0-estimated_contacts[i])*x.segment<3>(3) +  estimated_contacts[i]*root_rot_mat*leg_v;// vel estimation
        y(NumLeg*6+i) = (1.0-estimated_contacts[i])*(x(2)+fk_pos(2)) + estimated_contacts[i]*0;         // height z estimation
    }

    /* update kalman filter parameters */
    S = C * Pbar *C.transpose() + R;
    S = 0.5*(S+S.transpose());

    error_y = y - yhat;
    Serror_y = S.fullPivHouseholderQr().solve(error_y);
    x = xbar + Pbar * C.transpose() * Serror_y;

    SC = S.fullPivHouseholderQr().solve(C);
    P = Pbar - Pbar * C.transpose() * SC * Pbar;
    P = 0.5 * (P + P.transpose());

    // reduce position drift
    if (P.block<2, 2>(0, 0).determinant() > 1e-6) {
        P.block<2, 16>(0, 2).setZero();
        P.block<16, 2>(2, 0).setZero();
        P.block<2, 2>(0, 0) /= 10.0;
    }

    // final step
    // put estimated values back to A1CtrlStates& state
    // for (int i = 0; i < NumLeg; ++i) {
    //     if (estimated_contacts[i] < 0.5) {
    //         robot->footContact[i] = false;
    //     } else {
    //         robot->footContact[i] = true;
    //     }
    // }
    
    // std::cout << x.transpose() <<std::endl;
    // estimatedPosition = x.segment<3>(0).cast<float>();
    // Vec3<double> vInWorldFrame = x.segment<3>(3);
    // estimatedVelocity = (root_rot_mat.transpose()*vInWorldFrame).cast<float>(); // world frame->base frame
    // robot->stateDataFlow.baseVInWorldFrame = vInWorldFrame.cast<float>();
    // robot->basePosition = estimatedPosition;
    // robot->baseVelocityInBaseFrame = estimatedVelocity;
    // estimatedAngularVelocity = imu_ang_vel.cast<float>(); // base frame
    // robot->stateDataFlow.baseWInWorldFrame = (root_rot_mat*imu_ang_vel).cast<float>();
}

} // Namespace Quadruped

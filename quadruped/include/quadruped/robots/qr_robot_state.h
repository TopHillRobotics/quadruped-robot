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

#ifndef QR_ROBOT_STATE_H
#define QR_ROBOT_STATE_H

#include <Eigen/Dense>
#include "common/qr_se3.h"
#include "qr_robot_config.h"


/**
 * @brief The qrIMU struct stores the information of IMU
 */
struct qrIMU{
    /**
     * @brief quaternion of the IMU ( w,x,y,z )
     */
    std::array<float, 4> quaternion;

    /**
     * @brief angular velocity of IMU ( unit: rad/s )
     */
    std::array<float, 3> gyroscope;

    /**
     * @brief accelerometer of IMU ( m/(s2) )
     */
    std::array<float, 3> accelerometer;

    /**
     * @brief euler angle of IMU (rad)
     */
    std::array<float, 3> rpy;
};

/**
 * @brief The qrMotorState struct stores the information of current state of motors
 */
struct qrMotorState
{
    /**
     * @brief current angle (unit: radian)
     */
    float q;

    /**
     * @brief current velocity (unit: radian/second)
     */
    float dq;

    /**
     * @brief current estimated output torque (unit: N.m)
     */
    float tauEst;

    /**
     * @brief override of operator =
     * @param state: state to set equal to
     */
    void operator=(const qrMotorState &state) {
        this->q = state.q;
        this->dq = state.dq;
        this->tauEst = state.tauEst;
    }
};

/**
 * @brief The qrRobotState class stores the current status of the robot
 */
class qrRobotState{
public:

    /**
     * @brief qrRobotState
     */
    qrRobotState();

    /**
     * @brief qrRobotState
     * @param config
     */
    qrRobotState(qrRobotConfig* config);

    /**
     * @brief robot base position in world frame
     */
    Eigen::Matrix<float, 3, 1> basePosition;

    /**
     * @brief robot base orientation in world frame
     */
    Eigen::Matrix<float, 4, 1> baseOrientation;

    /**
     * @brief yaw calibrated robot rpy in world frame
     */
    Eigen::Matrix<float, 3, 1> baseRollPitchYaw;

    /**
     * @brief robot rpy rate in base frame
     */
    Eigen::Matrix<float, 3, 1> baseRollPitchYawRate;

    /**
     * @brief current angle (unit: radian)
     */
    Eigen::Matrix<float, 12, 1> motorAngles;

    Eigen::Matrix<float, 3, 1> baseVelocity;

    /**
     * @brief current velocity (unit: radian/second)
     */
    Eigen::Matrix<float, 12, 1> motorVelocities;

    /**
     * @brief force on foot
     */
    Eigen::Matrix<float, 4, 1> footForce;

    /**
     * @brief foot contact state ( true or false)
     */
    Eigen::Matrix<bool, 4, 1> footContact;


    /**
     * @brief yaw offset
     */
    float yawOffset = 0.f;

    /**
     * @brief robot config for calculation of some states
     */
    qrRobotConfig* config;

    /**
     * @brief imu information
     */
    qrIMU imu;

    /**
     * @brief state of 12 motors
     */
    std::array<qrMotorState, 12> motorState;

    /**
     * @brief update current robot states
     */
    void Update();

    /**
     * @brief set robot config for calculating some of the states
     * @param config: current config
     */
    void SetRobotConfig(qrRobotConfig* config);

    /**
     * @brief get foot positions in base frame
     * @return foot positions in base frame
     */
    Eigen::Matrix<float, 3, 4> GetFootPositionsInBaseFrame();

    /**
     * @brief get foot positions in world frame
     * @return foot positions in world frame
     */
    Eigen::Matrix<float, 3, 4> GetFootPositionsInWorldFrame(
            bool useInput=false, Vec3<float> basePositionIn={0.f,0.f,0.f}, Quat<float> baseOrientationIn={1.f,0.f,0.f,0.f});

    /**
     * @brief get current jacobian of leg legId
     * @param legId: which leg to calculate
     * @return current Jacobian
     */
    Eigen::Matrix<float, 3, 3> ComputeJacobian(int legId);

    /**
     * @brief convert contact force of one leg to joint torque
     * @param legId: which leg to convert
     * @param contractForce: contact force of the leg
     * @return joint torques, totally 3 joints
     */
    std::map<int, float> MapContactForceToJointTorques(int legId, Eigen::Matrix<float, 3, 1> contractForce);

private:

    /**
     * @brief get calibrated yaw
     * @return calibrated yaw
     */
    float CalibrateYaw();
};

#endif // QR_ROBOT_STATE_H

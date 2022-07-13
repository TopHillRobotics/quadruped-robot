// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: Xinyu Zhang   email: tophill.robotics@gmail.com

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

#include <iostream>
#include <array>
#include <Eigen/Dense>

#include "robot/qr_robot_config.h"

enum Joint{
    FRhip, FRthigh, FRcalf, \
    FLhip, FLthigh, FLcalf, \
    RRhip, RRthigh, RRcalf, \
    RLhip, RLthigh, RLcalf
};

// TODO: check initialization
/**
 * @brief The qrIMU struct saves data from imu
 */
struct qrIMU
{

    /**
     * @brief normalized quaternion  (w,x,y,z)
     */
    Vec4<float> quaternion;

    /**
     * @brief angular velocity （unit: rad/s)
     */
    Vec3<float> gyroscope;

    // TODO: discuss
    /**
     * @brief acceleration  (unit: m/(s2) )
     */
    Vec3<float> acc;

    /**
     * @brief euler angle（unit: rad)
     */
    Vec3<float> rpy;

    /**
     * @brief calibrate the yaw when out [-PI, PI]
     * @return rpy with calibrated yaw
     */
    Vec3<float> CalibratedYawRpy(){
        Vec3<float> rpyCalibrate = rpy;
        if(rpyCalibrate(2, 0) >= float(M_PI)){
            rpyCalibrate(2, 0) -= float(2 * M_PI);
        }
        else if (rpyCalibrate(2, 0) <= -float(M_PI)) {
            rpyCalibrate(2, 0) += float(2 * M_PI);
        }
        return rpyCalibrate;
    }

    void operator=(const qrIMU &imu){
        this->quaternion = imu.quaternion;
        this->rpy        = imu.rpy;
        this->gyroscope  = imu.gyroscope;
        this->acc        = imu.acc;
    }

};

/**
 * @brief The qrLegState struct stores observed data every iteration
 */
class qrRobotState
{

public:

    /**
     * @brief constructor of qrRobotState
     */
    qrRobotState();

    /**
     * @brief constructor of qrRobotState
     * @param robotConfig used to calculate current state
     */
    qrRobotState(qrRobotConfig* robotConfig);

    /**
     * @brief imu related date
     */
    qrIMU imu;

    /**
     * @brief current angle (unit: radian)
     */
    Vec12<float> q;

    /**
     * @brief current velocity (unit: radian/second)
     */
    Vec12<float> dq;

    /**
     * @brief current estimated output torque (unit: N.m)
     */
    Vec12<float> tau;

    /**
     * @brief force on foot
     */
    Vec4<float> footForce;

    /**
     * @brief time between this currentStamp and last stamp;
     *        at the first time, delta time will set 0.001 as default
     */
    float deltaTime;

    /**
     * @brief set current time stamp from hardware
     * @param tick: time stamp
     */
    void setTimeStamp(uint32_t tick);

    /**
     * @brief set qrRobotConfig object
     * @param robotConfig
     */
    void SetRobotConfig(qrRobotConfig *robotConfig);

    /**
     * @brief get a vector of motor velocities
     * @param legId: which leg's velocity
     * @return vector of motor velocities
     */
    Vec3<float> GetDq(unsigned int legId);


    /**
     * @brief get a vector of motor velocities
     * @param legId: which leg's velocity
     * @return vector of motor velocities
     */
    Vec3<float> GetQ(unsigned int legId);

    /**
     * @brief return foot position accroding to base frame
     * @return foot position in base frame
     */
    Mat3x4<float> GetFootPositionInBaseFrame();

    /**
     * @brief get roll pitch yall of robot body
     * @return roll, pitch, yaw
     */
    Vec3<float> GetRpy();

    /**
     * @brief get velocity of roll, pitch, yaw
     * @return velocity of roll, pitch, yaw
     */
    Vec3<float> GetDrpy();

    /**
     * @brief return orientation of robot body
     * @return orientation of robot body
     */
    Vec4<float> GetBaseOrientation();

    /**
     * @brief get current jacobian of leg legId
     * @param legId: which leg to calculate
     * @return current Jacobian
     */
    Mat3<float> GetJacobian(int legId);

    /**
     * @brief convert contact force of one leg to joint torque
     * @param contractForce: contact force of the leg
     * @param legId: which leg to convert
     * @return joint torques, totally 3 joints
     */
    Vec3<float> ContactForce2JointTorque(Vec3<float> contractForce, int legId);

    inline Vec4<bool> GetFootContact() const
    {
        for (int footId = 0; footId < qrRobotConfig::numLegs; footId++) {
            if (footForce[footId] > 5) {
                footContact[footId] = true;
            } else {
                footContact[footId] = false;
            }
        }
        return footContact;
    }

    inline Vec3<float> GetBasePosition() const{
        return basePosition;
    }


    void operator=(const qrRobotState &robotState);

private:

    /**
     * @brief the smallest time interval of each loop, which means max frequence is 1000HZ
     */
    static constexpr float leastDeltaTime = 0.001f;

    /**
     * @see robotConfig
     */
    qrRobotConfig* robotConfig;

    /**
     * @brief last time stamp
     */
    uint32_t lastStamp = 0;

    // TODO: get and set
    /**
     * @brief the contact status of 4 foot
     */
    mutable Vec4<bool> footContact;

    // TODO: check this
    Vec3<float> basePosition;
};

#endif // QR_ROBOT_STATE_H

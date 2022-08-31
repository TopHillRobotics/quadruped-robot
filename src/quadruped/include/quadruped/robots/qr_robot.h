// The MIT License

// Copyright (c) 2022
// qrRobot Motion and Vision Laboratory at East China Normal University
// Contact:tophill.robotics@gmail.com

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
#ifndef QR_ROBOT_H
#define QR_ROBOT_H

#include <iostream>
#include <string>
#include <cmath>
#include <unordered_map>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "unitree_legged_sdk/unitree_interface.h"
#include "config.h"
#include "types.h"
#include "robots/qr_timer.h"
#include "robots/qr_motor.h"
#include "common/se3.h"
#include "qr_robot_config.h"
#include "qr_robot_state.h"


/**
 * @brief base class of robot.
 */
class qrRobot {

public:
    /**
     * @brief constructor of qrRobot
     * @param path: path to config file
     */
    qrRobot(std::string path);

    virtual ~qrRobot();

    /**
     * @brief update observation in each loop.
     */
    virtual void ReceiveObservation() = 0;

    /**
     * @brief set and execute commands
     * @param motorCommands: matrix of commands to execute
     * @param motorControlMode: control mode
     */
    virtual void ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode) {};

    /**
     * @brief set and execute commands
     * @param motorCommands: vector of commands to execute
     * @param motorControlMode: control mode
     */
    virtual void ApplyAction(const std::vector<qrMotorCommand> &motorCommands, MotorMode motorControlMode){};

    /**
     * @brief observe and apply action
     * @param action: commands to execute
     * @param motorControlMode: control mode
     */
    virtual void Step(const Eigen::MatrixXf &action, MotorMode motorControlMode) {};

    /**
     * @brief observe and apply action
     * @param motorCommands: commands to execute
     * @param motorControlMode: control mode
     */
    virtual void Step(const std::vector<qrMotorCommand> &motorCommands, MotorMode motorControlMode) {};

    /**
     * @brief get current 12 motor angles
     * @return vector of current motor angles
     */
    inline Eigen::Matrix<float, 12, 1> GetMotorAngles()
    {
        return state.motorAngles;
    }

    /**
     * @brief get current 12 motor velocities
     * @return vector of current motor velocities
     */
    inline Eigen::Matrix<float, 12, 1> GetMotorVelocities() const
    {
        return state.motorVelocities;
    }

    /**
     * @brief Get robot base position in world frame.
     * @return robot base position in world frame.
     */
    inline Eigen::Matrix<float, 3, 1> GetBasePosition() const
    {
        return state.basePosition;
    };

    /**
     * @brief Get robot base orientation in world frame.
     * @return robot base orientation in world frame.
     */
    inline Eigen::Matrix<float, 4, 1> GetBaseOrientation() const
    {
        return state.baseOrientation;
    };

    /**
     * @brief Get robot base rpy in world frame.
     * @return yaw calibrated robot rpy in world frame.
     */
    inline Eigen::Matrix<float, 3, 1> GetBaseRollPitchYaw() const
    {
        return state.baseRollPitchYaw;
    }

    /**
     * @brief Get robot base rpy rate in base frame.
     * @return robot rpy rate in base frame
     */
    inline Eigen::Matrix<float, 3, 1> GetBaseRollPitchYawRate() const
    {
        return state.baseRollPitchYawRate;
    }

    /**
     * @brief get contact state of 4 feet
     * @return vector of foot contact
     */
    inline Eigen::Matrix<bool, 4, 1> GetFootContacts() const
    {
        return state.footContact;
    }

    /**
     * @brief get current time of one step
     * @return time step
     */
    inline float GetTimeStep()
    { return timeStep; }

    /**
     * @brief get robot timer
     * @return timer
     */
    inline Timer &GetTimer()
    { return timer; }

    /**
     * @brief reset the timer
     */
    void ResetTimer()
    {
        timer.ResetStartTime();
    }

    /**
     * @brief get time passed since robot reset
     * @return time after reset
     */
    float GetTimeSinceReset()
    {
        return timer.GetTimeSinceReset();
    }

    /**
     * @brief motor angles after robot stands up
     */
    Eigen::Matrix<float, 12, 1> standUpMotorAngles; // default motor angle when robot stands.

    /**
     * @brief motor angles after robot sits down
     */
    Eigen::Matrix<float, 12, 1> sitDownMotorAngles;

    /**
     * @brief control parameters
     */
    std::map<std::string, int> controlParams;

    /**
     * @brief timer that store time since robot starts
     */
    Timer timer;

    /**
     * @brief control frequence of the robot
     */
    float timeStep;

    /**
     * @brief stores last time that resets robot
     */
    float lastResetTime = GetTimeSinceReset();

    /**
     * @brief store the configuration of the robot
     */
    qrRobotConfig *config;

    /**
     * @brief real robot state used for compution time for each loop
     */
    LowState lowstate;

    /**
     * @brief restore the state of the robot
     */
    qrRobotState state;

    /**
     * @brief whether the robot has stopped
     */
    bool stop = false;

    /**
     * @brief default height in control frame
     */
    float heightInControlFrame = 0.27f;

    /**
     * @brief default base frame position in gazebo
     */
    Eigen::Matrix<float, 3, 1> gazeboBasePosition = {0.f, 0.f, A1_BODY_HIGHT}; //robot base position in world frame

    /**
     * @brief default base frame orientation in gazebo
     */
    Eigen::Matrix<float, 4, 1> gazeboBaseOrientation = {1.f,0.f,0.f,0.f}; //robot base orientation in world frame
    
    // std::unordered_map<int, std::string> modeMap = {{0, "velocity"}, {1, "position"}, {2, "walk"}};
};

#endif //QR_ROBOT_H

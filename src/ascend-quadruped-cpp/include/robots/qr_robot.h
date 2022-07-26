/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Base class of quadruped robots.
* Author: Zhao Yao
* Create: 2021-11-3
* Notes: xx
* Modify: init the file. @ Zhao Yao 2021.11.19
*/

#ifndef QR_ROBOT_H
#define QR_ROBOT_H

#include <iostream>
#include <string>
#include <cmath>
#include <unordered_map>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "config.h"
#include "types.h"
#include "robots/qr_timer.h"
#include "robots/qr_motor.h"
#include "utils/se3.h"
#include "utils/tools.h"
#include "qr_robot_config.h"
#include "qr_robot_state.h"
#include "unitree_legged_sdk/unitree_interface.h"

namespace Quadruped {
    /**
     * @brief base class of robot.
     */
    class qrRobot {

    public:
        qrRobot() = default;

        qrRobot(std::string path);

        virtual ~qrRobot() = default;

        /** @brief update observation in each loop. */
        virtual void ReceiveObservation() = 0;

        virtual void ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode) = 0;

        virtual void ApplyAction(const std::vector<qrMotorCommand> &motorCommands, MotorMode motorControlMode)
        {};

        virtual void Step(const Eigen::MatrixXf &action, MotorMode motorControlMode) = 0;

        virtual void Step(const std::vector<qrMotorCommand> &motorCommands,
                          MotorMode motorControlMode)
        {};

        inline Eigen::Matrix<float, 12, 1> GetMotorAngles()
        {
            return state.motorAngles;
        }

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

        inline Eigen::Matrix<bool, 4, 1> GetFootContacts() const
        {
            return state.footContact;
        }

        inline float GetTimeStep()
        { return timeStep; }

        inline Timer &GetTimer()
        { return timer; }

        void ResetTimer()
        {
            timer.ResetStartTime();
        }

        float GetTimeSinceReset()
        {
            return timer.GetTimeSinceReset();
        }

        Eigen::Matrix<float, 12, 1> standUpMotorAngles; // default motor angle when robot stands.

        Eigen::Matrix<float, 12, 1> sitDownMotorAngles;

        std::map<std::string, int> controlParams;

        Timer timer;
        float timeStep;
        float lastResetTime = GetTimeSinceReset();
        bool initComplete = false;

        RobotConfig *config;
        RobotState  state;

        RobotInterface robotInterface;
        bool stop = false;

        float heightInControlFrame = 0.27f;// ???
        //Note:
        Eigen::Matrix<float, 3, 1> gazeboBasePosition = {0.f, 0.f, A1_BODY_HIGHT}; //robot base position in world frame
        Eigen::Matrix<float, 4, 1> gazeboBaseOrientation = {1.f,0.f,0.f,0.f}; //robot base orientation in world frame
        

        std::unordered_map<int, std::string> modeMap = {{0, "velocity"}, {1, "position"}, {2, "walk"}};
    };
} // namespace Quadruped

#endif //QR_ROBOT_H

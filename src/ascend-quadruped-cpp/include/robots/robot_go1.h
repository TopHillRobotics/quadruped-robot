/* 
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.        
* Description: inherited from quadruped robot, name as GO1.
* Author: Zhu Yijie
* Create: 2021-12-09
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_GO1_ROBOT_H
#define ASCEND_QUADRUPED_CPP_GO1_ROBOT_H

#include "robots/robot.h"

namespace Quadruped {
     /**
     * @brief GO1 robot in reality, it receives observation and supplys sensor data.
     */
    class RobotGO1 : public Robot {
    public:
        RobotGO1(std::string configFilePath);

        virtual ~RobotGO1() = default;

        virtual void ReceiveObservation() override;

        /**
         * @brief
         * @param motorCommands
         * @param motorControlMode
         */
        virtual void ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode) override;

        virtual void ApplyAction(const std::vector<MotorCommand> &motorCommands,
                                 MotorMode motorControlMode);

        virtual void Step(const Eigen::MatrixXf &action, MotorMode motorControlMode) override;

        virtual void Step(const std::vector<MotorCommand> &motorCommands,
                          MotorMode motorControlMode);

        virtual LowState &GetLowState();
        // private:
        // RobotInterface robotInterface;
        // LowState lowState;
        // HighState highState;
        std::vector<IMU> imuDatas;
    };
} // Quadruped

#endif  // ASCEND_QUADRUPED_CPP_GO1_ROBOT_H

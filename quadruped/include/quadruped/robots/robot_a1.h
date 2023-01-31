/* 
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.        
* Description: inherited from quadruped robot, name as A1.
* Author: Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhao Yao
*/

#ifndef ASCEND_QUADRUPED_CPP_A1_ROBOT_H
#define ASCEND_QUADRUPED_CPP_A1_ROBOT_H

#include "robot.h"

namespace Quadruped {
    /**
     * @brief A1 robot in reality, it receives observation and supplys sensor data.
     */
    class RobotA1 : public Robot {

    public:

        RobotA1(std::string configFilePath);

        ~RobotA1() = default;

        void ReceiveObservation();

        /**
         * @brief
         * @param motorCommands
         * @param motorControlMode
         */
        void ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode);

        void ApplyAction(const std::vector<MotorCommand> &motorCommands, MotorMode motorControlMode);

        void Step(const Eigen::MatrixXf &action, MotorMode motorControlMode);
        
        virtual bool BuildDynamicModel();

        virtual void RecordData(int beginId);

        virtual void SaveData(std::string fileName);
        
        // RobotInterface robotInterface;
        // LowState lowState;
        // HighState highState;        
    };
} // namespace Quadruped
#endif //ASCEND_QUADRUPED_CPP_A1_ROBOT_H

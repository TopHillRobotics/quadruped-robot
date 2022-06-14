/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: motor object on robots.
* Author: Zhu Yijie
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie;
*       mv implementation of func to .cpp file. @ Zhu Yijie 2021-11-24;
*/

#ifndef ASCEND_QUADRUPED_CPP_MOTOR_MOTOR_H
#define ASCEND_QUADRUPED_CPP_MOTOR_MOTOR_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

namespace Quadruped {
    /**
     * @brief send motor commond to uitree_sdk
     * @param p double, position, joint angle
     * @param Kp double, position factor
     * @param d double, joint velocity
     * @param Kd double, velocity factor
     * @param tua double, torque
     */
    struct MotorCommand {
        double p;
        double Kp;
        double d;
        double Kd;
        double tua;

        MotorCommand()
        {}

        MotorCommand(double pIn, double KpIn, double dIn, double KdIn, double tuaIn);

        MotorCommand(const Eigen::Matrix<float, 5, 1> &cmd);

        Eigen::Matrix<float, 5, 1> convertToVector() const;

        friend std::ostream &operator<<(std::ostream &os, MotorCommand &data);

        static Eigen::Matrix<float, 5, 12> convertToMatix(const std::vector<MotorCommand> &MotorCommands)
        {
            Eigen::Matrix<float, 5, 12> MotorCommandMatrix;
            int i = 0;
            for (auto &cmd: MotorCommands) {
                MotorCommandMatrix.col(i) = cmd.convertToVector();
                ++i;
            }
            return MotorCommandMatrix;
        }

    };
} // Quadruped

#endif // ASCEND_QUADRUPED_CPP_MOTOR_MOTOR_H

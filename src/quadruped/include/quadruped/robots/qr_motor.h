/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: motor object on robots.
* Author: Zhu Yijie
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie;
*       mv implementation of func to .cpp file. @ Zhu Yijie 2021-11-24;
*/

#ifndef QR_MOTOR_H
#define QR_MOTOR_H

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
    struct qrMotorCommand {

        friend std::ostream &operator<<(std::ostream &os, qrMotorCommand &data);

        /**
         * @brief joint angle (unit: radian)
         */
        double p;

        /**
         * @brief position stiffness (unit: N.m/rad )
         */
        double Kp;

        /**
         * @brief joint velocity ( unit: radian/second)
         */
        double d;

        /**
         * @brief velocity stiffness (unit: N.m/(rad/s) )
         */
        double Kd;

        /**
         * @brief torque (unit: N.m)
         */
        double tua;

        qrMotorCommand(){}

        /**
         * @brief constructor of qrMotorCommand
         * @param pIn: joint angle input
         * @param KpIn: position stiffness input
         * @param dIn: joint velocity input
         * @param KdIn: velocity stiffness input
         * @param tuaIn:torque input
         */
        qrMotorCommand(double pIn, double KpIn, double dIn, double KdIn, double tuaIn);

        /**
         * @brief constructor of qrMotorCommand
         * @param cmd: vector of p, Kp, d, Kd and tau
         */
        qrMotorCommand(const Eigen::Matrix<float, 5, 1> &cmd);

        /**
         * @brief convert p, Kp, d, Kd and tau to Vector
         * @return vector of < p, Kp, d, Kd and tau >
         */
        Eigen::Matrix<float, 5, 1> convertToVector() const;

        /**
         * @brief convert vector of commands to eigen matrix
         * @param MotorCommands: vector of cmds
         * @return command matrix
         */
        static Eigen::Matrix<float, 5, 12> convertToMatix(const std::vector<qrMotorCommand> &MotorCommands)
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

#endif // QR_MOTOR_H

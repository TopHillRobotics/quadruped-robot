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

    double p;
    double Kp;
    double d;
    double Kd;
    double tua;

    qrMotorCommand() = default;

    /**
     * @brief constructor of qrMotorCommand
     * @param pIn: joint angle input
     * @param KpIn: position stiffness input
     * @param dIn: joint velocity input
     * @param KdIn: velocity stiffness input
     * @param tuaIn:torque input
     */
    qrMotorCommand(double pIn, double KpIn, double dIn, double KdIn, double tuaIn):
        p(pIn),
        Kp(KpIn),
        d(dIn),
        Kd(KdIn),
        tua(tuaIn) {
    };

    /**
     * @brief constructor of qrMotorCommand
     * @param cmd: vector of p, Kp, d, Kd and tau
     */
    qrMotorCommand(const Eigen::Matrix<float, 5, 1> &cmd);

    /**
     * @brief set p, Kp, d, Kd and tau to 0
     */
    void SetZero();

    /**
     * @brief convert p, Kp, d, Kd and tau to Vector
     * @return vector of < p, Kp, d, Kd and tau >
     */
    Eigen::Matrix<float, 5, 1> convertToVector() const;

    /**
     * @brief operator reload to output the motor command
     */
    friend std::ostream &operator<<(std::ostream &os, qrMotorCommand &data);

    /**
     * @brief convert vector of commands to eigen matrix
     * @param MotorCommands: vector of cmds
     * @return command matrix
     */
    static Eigen::Matrix<float, 5, 12> convertToMatix(const std::vector<qrMotorCommand> &MotorCommands) {
        Eigen::Matrix<float, 5, 12> MotorCommandMatrix;
        int i = 0;
        for (auto &cmd: MotorCommands) {
            MotorCommandMatrix.col(i) = cmd.convertToVector();
            ++i;
        }
        return MotorCommandMatrix;
    };

};

} // Namespace Quadruped

#endif // QR_MOTOR_H

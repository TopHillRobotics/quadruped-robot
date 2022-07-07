// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
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

#include "robot/qr_motor_cmd.h"


qrMotorCmd::qrMotorCmd()
{
    this->q   = 0.0f;
    this->dq  = 0.0f;
    this->tau = 0.0f;
    this->Kp  = 0.0f;
    this->Kd  = 0.0f;
}

qrMotorCmd::qrMotorCmd(float q, float dq, float tau, float Kp, float Kd)
{
    this->q   = q;
    this->dq  = dq;
    this->tau = tau;
    this->Kp  = Kp;
    this->Kd  = Kd;
}

Eigen::Matrix<float, 5, 1> qrMotorCmd::ToEigenVector() const
{
    Eigen::Matrix<float, 5, 1> cmd;
    cmd << q, Kp, dq, Kd, tau;
    return cmd;
}

std::array<float, 5> qrMotorCmd::ToArray() const
{
    std::array<float, 5> result;
    result[0] = q;
    result[1] = Kp;
    result[2] = dq;
    result[3] = Kd;
    result[4] = tau;
    return result;
}

void qrMotorCmd::SetCmd(float q, float Kp, float dq, float Kd, float tau)
{
    this->q   = q;
    this->Kp  = Kp;
    this->dq  = dq;
    this->Kd  = Kd;
    this->tau = tau;
}

void qrMotorCmd::operator=(const qrMotorCmd &cmd)
{
    this->q   = cmd.q;
    this->dq  = cmd.dq;
    this->Kd  = cmd.Kd;
    this->Kp  = cmd.Kp;
  this->tau = cmd.tau;
}

Eigen::Matrix<float, 5, 12> qrMotorCmd::CmdsToMatrix5x12(const std::vector<qrMotorCmd> &cmds)
{
    Eigen::Matrix<float, 5, 12> cmdMatrix;
    for (unsigned int i = 0; i < 12; i++) {
        cmdMatrix.col(i) = cmds[i].ToEigenVector();
    }
    return cmdMatrix;
}

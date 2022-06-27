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


Eigen::Matrix<float, 5, 1> qrMotorCmd::ToEigenVector()
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

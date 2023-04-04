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

#include "robots/qr_motor.h"
#include <iomanip>


namespace Quadruped {

qrMotorCommand::qrMotorCommand(const Eigen::Matrix<float, 5, 1> &cmd)
{
    p = cmd[0];
    Kp = cmd[1];
    d = cmd[2];
    Kd = cmd[3];
    tua = cmd[4];
}


void qrMotorCommand::SetZero()
{
    p = 0;
    Kp = 0;
    d = 0;
    Kd = 0;
    tua = 0;
}


Eigen::Matrix<float, 5, 1> qrMotorCommand::convertToVector() const
{
    Eigen::Matrix<float, 5, 1> HybridMotorCommandVector;
    HybridMotorCommandVector << p, Kp, d, Kd, tua;
    return HybridMotorCommandVector;
}


std::ostream &operator<<(std::ostream &os, qrMotorCommand &data)
{
    os << std::setprecision(3) << data.p << " " << data.Kp << " " << data.d << " " << data.Kd <<
        " " << data.tua;
    return os;
}

} // Namespace Quadruped

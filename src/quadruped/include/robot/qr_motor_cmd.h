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

#ifndef QR_MOTOR_CMD_H
#define QR_MOTOR_CMD_H

#include <Eigen/Dense>

/**
 * @brief The qrMotorCmd class is used to store the result motor command of one iteration
 */
class qrMotorCmd
{

public:

  /**
   * @brief Constructor of qrMotorCmd
   */
  qrMotorCmd();

  /**
   * @brief Constructor of qrMotorCmd. Initialize with input
   * @param q: angle
   * @param dq: joint velocity
   * @param tau: torque
   * @param Kp: position stiffness (unit: N.m/rad )
   * @param Kd: velocity stiffness (unit: N.m/(rad/s) )
   */
  qrMotorCmd(float q, float dq, float tau, float Kp, float Kd);

  /**
   * @brief Destructor of qrMotorCmd
   */
  ~qrMotorCmd();

  /**
   * @brief convert result to eigen vector
   * @return eigen vector
   */
  Eigen::Matrix<float, 5, 1> ToEigenVector();

  /**
   * @brief convert result to array
   * @return array of motor command
   */
  std::array<float, 5> ToArray() const;

  /**
   * @brief setCmd
   * @param q: joint angle (unit: radian)
   * @param Kp: position stiffness (unit: N.m/rad )
   * @param dq: joint velocity ( unit: radian/second)
   * @param Kd: velocity stiffness (unit: N.m/(rad/s) )
   * @param tau: torque (unit: N.m)
   */
  void SetCmd(float q, float Kp, float dq, float Kd, float tau);

  /**
   * @brief overload  operator =
   * @param cmd: command that need to be equal
   */
  void operator=(const qrMotorCmd &cmd);

private:

  /**
   * @brief joint angle (unit: radian)
   */
  float q;

  /**
   * @brief joint velocity ( unit: radian/second)
   */
  float dq;

  /**
   * @brief torque (unit: N.m)
   */
  float tau;

  /**
   * @brief position stiffness (unit: N.m/rad )
   */
  float Kp;

  /**
   * @brief velocity stiffness (unit: N.m/(rad/s) )
   */
  float Kd;
};

#endif // QR_MOTORCMD_H

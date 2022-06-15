// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: Xinyu Zhang   email: tophill.robotics@gmail.com

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

#ifndef QR_ROBOT_BASE_H
#define QR_ROBOT_BASE_H

#include <vector>
#include "qr_robotconfig.h"
#include "qr_motorcmd.h"

namespace Robot {
    class qrRobotBase;
}


/**
 *  @brief a base class for all robot classes.It stores runtime status and data of the robot.
 */
class qrRobotBase
{

public:

  /**
   *  @brief Constructor of the class
   */
  qrRobotBase() = default;

  /**
   *  @brief Destructor of the class
   */
  virtual ~qrRobotBase() = default;

  /**
   *  @brief Recieve information estimation results.
   */
  virtual void observation()=0;

  /**
   *  @brief Update the state of the robot.
   */
  virtual void update()=0;


  /**
   * @brief get the motor command to be executed
   * @return result of motor command
   */
  virtual std::vector<qrMotorCmd> getCmd()=0;

protected:

  /**
   * @brief stores the static config of the robot
   */
  qrRobotConfig* config = nullptr;

  /**
   * @brief stores the command that will execute at each motor
   */
  std::vector<qrMotorCmd> *cmds = nullptr;
};

#endif // QR_ROBOT_BASE_H

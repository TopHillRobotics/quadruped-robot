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

#ifndef QR_CONTROL_FSM_DATA_H
#define QR_CONTROL_FSM_DATA_H

#include "gait/qr_gait.h"
#include "robots/qr_robot.h"
#include "estimators/qr_state_estimator_container.h"
#include "controllers/qr_desired_state_command.hpp"


/** 
 * @brief ControlFSMData 
 */
template <typename T>
struct qrControlFSMData {

  /**
   * @brief Pointer to Robot.
   */
  Quadruped::qrRobot* quadruped;

  /**
   * @brief Pointer to StateEstimatorContainer.
   */
  Quadruped::qrStateEstimatorContainer* stateEstimators;

  /**
   * @brief Pointer to GaitGenerator.
   */
  Quadruped::qrGaitGenerator* gaitGenerator;

  /**
   * @brief Pointer to DesiredStateCommand
   */
  Quadruped::qrDesiredStateCommand* desiredStateCommand;

  /**
   * @brief Pointer to User Parameters
   */
  qrUserParameters* userParameters;

  /**
   * @brief The motor commands calculated in this control loop.
   */
  std::vector<Quadruped::qrMotorCommand> legCmd;

};

template struct qrControlFSMData<float>;

#endif // QR_CONTROL_FSM_DATA_H

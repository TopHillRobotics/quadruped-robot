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

#ifndef QR_ACTION_H
#define QR_ACTION_H

#include <math.h>
#include <algorithm>
#include "robots/qr_robot.h"
#include "controller/qr_locomotion_controller.h"

/**
 * @brief This file defines some atomic actions, such as stand up and sit down.
 */
namespace Action {
    
    /**
     * @brief stand up action function.
     * @param robot the robot to execute this action.
     * @param standUptime the time that stand up action need to cost.
     * @param totalTime the time that robot start to execute other operation.
     * @param timestep the time that each step cost.
     */
    void StandUp(qrRobot *robot, float standUpTime, float totalTime, float timeStep);

    /**
     * @brief sit down action function.
     * @param robot the robot to execute this action.
     * @param sitDownTime the time that sit down action need to cost.
     * @param timestep the time that each step cost.
     */
    void SitDown(qrRobot *robot, float sitDownTime, float timeStep);

    /**
     * @brief keep Stand action function.
     * @param robot the robot to execute this action.
     * @param KeepStandTime the time that robot keeps standing.
     * @param timestep the time that each step cost.
     */
    void KeepStand(qrRobot *robot, float KeepStandTime = 1.0, float timeStep = 0.001);

    /**
     * @brief control foot action function. This is for walk gait.
     * @param robot the robot to execute this action.
     * @param locomotionController the time that robot keeps standing.
     * @param walkTime the time that control foot action need to cost.
     * @param timestep the time that each step cost.
     */
    void ControlFoot(qrRobot *robot, qrLocomotionController *locomotionController, float walkTime, float timeStep);
    
} // Action
#endif //QR_ACTION_H

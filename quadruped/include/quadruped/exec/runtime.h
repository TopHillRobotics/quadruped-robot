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

#ifndef QR_RUNTIME_H
#define QR_RUNTIME_H

#include <iostream>
#include <typeinfo>
#include <yaml-cpp/yaml.h>


#include "controller/qr_locomotion_controller.h"
#include "planner/qr_com_planner.h"

#include "planner/qr_foothold_planner.h"
#include "action/qr_action.h"
#include "ros/qr_vel_param_receiver.h"
#include "robots/qr_robot_sim.h"
#include "state_estimator/qr_robot_estimator.h"


#define MAX_TIME_SECONDS 1000.0f

Eigen::Matrix<float, 3, 1> desiredSpeed = {0.f, 0.f, 0.f};
float desiredTwistingSpeed = 0.f;
float footClearance = 0.01f;

/**
 * @brief Launch all controllers, planners  and esimators.
 * @param quadruped Pointer to A1Robot.
 * @param homeDir Package path.
 * @param useMPC: whether to use MPC
 * @return pointer to qrLocomotionController.
 */
qrLocomotionController *setUpController(qrRobot *quadruped, std::string homeDir, ros::NodeHandle &nh, bool useMPC = false);

/** 
 * @brief Setup the desired speed for robot.
 * @param controller The locomotion controller.
 * @param linSpeed The desired linear speed.
 * @param angSpeed The desired angular speed.
 */
void updateControllerParams(qrLocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed);

#endif //QR_RUNTIME_H

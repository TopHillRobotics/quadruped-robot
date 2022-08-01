/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: The robot runtime used to lanuch the robot
* Author: Zhu Yijie & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhao Yao;
*       mv implementation of func as runtime.h. @ Zhu Yijie 2021-11-24;
*       add switch mode @ Zhu Linsen 2022-01-30;
*/

#ifndef ASCEND_QUADRUPED_CPP_RUNTIME_H
#define ASCEND_QUADRUPED_CPP_RUNTIME_H

#include <iostream>
#include <typeinfo>
#include <yaml-cpp/yaml.h>


#include "mpc_controller/qr_locomotion_controller.h"
#include "planner/qr_com_planner.h"
#include "planner/qr_pose_planner.h"
#include "planner/qr_foothold_planner.h"
#include "action/action.h"
#include "ros/qr_vel_param_receiver.h"
#include "sim/a1_sim.h"
#include "state_estimator/qr_robot_estimator.h"

using namespace Quadruped;
// using namespace std;

#define MAX_TIME_SECONDS 1000.0f

Eigen::Matrix<float, 3, 1> desiredSpeed = {0.f, 0.f, 0.f};
float desiredTwistingSpeed = 0.f;
float footClearance = 0.01f;

/**
 * @brief launch all controllers, planners  and esimators.
 * @param quadruped : pointer to A1Robot.
 * @return pointer to qrLocomotionController.
 */
qrLocomotionController *setUpController(qrRobot *quadruped, std::string homeDir, std::string robotName);

/** @brief setup the desired speed for robot. */
void updateControllerParams(qrLocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed);

qrLocomotionController *setUpController(qrRobot *quadruped, std::string homeDir, std::string robotName);

void updateControllerParams(qrLocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed);
#endif //ASCEND_QUADRUPED_CPP_RUNTIME_H

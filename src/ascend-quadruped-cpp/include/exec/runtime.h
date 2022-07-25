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

#include "sim/a1_sim.h"
#include "state_estimator/robot_estimator.h"
#include "mpc_controller/locomotion_controller.h"
#include "planner/com_adjuster.h"
#include "planner/pose_planner.h"
#include "planner/foothold_planner.h"
#include "action/action.h"
#include "ros/cmd_vel_receiver.h"
#include "ros/slam_pose_receiver.h"
#include "ros/robot_odom_estimator.h"
#include "utils/tools.h"

using namespace Quadruped;
// using namespace std;

#define MAX_TIME_SECONDS 1000.0f

Eigen::Matrix<float, 3, 1> desiredSpeed = {0.f, 0.f, 0.f};
float desiredTwistingSpeed = 0.f;
float footClearance = 0.01f;

/**
 * @brief launch all controllers, planners  and esimators.
 * @param quadruped : pointer to A1Robot.
 * @return pointer to LocomotionController.
 */
LocomotionController *setUpController(Robot *quadruped, std::string homeDir, std::string robotName);

/** @brief setup the desired speed for robot. */
void updateControllerParams(LocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed);

#ifdef _useros
/** @brief a toolkit for loading yaml config file correctly. */
std::string GetHomeDir(std::string homeName = "quadruped-robot/");
#else
/** @brief a toolkit for loading yaml config file correctly. */
std::string GetHomeDir(std::string homeName = "ascend-quadruped-cpp/");
#endif

LocomotionController *setUpController(Robot *quadruped, std::string homeDir, std::string robotName);

void updateControllerParams(LocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed);


/**
 * @brief switch control mode, 0: vel->position 1: position->vel
 */
template <typename T = Robot>
bool SwitchMode(Robot* quadruped, LocomotionController *locomotionController, Eigen::Matrix<float, 3, 1> &desiredSpeed, float &desiredTwistingSpeed, int switchParam, float startTimeWall) 
{   
    if (quadruped->controlParams["mode"] == switchParam) {
        ROS_INFO("The switch mode is same as the currect mode, do not change...");
        return true;
    }

    if (switchParam > 1 || switchParam < 0) {
        ROS_ERROR("switch Mode error, please check: %d", switchParam);
        return false;
    }    
    Eigen::Matrix<float, 3, 1> curSpeed = desiredSpeed;
    float curTwistingSpeed = desiredTwistingSpeed;
    ros::Rate loop_rate(1000);
    int countTime = 0;
    if (switchParam == 0) {
        quadruped->controlParams["mode"] = 0;
        return true;
    }
    while (countTime < 1000) { 
        ++countTime;
        // startTimeWall = quadruped->GetTimeSinceReset();

        curSpeed << 0.f, 0.f, 0.f;
        curTwistingSpeed = 0.f;
        // march on the spot for 1s
        if (countTime < 500) {
            // ROS_INFO("march on the spot");
            updateControllerParams(locomotionController,
                                curSpeed,
                                curTwistingSpeed);
            locomotionController->Update();
            auto[hybridAction, qpSol] = locomotionController->GetAction();
            quadruped->Step(MotorCommand::convertToMatix(hybridAction), HYBRID_MODE);
        } else { // keep stance for 0.5s
            // ROS_INFO("keep stance");
            quadruped->Step(quadruped->standUpMotorAngles, MotorMode::POSITION_MODE);
            // for simulation
            if (typeid(T) == typeid(Robot)) {
                quadruped->state.yawOffset = quadruped->state.imu.rpy[2];
            }
            else {
                quadruped->state.yawOffset = static_cast<T*>(quadruped)->state.imu.rpy[2];
            }
            quadruped->state.basePosition = {0.f, 0.f, A1_BODY_HIGHT};
            quadruped->state.baseOrientation << 1.f, 0.f, 0.f, 0.f;
            quadruped->ReceiveObservation();        
            // position mode to vel mode
            if (switchParam == 0) {
                quadruped->controlParams["mode"] = 0;
            }
            // vel mode to position mode
            else if (switchParam == 1) {
                desiredTwistingSpeed = 0.f;
                quadruped->controlParams["mode"] = 1;
            }
            locomotionController->Reset();                 
        }

        ros::spinOnce();
        loop_rate.sleep();
        while (quadruped->GetTimeSinceReset() - startTimeWall < quadruped->timeStep * countTime) {}
    }
    return true;
}
#endif //ASCEND_QUADRUPED_CPP_RUNTIME_H

/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: The robot runtime used to lanuch the robot
* Author: Zhu Yijie & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhao Yao;
*       mv implementation of func as runtime.h. @ Zhu Yijie 2021-11-24;
*       add switch mode @ Zhu Linsen 2022-01-30;
*       add robot-runner @ Zhu Yijie 2022-04-01;
*/

#ifndef ASCEND_QUADRUPED_CPP_RUNTIME_H
#define ASCEND_QUADRUPED_CPP_RUNTIME_H

#include <iostream>
#include <typeinfo>
#include <yaml-cpp/yaml.h>
#include "robots/robot_a1.h"
#include "robots/robot_go1.h"
#include "robots/robot_a1_sim.h"
#include "estimators/state_estimator.hpp"
#include "controllers/locomotion_controller.h"
#include "planner/com_adjuster.h"
#include "planner/pose_planner.h"
#include "planner/foothold_planner.h"
#include "action/action.h"
#include "ros/cmd_vel_receiver.h"
#include "ros/slam_pose_receiver.h"
#include "ros/robot_odom_estimator.h"
#include "ros/switch_mode_receiver.h"
#include "utils/tools.h"
#include "dynamics/physics_transform.h"
#include "fsm/control_fsm.hpp"

using namespace Quadruped;

/**
 * @brief launch all controllers, planners  and esimators.
 * @param quadruped : pointer to A1Robot.
 * @return pointer to LocomotionController.
 */
LocomotionController *SetUpController(Robot *quadruped, GaitGenerator* gaitGenerator,
                                    DesiredStateCommand* desiredStateCommand,
                                    StateEstimatorContainer<float>* stateEstimators, 
                                    UserParameters* userParameters,
                                    std::string& homeDir);

/** @brief setup the desired speed for robot. */
void UpdateControllerParams(LocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed);


class RobotRunner {

public:

    RobotRunner(Robot* quadruped, std::string& homeDir, ros::NodeHandle& nh);

    bool Update();

    bool Step();

    ~RobotRunner();

    LocomotionController* GetLocomotionController()
    {
        return  controlFSM->GetLocomotionController();
    }

    StateEstimatorContainer<float>* GetStateEstimator()
    {
        return  stateEstimators;
    }

    DesiredStateCommand* GetDesiredStateCommand()
    {
      return desiredStateCommand;
    }

private:
    Robot* quadruped;
    GaitGenerator* gaitGenerator;
    UserParameters userParameters;
    StateEstimatorContainer<float>* stateEstimators;
    DesiredStateCommand* desiredStateCommand;
    ControlFSM<float>* controlFSM;

    float resetTime;
    float timeSinceReset;

    std::vector<MotorCommand> hybridAction;

};

#endif //ASCEND_QUADRUPED_CPP_RUNTIME_H

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

#ifndef QR_ROBOT_RUNNER_H
#define QR_ROBOT_RUNNER_H

#include <iostream>
#include <typeinfo>
#include <yaml-cpp/yaml.h>

#include "robots/qr_robot_a1.h"
#include "robots/qr_robot_go1.h"
#include "robots/qr_robot_a1_sim.h"
#include "robots/qr_robot_lite2_sim.h"
#include "robots/qr_robot_lite2.h"
#include "robots/qr_robot_sim.h"

#include "estimators/qr_state_estimator_container.h"
#include "controllers/qr_locomotion_controller.h"
#include "planner/qr_com_adjuster.h"
#include "planner/qr_pose_planner.h"
#include "planner/qr_foothold_planner.h"
#include "action/qr_action.h"
#include "ros/qr_cmd_vel_receiver.h"
#include "ros/qr_switch_mode_receiver.h"
#include "utils/qr_tools.h"
#include "utils/physics_transform.h"
#include "fsm/qr_control_fsm.hpp"


using namespace Quadruped;

/**
 * @brief launch all controllers, planners  and esimators.
 * @param quadruped : pointer to A1Robot.
 * @return pointer to LocomotionController.
 */
qrLocomotionController *SetUpController(qrRobot *quadruped, qrGaitGenerator* gaitGenerator,
                                    qrDesiredStateCommand* desiredStateCommand,
                                    qrStateEstimatorContainer* stateEstimators, 
                                    qrUserParameters* userParameters,
                                    std::string& homeDir);

/**
 * @brief Setup the desired speed for robot.
 */
void UpdateControllerParams(qrLocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed);


class qrRobotRunner {

public:

    qrRobotRunner(qrRobot* quadruped, std::string& homeDir, ros::NodeHandle& nh);

    bool Update();

    bool Step();

    ~qrRobotRunner();

    inline qrLocomotionController* GetLocomotionController() {
        return  controlFSM->GetLocomotionController();
    }

    inline qrStateEstimatorContainer* GetStateEstimator() {
        return  stateEstimators;
    }

    inline qrDesiredStateCommand* GetDesiredStateCommand() {
      return desiredStateCommand;
    }

    inline qrGaitGenerator* GetGaitGenerator() {
      return gaitGenerator;
    }
private:

    qrRobot* quadruped;

    qrGaitGenerator* gaitGenerator;

    qrUserParameters userParameters;

    qrStateEstimatorContainer* stateEstimators;

    qrDesiredStateCommand* desiredStateCommand;

    qrControlFSM<float>* controlFSM;

    float resetTime;

    float timeSinceReset;

    std::vector<qrMotorCommand> hybridAction;

};

#endif //QR_ROBOT_RUNNER_H

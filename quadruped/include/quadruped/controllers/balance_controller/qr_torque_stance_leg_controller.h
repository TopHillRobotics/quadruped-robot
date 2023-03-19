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

#ifndef QR_TORQUE_STANCE_LEG_CONTROLLER_H
#define QR_TORQUE_STANCE_LEG_CONTROLLER_H

#include "utils/qr_se3.h"
#include "robots/qr_robot.h"
#include "gait/qr_openloop_gait_generator.h"
#include "estimators/qr_state_estimator_container.h"
#include "planner/qr_com_adjuster.h"
#include "planner/qr_pose_planner.h"
#include "planner/qr_foothold_planner.h"
#include "controllers/qr_desired_state_command.hpp"

namespace Quadruped {

class TorqueStanceLegController {

public:

    /**
     * @brief constructor of TorqueStanceLegController
     * @param robot: pointer to robot.
     * @param gaitGenerator: pointer to gait generator.
     * @param stateEstimators: pointer to state estimator container.
     * @param comAdjuster: pointer to com adjuster.
     * @param posePlanner: pointer to pose planner.
     * @param footholdPlanner: pointer to foothold planner.
     * @param userParameters: reference to user parameters.
     * @param configFilepath: the string of path to config file.
     */
    TorqueStanceLegController(qrRobot *robot,
                              qrGaitGenerator *gaitGenerator,
                              qrStateEstimatorContainer* stateEstimators,
                              qrComAdjuster *comAdjuster,
                              qrPosePlanner *posePlanner,
                              qrFootholdPlanner *footholdPlanner,
                              qrUserParameters& userParameters,
                              std::string configFilepath
                              );

    /**
     * @brief default constructor of TorqueStanceLegController;
     */
    TorqueStanceLegController() = default;

    /**
     * @brief default destructor of TorqueStanceLegController.
     */
    virtual ~TorqueStanceLegController() = default;

    /**
     * @brief bind desired state command to this stance leg controller.
     * @param desiredStateCommandIn
     */
    void BindCommand(qrDesiredStateCommand* desiredStateCommandIn) {
        desiredStateCommand = desiredStateCommandIn;
    }

    /**
     * @brief Reset the stance leg controller with current time.
     * @param currentTime: current time to reset.
     */
    virtual void Reset(float currentTimeIn);

    /**
     * @brief Update the ratio of the friction force to robot gravity.
     * @param contacts: descripte the contact status of four feet
     * @param N: the number of contact feet
     * @param normalizedPhase: the phase of swing leg
     */
    void UpdateFRatio(Vec4<bool> &contacts, int &N, float &normalizedPhase);

    /**
     * @brief Update desired command, especially acceleration for force balance using KP/KD method.
     * MPC does not use the acceleration for calculation.
     */
    virtual void UpdateDesCommand();

    /**
     * @brief Update the stance leg controller with current time every control loop.
     * @param currentTime: current time to update.
     */
    void Update(float currentTime);

    /**
     * @brief Get the motor commands of the stance leg controller
     * @return commands and forces calculated by force balance.
     */
    virtual std::tuple<std::map<int, qrMotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

    /**
     * @brief Desired linear speed of quadruped. commanded by user.
     */
    Vec3<float> desiredSpeed;

    /**
     * @brief Desired twisting speed of quadruped commanded by user.
     */
    float desiredTwistingSpeed;

//private:

    /**
     * @brief Stores the current time.
     */
    float currentTime;

    /**
     * @brief Pointer to robot.
     */
    qrRobot *robot;

    /**
     * @brief Pointer to gait generator.
     */
    qrGaitGenerator *gaitGenerator;

    /**
     * @brief Pointer to robot estimator.
     */
    qrRobotEstimator *robotEstimator;

    /**
     * @brief Pointer to ground estimator.
     */
    qrGroundSurfaceEstimator *groundEstimator;

    /**
     * @brief Pointer to com adjuster.
     */
    qrComAdjuster *comAdjuster;

    /**
     * @brief Pointer to pose planner.
     */
    qrPosePlanner *posePlanner;

    /**
     * @brief Pointer to foothold planner.
     */
    qrFootholdPlanner *footholdPlanner;

    /**
     * @brief Pointer to desired State command.
     */
    qrDesiredStateCommand* desiredStateCommand;

    /**
     * @brief Desired body height while trotting.
     * This is overwritten in the class constructor by userParameters.desiredHeight.
     */
    float desiredBodyHeight = 0.3;

    /**
     * @brief Defines the interaction force effect between foot and env.
     */
    Vec4<float> frictionCoeffs;

    /**
     * @brief YAML node for stance leg comtroller.
     */
    YAML::Node param;

    /**
     * @brief A string stores the control mode.
     * Trot, walk or advanced trot.
     */
    std::string controlModeStr;

    /**
     * @brief Whether to compute the force in world frame.
     */
    bool computeForceInWorldFrame;

    /**
     * @brief Number of stance legs at current time.
     */
    int N;

    float moveBasePhase;

    /**
     * @brief 4-element vector stores the contact state of 4 footholds.
     */
    Eigen::Matrix<bool, 4, 1> contacts;

    /**
     * @brief KP vector for calculating acceleration.
     */
    Eigen::Matrix<float, 6, 1> KP;

    /**
     * @brief KD vector for calculating acceleration.
     */
    Eigen::Matrix<float, 6, 1> KD; // for acceleration

    /**
     * @brief Max acceleration for KP/KD calculation.
     */
    Eigen::Matrix<float, 6, 1> maxDdq;

    /**
     * @brief Minimal acceleration for KP/KD calculation.
     */
    Eigen::Matrix<float, 6, 1> minDdq;

    /**
     * @brief The weight for the 6 acceleration components.
     */
    Eigen::Matrix<float, 6, 1> accWeight;

    /**
     * @brief Min force that applys. User can adjust parameter of each leg.
     */
    Vec4<float> fMinRatio;

    /**
     * @brief Min force that applys. User can adjust parameter of each leg.
     */
    Vec4<float> fMaxRatio;

    /**
     * @brief An iteration counter for debug.
     */
    long long count = 0;

};

} // namespace Quadruped

#endif // QR_TORQUE_STANCE_LEG_CONTROLLER_H

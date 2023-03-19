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

#ifndef QR_LOCOMOTION_CONTROLLER_H
#define QR_LOCOMOTION_CONTROLLER_H

#include <iostream>
#include <map>
#include <Eigen/Dense>

#include "utils/qr_cpptypes.h"
#include "robots/qr_robot.h"
#include "gait/qr_openloop_gait_generator.h"
#include "gait/qr_walk_gait_generator.h"
#include "controllers/qr_swing_leg_controller.h"
#include "controllers/qr_stance_leg_controller_interface.h"
#include "planner/qr_com_adjuster.h"
#include "planner/qr_pose_planner.h"
#include "estimators/qr_state_estimator_container.h"


namespace Quadruped {

class qrLocomotionController {

public:

    /**
     * @brief Constructor of class qrLocomotionController.
     * @param robot: pointer to the Robot.
     * @param gaitGenerator: pointer to the Gait Generator.
     * Openloop Gait Generator or Walk Gait Generator.
     * @param desiredStateCommand: pointer to DesiredStateCommand.
     * @param stateEstimator: pointer to StateEstimatorContainer.
     * @param comAdjuster: pointer to ComAdjuster.
     * @param posePlanner: pointer to PosePlanner.
     * @param swingLegController: pointer to RaibertSwingLegController.
     * @param stanceLegController: pointer to StanceLegControllerInterface.
     * @param userParameters: pointer to UserParameters.
     */
    qrLocomotionController(qrRobot *robot,
                         qrGaitGenerator *gaitGenerator,
                         qrDesiredStateCommand* desiredStateCommand,
                         qrStateEstimatorContainer *stateEstimator,
                         qrComAdjuster *comAdjuster,
                         qrPosePlanner *posePlanner,
                         qrRaibertSwingLegController *swingLegController,
                         qrStanceLegControllerInterface *stanceLegController,
                         qrUserParameters *userParameters);

    ~qrLocomotionController() = default;

    /**
     * @brief Reset all components in the locomotion controller.
     */
    void Reset();

    /**
     * @brief Bind desiredStateCommand to swingLegController.
     */
    void BindCommand() {
        swingLegController->BindCommand(desiredStateCommand);
        stanceLegController->BindCommand(desiredStateCommand);
    }

    /**
     * @brief Update components in the locomotion controller every control loop.
     */
    void Update();

    /** @brief Compute all motors' commands via subcontrollers.
     *  @return control ouputs (e.g. positions/torques) for all (12) motors.
     */
    std::tuple<std::vector<qrMotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

    /**
     * @brief A function as GetAction(). Only for Debug.
     * @return control ouputs (e.g. positions/torques) for all (12) motors.
     */
    std::tuple<std::vector<qrMotorCommand>, Eigen::Matrix<float, 3, 4>> GetFakeAction();

    /**
     * @brief Getter method of member gaitGenerator.
     */
    inline qrGaitGenerator *GetGaitGenerator() const {
        return gaitGenerator;
    }

    /**
     * @brief Getter method of member swingLegController.
     */
    inline qrRaibertSwingLegController *GetSwingLegController() const {
        return swingLegController;
    }

    /**
     * @brief Getter method of member stanceLegController.
     */
    inline TorqueStanceLegController *GetStanceLegController() const {
        return stanceLegController->c;
    }

    /**
     * @brief Getter method of member posePlanner.
     */
    inline qrPosePlanner *GetPosePlanner() {
        return posePlanner;
    }

    /**
     * @brief get current time since reset of the robot
     * @return time since reset of the robot
     */
    double GetTime() {
        return robot->GetTimeSinceReset();
    }

    /**
     * @brief pointer to RaibertSwingLegController
     */
    qrRaibertSwingLegController *swingLegController;

    /**
     * @brief pointer to StanceLegControllerInterface
     */
    qrStanceLegControllerInterface *stanceLegController;

    /**
     * @brief a member indicates whether the locomotion has been stopped
     */
    bool stop=false;

    int swingSemaphore = 10000000;

    float stopTick = 0;

private:

    /**
     * @brief Pointer to Robot.
     */
    qrRobot *robot;

    /**
     * @brief Pointer to GaitGenerator.
     */
    qrGaitGenerator *gaitGenerator;

    /**
     * @brief Pointer to StateEstimatorContainer.
     */
    qrStateEstimatorContainer *stateEstimator;

    /**
     * @brief Pointer to ComAdjuster.
     */
    qrComAdjuster *comAdjuster;

    /**
     * @brief Pointer to PosePlanner.
     */
    qrPosePlanner *posePlanner;

    /**
     * @brief Pointer to DesiredStateCommand.
     */
    qrDesiredStateCommand* desiredStateCommand;

    /**
     * @brief A list of commands that will send to gazebo/real quadrupeds every control loop.
     */
    std::vector<qrMotorCommand> action;

    /**
     * @brief Records the time quadruped resets.
     */
    double resetTime;

    /**
     * @brief Records the time since the quadruped resets.
     */
    double timeSinceReset;

};

} // Namespace Quadruped

#endif // QR_LOCOMOTION_CONTROLLER_H

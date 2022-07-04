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

#include "common/qr_timer.h"
#include "common/qr_c_types.h"
#include "robot/qr_robot.h"
#include "robot/qr_motor_cmd.h"
#include "planner/qr_gait_generator.h"
#include "controller/qr_swing_leg_controller.h"
#include "controller/qr_stance_leg_controller.h"
#include "planner/qr_com_planner.h"
#include "estimator/qr_ground_estimator.h"

/** 
 * @brief Universe Controller that combines planners and estimators.
 * todo : laterly the estimators need to be moved outside and runs asynchronously with controllers.
 */
class qrLocomotionController {

public:
    /** 
     * @brief Constructor of qrLocomotionController.
     * @param robot The robot object pointer.
     * @param homeDir the file path of this project.
     * @param robotName the name of the robot.
     */
    qrLocomotionController(qrRobot *robot, std::string homeDir, std::string robotName);

    ~qrLocomotionController() = default;

    /**
     * @brief Initial locomotion controller.
     * @param homeDir the file path of this project.
     * @param robotName the name of the robot.
     */
    void Initialization(std::string homeDir, std::string robotName);

    /** 
     * @brief Reset the planners, estimatiors and controllers.
     */
    void Reset();

    /** 
     * @brief Update the planners, estimatiors and controllers.
     */
    void Update();

    /** 
     * @brief setup the desired speed for robot. 
     * @param linSpeed The desired linear speed.
     * @param angSpeed The desired angular speed.
     */
    void UpdateDesiredSpeed(Vec3<float> linSpeed, float angSpeed);

    /** 
     * @brief Compute all motors' commands via subcontrollers.
     *  @return tuple<map, Matrix<3,4>> : return control ouputs (e.g. positions/torques) for all (12) motors.
     */
    std::tuple<std::vector<qrMotorCmd>, Mat3x4<float>> GetAction();
    std::tuple<std::vector<qrMotorCmd>, Mat3x4<float>> GetFakeAction();

    /** 
     * @brief Get gait generator object.
     *  @return qrGaitGenerator.
     */
    inline qrGaitGenerator *GetGaitGenerator()
    {
        return gaitGenerator;
    }

    /** 
     * @brief Get gait generator object.
     *  @return qrSwingLegController.
     */
    inline qrSwingLegController *GetSwingLegController()
    {
        return swingLegController;
    }

    /** 
     * @brief Get gait generator object.
     *  @return qrGaitGenerator.
     */
    inline qrStanceLegController *GetStanceLegController()
    {
        return stanceLegController;
    }

    /** 
     * @brief Get ground estimator object.
     *  @return groundEstimator.
     */
    inline qrGroundSurfaceEstimator *GetGroundEstimator()
    {
        return groundEstimator;
    }

    /** 
     * @brief Get current time.
     *  @return float time.
     */
    double GetTime()
    {
        return robot->GetTimeSinceReset();
    }

    /** 
     * @brief let robot walk one step more.
     */
    void ForwardOne();

    /** 
     * @brief qrSwingLegController object.
     */
    qrSwingLegController *swingLegController;

    /** 
     * @brief qrStanceLegController object.
     */
    qrStanceLegController *stanceLegController;

    /** 
     * @brief mark if the robot has stopped. True is stop, false is running.
     */
    bool stop=false;

    /** 
     * @brief total steps.
     */
    int swingSemaphore = 10000000; 

    /** 
     * @brief the time when robot stopped.
     */
    float stopTick = 0;

    /**
     * @brief The desired linear velocity.
     */
    Vec3<float> desiredSpeed;

    /**
     * @brief The desired angular velocity.
     */
    Vec3<float> desiredTwistingSpeed;

private:

    /** 
     * @brief qrRobot object.
     */
    qrRobot *robot;

    /**
     * @brief qrRobotState object pointer.
     */
    qrRobotState* robotState;

    /**
     * @brief qrRobotConfig object pointer.
     */
    qrRobotConfig* robotConfig;
    
    /** 
     * @brief qrGaitGenerator object.
     */
    qrGaitGenerator *gaitGenerator;

    /** 
     * @brief qrGroundSurfaceEstimator object.
     */
    qrGroundSurfaceEstimator *groundEstimator;

    /** 
     * @brief qrComPlanner object.
     */
    qrComPlanner *comPlanner;

    /** 
     * @brief joint command list.
     */
    std::vector<qrMotorCmd> action;
    
    /** 
     * @brief the time when last call Reset() function.
     */
    double resetTime;
    
    /** 
     * @brief the time has passed after last call Reset() function.
     */
    double timeSinceReset;
};

#endif // QR_LOCOMOTION_CONTROLLER_H

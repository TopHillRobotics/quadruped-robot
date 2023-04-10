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

#ifndef QR_RAIBERT_SWING_LEG_CONTROLLER_H
#define QR_RAIBERT_SWING_LEG_CONTROLLER_H

#include "robots/qr_motor.h"
#include "gait/qr_openloop_gait_generator.h"
#include "estimators/qr_state_estimator_container.h"
#include "planner/qr_foothold_planner.h"
#include "controllers/qr_foot_trajectory_generator.h"


namespace Quadruped {

class qrRaibertSwingLegController {

public:

    /**
     * @brief Constructor of member qrRaibertSwingLegController.
     * @param robot: pointer to Robot and its derived class.
     * @param gaitGenerator: pointer to GaitGenerator.
     * @param stateEstimators: pointer to StateEstimatorContainer.
     * @param FootholdPlanner: pointer to FootholdPlanner.
     * @param userParameters: pointer to UserParameters.
     * @param configPath: path of config files.
     */
    qrRaibertSwingLegController(qrRobot *robot,
                              qrGaitGenerator *gaitGenerator,
                              qrStateEstimatorContainer* stateEstimators,
                              qrFootholdPlanner *qrFootholdPlanner,
                              qrUserParameters& userParameters,
                              std::string configPath);

    ~qrRaibertSwingLegController() = default;

    /**
     * @brief Bind desired command to this swing leg controller.
     * @param desiredStateCommandIn: pointer to DesiredStateCommand.
     */
    void BindCommand(qrDesiredStateCommand* desired_state_command) {
        desiredStateCommand = desired_state_command;
    }

    /**
     * @brief Quadratic interpolation function, used to generate polygon curve.
     * @param phase: specifies the given phase in [0, 1] to compute the trajectory.
     * @param start: specifies the foot's position at the beginning of swing cycle.
     * @param mid: mid point of the parabola.
     * @param end specifies the foot's desired position at the end of swing cycle.
     * @return a float value with phase.
     */
    float GenParabola(float phase, float start, float mid, float end);

    /**
     * @brief Generating the trajectory of the swing leg.
     * @param inputPhase: current phase of swing leg.
     * @param startPos: start position of the footstep of the swing leg.
     * @param endPos: final position of the footstep of the swing leg.
     * @return foot position like (x,y,z).
     */
    Eigen::Matrix<float, 3, 1> GenSwingFootTrajectory(float inputPhase,
                                                      Eigen::Matrix<float, 3, 1> startPos,
                                                      Eigen::Matrix<float, 3, 1> endPos);

    /**
     * @brief Reset the swing leg controller with current time.
     * @param currentTime: current time to reset.
     */
    void Reset(float currentTime);

    /**
     * @brief Update the swing leg controller with current time.
     * @param CurrentTime: current time to reset.
     */
    void Update(float currentTime);

    /**
     * @brief Get position-mode commands for swing leg motors.
     */
    std::map<int, Eigen::Matrix<float, 5, 1>> GetAction();

    /**
     * @brief pointer to DesiredStateCommand.
     */
    qrDesiredStateCommand* desiredStateCommand;

    /**
     * @brief The desired speed of the quadruped.
     * Appear in velocity mode usually.
     */
    Eigen::Matrix<float, 3, 1> desiredSpeed;

    /**
     * @brief The desired angular velocity of the quadruped.
     * Appear in velocity mode usually.
     */
    float desiredTwistingSpeed;

    /**
     * @brief The position correction coefficients in Raibert's formula.
     * In the formula, Kp = sqrt(z0/||g||).
     * Currently, we set it to: const Matrix<float, 3, 1> swingKp(0.03, 0.03, 0.03).
     */
    Eigen::Matrix<float, 3, 1> swingKp;

    /**
     * @brief pointer to Robot.
     */
    qrRobot *robot;

    /**
     * @brief pointer to GaitGenerator.
     */
    qrGaitGenerator *gaitGenerator;

    /**
     * @brief pointer to RobotEstimator.
     */
    qrRobotEstimator *stateEstimator;

    /**
     * @brief pointer to GroundSurfaceEstimator.
     */
    qrGroundSurfaceEstimator *groundEstimator;

    /**
     * @brief pointer to FootholdPlanner.
     */
    qrFootholdPlanner *footholdPlanner;

    /**
     * @brief pointer to UserParameters.
     */
    qrUserParameters* userParameters;

    /**
     * @brief stores the leg id of the swing leg.
     */
    std::vector<u8> swingFootIds;

    /**
     * @brief leg state of last control loop.
     */
    Eigen::Matrix<int, 4, 1> lastLegState;

    /**
     * @brief desired height of the quadruped.
     */
    Eigen::Matrix<float, 3, 1> desiredHeight;

    /**
     * @brief Stores the desired joint angle and velocity.
     * Map from joint ID to joint angle, velocity and leg ID.
     */
    std::map<int, std::tuple<float, float, int>> swingJointAnglesVelocities;

    /**
     * @brief Foot positions in base frame when switch leg state.
     */
    Eigen::Matrix<float, 3, 4> phaseSwitchFootLocalPos;

    /**
     * @brief Foot positions in world frame when switch leg state.
     */
    Eigen::Matrix<float, 3, 4> phaseSwitchFootGlobalPos;

    /**
     * @brief Footholds in world frame.

     */
    Eigen::Matrix<float, 3, 4> footHoldInWorldFrame;

    /**
     * @brief Desired foot positions in world frame.
     * This member is only used for gazebo. Maybe removed in the future.
     */
    Eigen::Matrix<float, 3, 4> footTargetPositionsInWorldFrame;

    /**
     * @brief Desired foot positions in base frame.
     */
    Eigen::Matrix<float, 3, 4> desiredFootPositionsInBaseFrame;

    /**
     * @brief Stores the trajectory object for all 4 legs.
     * Will be used when any of the legs is swinging.
     */
    SwingFootTrajectory swingFootTrajectories[4];

    /**
     * @brief The spline information for the swing trajectory.
     */
    qrSplineInfo splineInfo;

    /**
     * @brief The config file path of swing_leg_controller.yaml.
     */
    YAML::Node swingLegConfig;

    /**
     * @brief Foot offset. This member will be removed in the future.
     */
    float footOffset;

    /**
     * @brief Foot position in base frame at last control loop.
     */
    Eigen::Matrix<float, 3, 4> foot_pos_target_last_time;

    /**
     * @brief Same as foot_pos_target_last_time. Will be removed in the future.
     */
    Eigen::Matrix<float, 3, 4> foot_pos_rel_last_time;
};

} // namespace Quadruped

#endif // QR_RAIBERT_SWING_LEG_CONTROLLER_H

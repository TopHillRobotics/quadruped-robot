// The MIT License

// Copyright (c) 2022 
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:  tophill.robotics@gmail.com

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

#ifndef QR_SWING_LEG_CONTROLLER_H
#define QR_SWING_LEG_CONTROLLER_H

#include "robots/qr_motor.h"
#include "planner/qr_gait_generator.h"
#include "state_estimator/qr_robot_estimator.h"
#include "state_estimator/qr_ground_estimator.h"
#include "planner/qr_foothold_planner.h"
#include "controller/qr_foot_trajectory_generator.h"

/**
 * @brief Control swing leg of robot
 */
class qrSwingLegController {

public:

    /**
     * @brief Construct a qrSwingLegController object using the given parameters.
     * @param robot The robot to which the controller is associated.
     * @param gaitGenerator The gait generator 
     * @param stateEstimator The gait estimator
     * @param groundEstimator The ground estimator
     * @param FootholdPlanner The foothold planner
     * @param desiredLinearSpeed The desired linear velocity
     * @param desiredTwistingSpeed The desired angular velocity
     * @param footClearance 
     * @param configPath The config file path
     */
    qrSwingLegController(qrRobot *robot,
                                qrGaitGenerator *gaitGenerator,
                                qrRobotEstimator *stateEstimator,
                                qrGroundSurfaceEstimator *groundEstimator,
                                qrFootholdPlanner *FootholdPlanner,
                                Eigen::Matrix<float, 3, 1> desiredSpeed,
                                float desiredTwistingSpeed,
                                float desiredHeight,
                                float footClearance,
                                std::string configPath);

    /**
     * @brief Deconstruct a qrSwingLegController object.
     */
    ~qrSwingLegController() = default;

    /**
     * @brief Quadratic interpolation function, used to generate polygon curve.
     * @param phase specifies the given phase in [0, 1] to compute the trajectory.
     * @param start specifies the foot's position at the beginning of swing cycle.
     * @param mid
     * @param end specifies the foot's desired position at the end of swing cycle.
     * @return a float value with phase
     */
    float GenParabola(float phase, float start, float mid, float end);

    /**
     * @brief Generating the trajectory of the swing leg
     * @param inputPhase
     * @param startPos
     * @param endPos
     * @return foot position like (x,y,z)
     */
    Eigen::Matrix<float, 3, 1> GenSwingFootTrajectory(float inputPhase,
                                                        Eigen::Matrix<float, 3, 1> startPos,
                                                        Eigen::Matrix<float, 3, 1> endPos);

    /**
     * @brief The process of velocity locomotion.
     * @param dR Represent base frame in control frame.
     * @param footPositionInBaseFrame The the foot position in base frame.
     * @param legId The id of processing leg.
     */
    void VelocityLocomotionProcess(const Eigen::Matrix<float, 3, 3> &dR, 
                                    Eigen::Matrix<float, 3, 1> &footPositionInBaseFrame, 
                                    int legId);

    /**
     * @brief The process of position locomotion.
     * @param footPositionInWorldFrame The the foot position in world frame.
     * @param footPositionInBaseFrame The the foot position in base frame.
     * @param legId The id of processing leg.
     */
    void PositionLocomotionProcess(Eigen::Matrix<float, 3, 1> &footPositionInWorldFrame, 
                                    Eigen::Matrix<float, 3, 1> &footPositionInBaseFrame, 
                                    int legId);


    /**
     * @brief update linear velocity and angular velocity of controllers
     * @param linSpeed: linear velocity
     * @param angSpeed: yaw twist velocity
     */
    virtual void UpdateControlParameters(const Eigen::Vector3f& linSpeed, const float& angSpeed);

    /**
     * @brief Reset the controller parameters.
     */
    void Reset(float currentTime);

    /**
     * @brief Update the controller parameters.
     */
    void Update(float currentTime);


    /** @brief Compute all motors' commands using this controller.
     *  @return tuple<map, Matrix<float, 5, 1>> : 
     *          return control ouputs (e.g. positions/torques) for all 12 motors.
     */
    std::map<int, Eigen::Matrix<float, 5, 1>> GetAction();

    /**
     * @brief The desired linear velocity. This memeber variable appears in velocity mode.
     */
    Eigen::Matrix<float, 3, 1> desiredSpeed;

    /**
     * @brief The desired angular velocity. This memeber variable appears in velocity mode.
     */
    float desiredTwistingSpeed;

    /**
     * @brief the robot object to which the controller is associated.
     */
    qrRobot *robot;

    /**
     * @brief Gait Generator object pointer.
     */
    qrGaitGenerator *gaitGenerator;

    /**
     * @brief Robot estimator pointre.
     */
    qrRobotEstimator *stateEstimator;

    /**
     * @brief Ground estimator pointer.
     */
    qrGroundSurfaceEstimator *groundEstimator;

    /**
     * @brief Robot's foothold planner. Get desired COM pose when in walk locomotion.
     */
    qrFootholdPlanner *footholdPlanner;

    /**
     * @brief The state of each leg.
     */
    Eigen::Matrix<int, 4, 1> lastLegState;

    /**
     * @brief Desired robot's body height. 
     */
    Eigen::Matrix<float, 3, 1> desiredHeight;

    /**
     * @brief The joint's angles and motor velocities data structure. The first is joint angle,
     *        the second is motor velocity and the third is the leg index.
     */ 
    std::map<int, std::tuple<float, float, int>> swingJointAnglesVelocities;

    /**
     * @brief Foot positions in the base frame when the leg state changes.
     */
    Eigen::Matrix<float, 3, 4> phaseSwitchFootLocalPos;
    
    /**
     * @brief Foot positions in the world frame when the leg state changes.
     */
    Eigen::Matrix<float, 3, 4> phaseSwitchFootGlobalPos;
    
    /**
     * @brief the footholds in the world frame.
     */
    Eigen::Matrix<float, 3, 4> footHoldInWorldFrame;

    /**
     * @brief The trajectories of each leg.
     */
    qrSwingFootTrajectory swingFootTrajectories[4];


    /**
     * @brief The config file path of swing_leg_controller.yaml.
     */
    YAML::Node swingLegConfig;

    /**
     * @brief the initial foot pose in position mode.
     */
    std::vector<std::vector<float>> footInitPose;

    /**
     * @brief the foot offset.
     */
    float footOffset;
};

#endif //QR_SWING_LEG_CONTROLLER_H

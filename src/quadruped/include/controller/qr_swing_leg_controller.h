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

#ifndef QR_SWING_LEG_CONTROLLER_H
#define QR_SWING_LEG_CONTROLLER_H

#include "robot/qr_motor_cmd.h"
#include "planner/qr_gait_generator.h"
#include "estimator/qr_robot_estimator.h"
#include "estimator/qr_ground_estimator.h"
#include "planner/qr_foothold_planner.h"
#include "planner/qr_foot_trajectory_generator.h"

class qrSwingLegController{
public:
    /**
     * @brief Construct a qrSwingLegController object using the given parameters.
     * @param robot The robot to which the controller is associated.
     * @param gaitGenerator The gait generator 
     * @param stateEstimator The gait estimator
     * @param groundEstimator The ground estimator
     * @param FootholdPlanner The foothold planner
     * @param desiredLinearVelocity The desired linear velocity
     * @param desiredTwistingVelocity The desired angular velocity
     * @param desiredHeight The desired body height of the robot.
     * @param footClearance 
     * @param configPath The config file path
     */
    qrSwingLegController(qrRobot *robot,
                         qrGaitGenerator *gaitGenerator,
                         qrRobotEstimator *stateEstimator,
                         qrGroundSurfaceEstimator *groundEstimator,
                         qrFootholdPlanner *footholdPlanner,
                         Eigen::Matrix<float, 3, 1> desiredLinearVelocity,
                         float desiredTwistingVelocity,
                         float desiredHeight,
                         float footClearance,
                         std::string configPath);

    /**
     * @brief Deconstruct a qrSwingLegController object.
     */
    virtual ~qrSwingLegController();

    /**
     * @brief Reset the controller parameters.
     */
    virtual void Reset();

    /**
     * @brief Update the controller parameters.
     */
    virtual void Update();

    /**
     * @brief Generate a parabola curve using three given points: (0, y0), (0.5, ym) and (1, y1).
     * y = ax^2 + bx + c
     * @param x specifies the given x coordiate (phase) to compute the parabola value. x is normalized to [0,1].
     * @param y0 specifies the y coordinate of the start point at t=0.
     * @param ym specifies the y coordinate of the middle point at x=0.5. 
     * @param y1 specifies the y coordinate of the end point at x=1.
     * @return the y coordinate of the parabola curve for a given x (phase).
     */
    float GenerateParabola(float x, float y0, float ym, float y1);

    /**
     * @brief Generate the 3D trajectory of the swing leg
     * @param phase specifies the given phase in [0, 1] to compute the trajectory.
     * @param startPos specifies the foot's position at the beginning of swing cycle.
     * @param endPos specifies the foot's desired position at the end of swing cycle.
     * @param clearance specifies the height over the ground.
     * @return the desired foot position (x,y,z) at the current swing phase. 
     */
    Eigen::Matrix<float, 3, 1> GenerateSwingFootTrajectory(float phase,
                                                           Eigen::Matrix<float, 3, 1> startPos,
                                                           Eigen::Matrix<float, 3, 1> endPos,
                                                           float clearance=0.1);

    /** @brief Compute all motors' commands using this controller.
     *  @return tuple<map, Matrix<3,4>> : 
     *          return control ouputs (e.g. positions/torques) for all 12 motors.
     */
    virtual std::tuple<std::vector<MotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

private:
    
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
    
    qrRobotEstimator *robotEstimator;
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
    qrEigen::Matrix<int, 4, 1> lastLegState;

    /**
     * @brief Desired robot's body height. 
     */
    Eigen::Matrix<float, 3, 1> desiredHeight;

    /**
     * @brief The desired linear velocity. This memeber variable appears in velocity mode.
     */
    Eigen::Matrix<float, 3, 1> desiredLinearVelocity;

    /**
     * @brief The desired angular velocity. This memeber variable appears in velocity mode.
     */
    float desiredTwistingVelocity;

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
    std::string configFilepath;

    /**
     * @brief the initial foot pose in position mode.
     */
    std::vector<std::vector<float>> footInitPose;
    
    /**
     * @brief The time when executing Reset().
     */
    float resetTime;
    
};

#endif //QR_SWING_LEG_CONTROLLER_H
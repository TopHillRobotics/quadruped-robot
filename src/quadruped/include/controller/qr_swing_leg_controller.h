// The MIT License

// Copyright (c) 2022 
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: Xinyu Zhang   email: tophill.robotics@gmail.com

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
#include "planner/qr_openloop_gait_generator.h"
#include "estimator/qr_robot_estimator.h"
#include "estimator/qr_ground_estimator.h"
#include "planner/qr_foothold_planner.h"
#include "planner/qr_foot_trajectory_generator.h"

class qrSwingLegController{
public:
    /**
     * @brief Construct function of qrSwingLegController
     * @param robot
     * @param gaitGenerator
     * @param stateEstimator
     * @param groundEstimator
     * @param FootholdPlanner
     * @param desiredSpeed
     * @param desiredTwistingSpeed
     * @param desiredHeight
     * @param footClearance
     * @param configPath
     */
    qrSwingLegController(qrRobot *robot,
                         qrGaitGenerator *gaitGenerator,
                         qrRobotEstimator *stateEstimator,
                         qrGroundSurfaceEstimator *groundEstimator,
                         qrFootholdPlanner *footholdPlanner,
                         Eigen::Matrix<float, 3, 1> desiredSpeed,
                         float desiredTwistingSpeed,
                         float desiredHeight,
                         float footClearance,
                         std::string configPath);

    virtual ~qrSwingLegController();

    /**
     * @brief Reset the parameters of the qrSwingLegController.
     */
    virtual void Reset();

    /**
     * @brief Update the parameters of the qrSwingLegController.
     */
    virtual void Update();

    /** @brief Compute all motors' commands via controllers.
     *  @return tuple<map, Matrix<3,4>> : 
     *          return control ouputs (e.g. positions/torques) for all (12) motors.
     */
    virtual std::tuple<std::vector<MotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

private:
    
    /**
     * @brief The robot object pointer.
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
     * @brief Desired speed. This param usually appears in velocity mode.
     */
    Eigen::Matrix<float, 3, 1> desiredSpeed;
    /**
     * @brief Desired speed of twist command. This param usually appears in velocity mode.
     */
    float desiredTwistingSpeed;
    /**
     * @brief The joint's angles and motor velocities data structure. The first data is joint angle,
     *        the second data is motor velocity and the third data is the index of leg.
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
     * @brief Orientation in control frame.(not use)
     */
    Quat<float> controlFrameOrientationSource;
    /**
     * @brief Robot position.(not use)
     */
    Vec3<float> controlFrameOriginSource;
    /**
     * @brief (not use)
     */
    Eigen::Matrix<float, 3, 4> phaseSwitchFootControlPos;
    /**
     * @brief (not use)
     */
    Eigen::Matrix<float, 3, 4> footHoldInControlFrame;
    /**
     * @brief The trajectories of each leg.
     */
    SwingFootTrajectory swingFootTrajectories[4];
    /**
     * @brief File path of swing_leg_controller.yaml.
     */
    std::string configFilepath;
    /**
     * @brief Init pose in position mode.
     */
    std::vector<std::vector<float>> footInitPose;
    
};

#endif //QR_SWING_LEG_CONTROLLER_H
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

#ifndef QR_STANCE_LEG_CONTROLLER_H
#define QR_STANCE_LEG_CONTROLLER_H

#include <iostream>
#include <map>

#include <eigen3/Eigen/Dense>

#include "common/qr_timer.h"
#include "common/qr_types.h"
#include "common/qr_qp_torque_optimizer.h"
#include "robot/qr_robot.h"
#include "robot/qr_motor_cmd.h"
#include "planner/qr_gait_generator.h"
#include "planner/qr_com_planner.h"
#include "planner/qr_pose_planner.h"
#include "estimator/qr_robot_velocity_estimator.h"
#include "estimator/qr_ground_estimator.h"

/**
 * @brief Controller for stance leg.
 */
class qrStanceLegController{
public:
    /**
     * @brief Constructor of qrStanceLegController using given many object pointers and attributes.
     * @param robot The robot object pointer.
     * @param gaitGenerator The gait generator object pointer.
     * @param robotVelocityEstimator The robot estimator object pointer.
     * @param groundEstimator The ground estimator object pointer.
     * @param comPlanner The COM planner object pointer.
     * @param posePlanner The pose planner object pointer.
     * @param footholdPlanner The foothold planner object pointer.
     * @param desired_speed The robot desired speed in velocity control.
     * @param desiredTwistingSpeed The robot desired twisting speed in velocity control.
     * @param desiredBodyHeight The desired body height of robot.
     * @param configFilepath The stance leg config file path.
     */
    qrStanceLegController(qrRobot *robot,
                          qrGaitGenerator *gaitGenerator,
                          qrRobotVelocityEstimator *robotVelocityEstimator,
                          qrGroundSurfaceEstimator *groundEstimator,
                          qrComPlanner *comPlanner,
                          qrPosePlanner *posePlanner,
                          qrFootholdPlanner *footholdPlanner,
                          Eigen::Matrix<float, 3, 1> desired_speed,
                          float desiredTwistingSpeed,
                          float desiredBodyHeight,
                          std::string configFilepath);

    virtual ~qrStanceLegController();
    
    /**
     * @brief Reset the parameters of the qrStanceLegController.
     * @param currentTime Current run time
     */
    virtual void Reset(float currentTime);
    
    /**
     * @brief Update the parameters of the qrStanceLegController.
     */
    virtual void Update();

    /**
     * @brief Update ratio
     * @param contacts The status of each leg contacts with ground.
     * @param N The number of leg which contacts with ground.
     * @param moveBasePhase 
     */
    void UpdateFRatio(Vec4<bool> &contacts, int &N, float &moveBasePhase);

    /** @brief Compute all motors' commands via controllers.
     *  @return tuple<map, Matrix<3,4>> : 
     *          return control ouputs (e.g. positions/torques) for all (12) motors.
     */
    std::tuple<std::map<int, qrMotorCmd>, Eigen::Matrix<float, 3, 4>> GetAction();

private:

    /**
     * @brief The robot object pointer.
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
     * @brief Gait Generator object pointer.
     */
    qrGaitGenerator *gaitGenerator;

    /**
     * @brief Robot estimator pointre. Get the estimated velocity.
     */
    qrRobotVelocityEstimator *robotVelocityEstimator;

    /**
     * @brief Ground estimator pointer.
     */
    qrGroundSurfaceEstimator *groundEstimator;

    /**
     * @brief The center of mass adjuster pointer. Get the position of COM in base frame
     *        in position locomotion.
     */
    qrComPlanner *comPlanner;

    /**
     * @brief Robot pose planner obejct pointer. Get the intermediate base pose.
     */
    qrPosePlanner *posePlanner;

    /**
     * @brief Robot's foothold planner. Get desired COM pose when in walk locomotion.
     */
    qrFootholdPlanner *footholdPlanner;

    /**
     * @brief Desired speed of the robot in walk or position locomotion.
     */
    Eigen::Matrix<float, 3, 1> desiredSpeed = {0., 0., 0.};

    /**
     * @brief The speed message of twist command given by gamepad.
     */
    float desiredTwistingSpeed = 0.;

    /**
     * @brief Desired robot's body height. Overwrite in the class constructor by robot->bodyHeight
     */
    float desiredBodyHeight = 0.45;

    /**
     * @brief File path of stance_leg_controller.yaml.
     */
    std::string configFilepath;

    /**
     * @brief The parameter KP in stance_leg_controller.yaml.
     */
    Eigen::Matrix<float, 6, 1> KP;

    /**
     * @brief The parameter KD in stance_leg_controller.yaml.
     */
    Eigen::Matrix<float, 6, 1> KD;

    /**
     * @brief The parameter maxDdq in stance_leg_controller.yaml.
     */
    Eigen::Matrix<float, 6, 1> maxDdq;

    /**
     * @brief The parameter minDdq in stance_leg_controller.yaml.
     */
    Eigen::Matrix<float, 6, 1> minDdq;

    /**
     * @brief The parameter accWeight in stance_leg_controller.yaml.
     */
    Eigen::Matrix<float, 6, 1> accWeight;

    /**
     * @brief The minimum ratio.
     */
    Vec4<float> fMinRatio;

    /**
     * @brief The maximum ratio.
     */
    Vec4<float> fMaxRatio;

    /**
     * @brief Current time.
     */
    float currentTime;

    /**
     * @brief The time when running Reset().
     */
    float resetTime;
    
    /**
     * @brief The time after running Reset().
     */
    float timeSinceReset;
};

#endif //QR_STANCE_LEG_CONTROLLER_H

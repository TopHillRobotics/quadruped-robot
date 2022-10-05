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

#ifndef QR_STANCE_LEG_CONTROLLER_H
#define QR_STANCE_LEG_CONTROLLER_H

#include "common/qr_se3.h"
#include "robots/qr_robot.h"
#include "planner/qr_gait_generator.h"
#include "state_estimator/qr_robot_estimator.h"
#include "state_estimator/qr_ground_estimator.h"
#include "planner/qr_com_planner.h"
#include "planner/qr_foothold_planner.h"


/**
 * @brief Control stance leg of robot
 */
class qrStanceLegController {
public:

    /**
     * @brief Constructor of qrStanceLegController using given many object pointers and attributes.
     * @param robot The robot object pointer.
     * @param gaitGenerator The gait generator object pointer.
     * @param robotVelocityEstimator The robot estimator object pointer.
     * @param groundEstimatorIn The ground estimator object pointer.
     * @param comPlanner The com planner object pointer.
     * @param footholdPlanner The foothold planner object pointer.
     * @param desired_speed The robot desired speed in velocity control.
     * @param desiredTwistingSpeed The robot desired twisting speed in velocity control.
     * @param desiredBodyHeight The robot desired body height.
     * @param numLegs The number of leg.
     * @param configFilepath The stance leg config file path.
     * @param frictionCoeffs The coefficients of friction.
     */
    qrStanceLegController(qrRobot *robot,
                          qrGaitGenerator *gaitGenerator,
                          qrRobotEstimator *robotVelocityEstimator,
                          qrGroundSurfaceEstimator *groundEstimatorIn,
                          qrComPlanner *comPlanner,
                          qrFootholdPlanner *footholdPlanner,
                          Eigen::Matrix<float, 3, 1> desired_speed,
                          float desiredTwistingSpeed,
                          float desiredBodyHeight,
                          int numLegs,
                          std::string configFilepath,
                          std::vector<float> frictionCoeffs = {0.45, 0.45, 0.45, 0.45});


    /**
     * @brief factory function for stance controller
     * @param useMPC: whether use model predictive control
     * @return stance controller
     */
    static qrStanceLegController* createStanceController(qrRobot *robot,
                                                         qrGaitGenerator *gaitGenerator,
                                                         qrRobotEstimator *robotEstimator,
                                                         qrGroundSurfaceEstimator *groundEstimatorIn,
                                                         qrComPlanner *comPlanner,
                                                         qrFootholdPlanner *footholdPlanner,
                                                         Eigen::Matrix<float, 3, 1> desiredSpeed,
                                                         float desiredTwistingSpeed,
                                                         float desiredBodyHeight,
                                                         int numLegs,
                                                         std::string configFilepath,
                                                         std::vector<float> frictionCoeffs,
                                                         bool useMPC);

    /**
     * @brief Default desconstructor of qrStanceLegController
     */
    virtual ~qrStanceLegController() = default;

    /**
     * @brief Reset the parameters of the qrStanceLegController.
     * @param currentTime Current run time
     */
    void Reset(float currentTime);

    /**
     * @brief update the ratio of the friction force to robot gravity
     * @param contacts Vec4<bool>&,  descripte the contact status of four feet
     * @param N int&, the number of contact feet
     * @param normalizedPhase float&, the phase of swing leg
     */
    void UpdateFRatio(Vec4<bool> &contacts, int &N, float &normalizedPhase);

    /**
     * @brief Update the parameters of the qrStanceLegController.
     * @param currentTime Current run time
     */
    void Update(float currentTime);

    /**
     * @brief The process of velocity locomotion.
     * @param robotComOrientation The orientation of the robot's COM.
     * @param robotComPosition The position of the robot's COM.
     * @param robotComVelocity The velocity of the robot's COM.
     * @param robotComRpy The rpy of the robot's COM.
     * @param robotComRpyRate The pry rate of the robot's COM.
     * @param desiredComPosition The desired position of the robot's COM.
     * @param desiredComVelocity The desired velocity of the robot's COM.
     * @param desiredComRpy The desired rpy of the robot's COM.
     * @param desiredComAngularVelocity The desired angular velocity of the robot's COM.
     */
    void VelocityLocomotionProcess(Quat<float> &robotComOrientation,
                                    Eigen::Matrix<float, 3, 1> &robotComPosition, 
                                    Eigen::Matrix<float, 3, 1> &robotComVelocity,
                                    Eigen::Matrix<float, 3, 1> &robotComRpy,
                                    Eigen::Matrix<float, 3, 1> &robotComRpyRate,
                                    Eigen::Matrix<float, 3, 1> &desiredComPosition,
                                    Eigen::Matrix<float, 3, 1> &desiredComVelocity,
                                    Eigen::Matrix<float, 3, 1> &desiredComRpy,
                                    Eigen::Matrix<float, 3, 1> &desiredComAngularVelocity);

    /**
     * @brief The process of position locomotion.
     * @param robotComPosition The current position of the robot's COM.
     * @param robotComPosition The position of the robot's COM.
     * @param robotComVelocity The velocity of the robot's COM.
     * @param robotComRpy The rpy of the robot's COM.
     * @param desiredComAngularVelocity The desired angular velocity of the robot's COM.
     */
    void PositionLocomotionProcess(Eigen::Matrix<float, 3, 1> &robotComPosition,
                                    Eigen::Matrix<float, 3, 1> &desiredComPosition,
                                    Eigen::Matrix<float, 3, 1> &desiredComVelocity,
                                    Eigen::Matrix<float, 3, 1> &desiredComRpy,
                                    Eigen::Matrix<float, 3, 1> &desiredComAngularVelocity);

    /** 
     * @brief Compute all motors' commands via controllers.
     *  @return tuple<map, Matrix<3,4>> : 
     *          return control ouputs (e.g. positions/torques) for all (12) motors.
     */
    virtual std::tuple<std::map<int, qrMotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

    /**
     * @brief update linear velocity and angular velocity of controllers
     * @param linSpeed: linear velocity
     * @param angSpeed: yaw twist velocity
     */
    virtual void UpdateControlParameters(const Eigen::Vector3f& linSpeed, const float& angSpeed);

    /**
     * @brief The robot object pointer.
     */
    qrRobot *robot;

    /**
     * @brief Gait Generator object pointer.
     */
    qrGaitGenerator *gaitGenerator;

    /**
     * @brief Robot estimator pointre. Get the estimated velocity.
     */
    qrRobotEstimator *robotEstimator;

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
    float desiredBodyHeight = 0.28; //overwrite in the class constructor by robot->bodyHeight
    
    /**
     * @brief The number of legs.
     */
    int numLegs = 4;

    /**
     * @brief The coefficients of friction.
     */
    std::vector<float> frictionCoeffs = {0.45, 0.45, 0.45, 0.45};

    /**
     * @brief File path of stance_leg_controller.yaml.
     */
    std::string configFilepath;

    /**
     * @brief The dimension of force.
     */
    int force_dim;

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
    Vec4<float> fMinRatio; // the minimum ratio

    /**
     * @brief The maximum ratio.
     */
    Vec4<float> fMaxRatio; // the maximum ratio

    /**
     * @brief Current time.
     */
    float currentTime;
};

#endif //QR_STANCE_LEG_CONTROLLER_H

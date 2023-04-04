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

#ifndef QR_GAIT_GENERATOR_H
#define QR_GAIT_GENERATOR_H

#include "robots/qr_robot.h"

/**
 * @brief The qrGaitGenerator class generates the corresponding gait
 * uisng the defined gait parameters. This class also resets and updates these parameters regularly.
 */
namespace Quadruped {

class qrGaitGenerator {

public:
    /**
    * @brief Default constructor that constructs a qrGaitGenerator object.
    */
    qrGaitGenerator() {}

    /**
     * @brief Construct a qrGaitGenerator object using a given config file.
     * @param robot: robot the class of robot state.
     * @param configFilePath: configFilePath the given config file.
     */
    qrGaitGenerator(qrRobot *robot, std::string configFilePath);

    /**
     * @brief Construct a qrGaitGenerator object using the given parameters.
     * @param robot: the class of robot state.
     * @param stanceDuration: specifies the amount of stance time for each leg in a gait cycle.
     * @param dutyFactor: specifies the duty factor for each leg. dutyFactor represents the fraction of stance phase in the gait cycle.
     * @param initialLegState: specifies the state (SWING or STANCE) of each leg at the initialization of generating a gait.
     * @param initialLegPhase: specifies the relative phase for each leg at the initialization of generating a gait.
     * @param contactDetectionPhaseThreshold: specifies the contact threshold when the leg state switches from SWING to STANCE.
     */
    qrGaitGenerator(qrRobot *robot,
                Eigen::Matrix<float, 4, 1> stanceDuration,
                Eigen::Matrix<float, 4, 1> dutyFactor,
                Eigen::Matrix<int, 4, 1> initialLegState,
                Eigen::Matrix<float, 4, 1> initialLegPhase,
                float contactDetectionPhaseThreshold=0.1f);

    /**
     * @brief Deconstruct a qrGaitGenerator object.
     */
    virtual ~qrGaitGenerator() = default;

    /**
     * @brief Reset the gait parameters using the given time.
     * @param currentTime the given time.
     */
    virtual void Reset(float currentTime)
    {
        resetTime = currentTime;
        lastTime = currentTime;
        normalizedPhase.setZero();
        curLegState = initialLegState;
        lastLegState = curLegState;
        legState = curLegState;
        desiredLegState = curLegState;
        firstStanceAngles = robot->standUpMotorAngles;
        contactStartPhase.setZero();
    };

    /**
    * @brief Update of the gait parameters using the given time.
    * @param currentTime: the given time.
    */
    virtual void Update(float currentTime) = 0;

    /**
     * @brief If foot does not get into desired state, how to plan the next desired state.
     * @param currentTime: the given time.
     */
    virtual void Schedule(float currentTime) {}

    /**
     * @brief The yaml object for loading a yaml config file.
     */
    YAML::Node config;

    /**
     * @brief Robot object.
     */
    qrRobot *robot;

    /**
     * @brief Gait name.
     */
    std::string gait;

    /**
     * @brief The time when exec reset.
     */
    float resetTime;

    /**
     * @brief The time since exec reset.
     */
    float timeSinceReset;

    /**
     * @brief The time when exec reset.
     */
    float lastTime;

    /**
     * @brief The amount of stance time for each leg in a gait cycle.
     */
    Eigen::Matrix<float, 4, 1> stanceDuration;

    /**
     * @brief The amount of swing time for each leg in a gait cycle.
     */
    Eigen::Matrix<float, 4, 1> swingDuration;

    /**
     * @brief The fraction of the cycle for stance phase.
     * dutyFactor = stanceDuration / (stanceDuration + swingDuration).
     * @note In a periodic gait, dutyFactor is the same for all the legs.
     */
    Eigen::Matrix<float, 4, 1> dutyFactor;

    /**
     * @brief Current phase for each leg in full gait cycle.
     */
    Eigen::Matrix<float, 4, 1> phaseInFullCycle;

    /**
     * @brief The relative phase for each leg at the initialization of generating a gait.
     * @note The one leg is assigned relative phase 0 and the others have the relative phases in the range [0,1).
     */
    Eigen::Matrix<float, 4, 1> initialLegPhase;

    /**
     * @brief True global initial leg phase, [0, stance duty] for stance, [stance duty, 1.0] for swing.
     */
    Eigen::Matrix<float, 4, 1> offset;

    /**
     * @brief The new state of each leg when the current leg state switches, either from STAND to SWING, or from SWING to STAND.
     * @note If the current state is SWING, the next state will be STAND. If the current state is STANCE, the next state will be SWING.
     */
    Eigen::Matrix<int, 4, 1> initialLegState;

    /**
     * @brief The new state of each leg when the current leg state switches, either from STAND to SWING, or from SWING to STAND.
     * @note If the current state is SWING, the next state will be STAND. If the current state is STANCE, the next state will be SWING.
     */
    Eigen::Matrix<int, 4, 1> nextLegState;

    /**
     * @brief The current state of each leg (either STANCE or SWING).
     */
    Eigen::Matrix<int, 4, 1> legState;

    /**
     * @brief the relative phase for the desired state.
     * @note If current state is SWING, then the normalized leg phase means (time in swing)/(total swing time), same as STANCE.
     */
    Eigen::Matrix<float, 4, 1> normalizedPhase;

    /**
     * @brief the desired state (either STANCE or SWING) of each leg at a given moment.
     * e.g. if legState = STANCE and normalizedLegPhase < 1.0,  desiredLegState = legState.
     * e.g. if legState = STANCE and normalizedLegPhase >= 1.0, desiredLegState = nextLegState.
     */
    Eigen::Matrix<int, 4, 1> desiredLegState;

    /**
     * @brief the previous state of each leg,
     * @note This is used to determine if the state changed in Velocity Mode.
     */
    Eigen::Matrix<int, 4, 1> lastLegState;

    /**
     * @brief The previous  (current) states of each leg,
     * @note this is used to determine if the state changed in Position Mode or Walk Mode.
     */
    Eigen::Matrix<int, 4, 1> curLegState; // curLegState is the planned current state, while legState is the state actually detected via senros.

    /**
     * @brief the fraction of the cycle for stance phase or for swing phase at a moment.
     * @note If legState = STANCE, the value is dutyfactor; if legState = SWING, the value is (1 - dutyfactor).
     */
    Eigen::Matrix<float, 4, 1> initStateRadioInCycle;

    /**
     * @brief the duration of a gait cycle. fullCyclePeriod = stanceDuration + swingDuration.
     */
    Vec4<float> fullCyclePeriod;

    /**
     * @brief if allow switch leg state.
     */
    Vec4<bool> allowSwitchLegState;

    /**
     * @brief the contact threshold when the leg state switches from SWING to STANCE.
     */
    float contactDetectionPhaseThreshold;

    /**
     * @brief the ratio means percentage of cycle, so the variable means during.
     * which part of the cycle to move the base.
     */
    std::vector<float> moveBaseRatioPoint;

    /**
     * @brief the amount of time it takes to move the torso(base) in stance period.
     */
    float moveBaseTime;

    /**
     * @brief the relative time during moving the base.
     */
    float moveBasePhase;

    /**
     * @brief the initial phase when 'trueSwing' gait start in a swing cycle.
     * @note a total cycle include swing and stance, what's more, the stance.
     * period consists of 'load_force' 'full_stance' 'unload_stance'.
     */
    float trueSwingStartPhaseInSwingCycle;

    /**
     * @brief the initial time when 'trueSwing' gait start in a full cycle.
     */
    float trueSwingStartPhaseInFullCycle;

    /**
     * @brief the end time when 'trueSwing' gait start in a full cycle.
     */
    float trueSwingEndPhaseInFullCycle;

    /**
     * @brief how many gait cycle generated.
     */
    unsigned long gaitCycle=0;

    /**
     * @brief leg state the robot detected, e.g.if the leg contact when the gait is swing,
     * then the leg may in a 'early contact' state.
     */
    Eigen::Matrix<int, 4, 1> detectedLegState;

    /**
     * @brief the time when event(early contact or lost contact) happend in a full cycle.
     */
    Vec4<float> detectedEventTickPhase;

    /**
      * @brief if the leg state should change to swing.
      */
    Vec4<bool> firstSwing = {false, false, false, false};

    /**
     * @brief leg contact time in full cycle.
     */
    Vec4<float> contactStartPhase;

    /**
      * @brief remain time when the ture swing finished.
      */
    Vec4<float> swingTimeRemaining = {0.f,0.f,0.f,0.f};

    /**
      * @brief if the leg finished its first stance gait.
      */
    Vec4<bool> firstStance = {false, false, false, false};

    /**
     * @brief target joint angles of robot for stance.
     */
    Eigen::Matrix<float,12,1> firstStanceAngles;
};

} // Namespace Quadruped

#endif // QR_GAIT_GENERATOR_H

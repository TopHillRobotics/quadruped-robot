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

#ifndef QR_WALK_GAIT_GENERATOR_H
#define QR_WALK_GAIT_GENERATOR_H

#include "robots/qr_robot.h"
#include "gait/qr_gait.h"


namespace Quadruped {

/**
 * @brief The gait generator for walk mode.
 */
class qrWalkGaitGenerator: public qrGaitGenerator {

public:

    /**
     * @brief Constructor of WalkGaitGenerator class.
     */
    qrWalkGaitGenerator() {};

    /**
     * @brief Constructor of WalkGaitGenerator class.
     * @param robot: robot for generate desired gait.
     * @param configFilePath: configFilePath the given config file.
     */
    qrWalkGaitGenerator(qrRobot *robot, std::string configFilePath);

    /**
     * @brief Constructor of WalkGaitGenerator class.
     * @param robot: the class of robot state.
     * @param stateSwitchQue: store the order of the leg substates.
     * @param stateRatioQue: store the proportion in a cycle of every leg substate.
     * @param stanceDuration: stanceDuration specifies the amount of stance time for each leg in a gait cycle.
     * @param dutyFactor: dutyFactor specifies the duty factor for each leg. dutyFactor represents the fraction of stance phase in the gait cycle.
     * @param initialLegState: initialLegState specifies the state (SWING or STANCE) of each leg at the initialization of generating a gait.
     * @param initialLegPhase: initialLegPhase specifies the relative phase for each leg at the initialization of generating a gait.
     * @param contactDetectionPhaseThreshold: contactDetectionPhaseThreshold specifies the contact threshold when the leg state switches from SWING to STANCE.
     */
    qrWalkGaitGenerator(qrRobot *robot,
                      std::vector<SubLegState> stateSwitchQue,
                      std::vector<float> stateRatioQue,
                      Eigen::Matrix<float, 4, 1> stanceDuration,
                      Eigen::Matrix<float, 4, 1> dutyFactor,
                      Eigen::Matrix<int, 4, 1> initialLegState,
                      Eigen::Matrix<float, 4, 1> initialLegPhase,
                      float contactDetectionPhaseThreshold=0.1f);

    /**
     * @brief Dstructor of should start from stand stateWalkGaitGenerator class.
     */
    virtual ~qrWalkGaitGenerator() = default;

    /**
     * @brief Reset the gait parameters using the given time.
     * @param currentTime the given time.
     */
    virtual void Reset(float currentTime);

    /**
     * @brief Update of the gait parameters and leg states using the given time.
     * @param currentTime: the given time.
     */
    virtual void Update(float currentTime);

//private:

    /**
     * @brief Store the order of the leg substates.
     * it should start from stand state.
     */
    std::vector<SubLegState> stateSwitchQue;

    /**
     * @brief Store the time proportion in a cycle of every leg substate.
     */
    std::vector<float> stateRatioQue;

    /**
     * @brief Accumulation of state ration in complete walk cycle.
     */
    std::vector<float> accumStateRatioQue;

    /**
     * @brief Index in the stateSwitchQue, used for changing substate.
     */
    Vec4<int> stateIndexOfLegs;

    /**
     * @brief Current substate ratio in a cycle.
     */
    Vec4<float> curStateRadio;

    /**
     * @brief When "trueSwing" sub state start in swing cycle.
     */
    float trueSwingStartPhaseInSwingCycle;


};
} // Namespace Quadruped

#endif //QR_WALK_GAIT_GENERATOR_H

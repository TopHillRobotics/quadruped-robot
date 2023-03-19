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

#ifndef QR_CONTROL_FSM_H
#define QR_CONTROL_FSM_H

#include <iostream>

#include "qr_safety_checker.hpp"
#include "qr_control_fsm_data.hpp"

#include "qr_fsm_state_locomotion.hpp"
#include "qr_fsm_state_passive.hpp"
#include "qr_fsm_state_standup.hpp"

/**
 * @brief Enumerate all of the operating modes.
 */
enum class FSM_OperatingMode {
    NORMAL,
    TRANSITIONING,
    ESTOP,
    EDAMP
};

template<typename T>
struct qrFSMStatesList {

    qrFSMState<T> *invalid;

    qrFSMStatePassive<T>* passive;

    qrFSMStateStandUp<T>* standUp;

    qrFSMStateLocomotion<T> *locomotion;

};

/**
 * @brief Control FSM handles the FSM states from a higher level.
 */
template<typename T>
class qrControlFSM {

public:

    /**
     * @brief Constructor of class qrControlFSM.
     * @param quadruped: pointer to Robot.
     * @param stateEstimator: pointer to state estimator container.
     * @param gaitScheduler: pointer to gait scheduler.
     * @param desiredStateCommand: pointer to desired state command.
     * @param userParameters: pointer to user parameters.
     */
    qrControlFSM(Quadruped::qrRobot *quadruped,
                 Quadruped::qrStateEstimatorContainer *stateEstimator,
                 Quadruped::qrGaitGenerator *gaitScheduler,
                 Quadruped::qrDesiredStateCommand *desiredStateCommand,
                 qrUserParameters *userParameters);

    ~qrControlFSM() = default;

    /**
     * @brief Initializes the ControlFSM.
     * The quadruped will enter standup state.
     */
    void Initialize();

    /**
     * @brief Reset the times of ControlFSM.
     * @param currentTime: current time to reset ControlFSM.
     */
    void Reset(float currentTime) {
        resetTime = currentTime;
        timeSinceReset = 0;
        statesList.locomotion->Reset(currentTime);
    }

    /**
     * @brief Step the ControlFSM once. It will be in one state or on transition.
     * @param hybridAction: A return value of the desired motor command.
     */
    void RunFSM(std::vector<Quadruped::qrMotorCommand> &hybridAction);

    /**
     * @brief Check the robot state before the calculation of commands in this control loop.
     * @todo Remove this function or move it into a SaftyCheck class.
     * @return operating mode
     */
    FSM_OperatingMode SafetyPreCheck();

    /**
     * @brief Check the robot state after the desired command has calculated in this control loop.
     * @return operating mode
     */
    FSM_OperatingMode SafetyPostCheck();

    /**
     * @brief Get next FSM state pointer by the required state name.
     * @param stateName: the name of the state needed.
     * @return a pointer to the FSM state in the list of created states
     */
    qrFSMState<T> *GetNextState(FSM_StateName stateName);

    /**
     * @brief Getter method of the locomotion controller in locomotion state.
     * @return pointer to locomotionController
     */
    Quadruped::qrLocomotionController* GetLocomotionController() const {
        return statesList.locomotion->GetLocomotionController();
    }

    /**
     * @brief Print current FSM status
     * @param opt: the options of printing the status
     */
    void PrintInfo(int opt);

private:

    /**
     * @brief Operating mode of the FSM.
     */
    FSM_OperatingMode operatingMode;

    /**
     * @brief Data needed for FSM.
     */
    qrControlFSMData<T> data;

    /**
     * @brief Check inputs and calculated commands.
     */
    qrSafetyChecker<T> *safetyChecker;

    /**
     * @brief Stores data when transitioning.
     */
    qrTransitionData<T> transitionData;

    /**
     * @brief Contains all the FSM states.
     */
    qrFSMStatesList<T> statesList;

    /**
     * @brief Current FSM state.
     */
    qrFSMState<T> *currentState;

    /**
     * @brief Next FSM state.
     */
    qrFSMState<T> *nextState;

    /**
     * @brief Name of next FSM state.
     */
    FSM_StateName nextStateName;

    /**
     * @brief The time of resetting the FSM.
     */
    float resetTime;

    /**
     * @brief Time pased since resetting the FSM.
     */
    float timeSinceReset;
    /**
     * @brief Print information every %printNUM iterations.
     */
    int printNum = 10000;

    /**
     * @brief Current normal printing count.
     */
    int printIter = 0;

    /**
     * @brief Current iteration of the FSM.
     */
    int iter = 0;

};

#endif // QR_CONTROL_FSM_H

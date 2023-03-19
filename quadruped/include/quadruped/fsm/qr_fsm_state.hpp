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

#ifndef QR_FSM_State_H
#define QR_FSM_State_H

#include <stdio.h>

#include "qr_control_fsm_data.hpp"
#include "qr_transition_data.hpp"
#include "action/qr_action.h"


/* Robot states corresponding to RC modes.
 * Some states are defined by MIT cheetah3 but not used in our project.
 */
#define K_STAND_DOWN        -1
#define K_PASSIVE           0
#define K_STAND_UP          1
#define K_BALANCE_STAND     2
#define GAIT_TRANSITION     3 // when K_LOCOMOTION

#define K_LOCOMOTION        4
#define K_LOCOMOTION_TEST   5
#define K_RECOVERY_STAND    6

#define LOCOMOTION_STAND    7 // when K_LOCOMOTION

#define K_VISION            8
#define K_BACKFLIP          9
#define K_FRONTJUMP         11

/* Specific control states. */
#define K_JOINT_PD          51
#define K_IMPEDANCE_CONTROL 52

#define K_INVALID           100

/**
 * @brief All FSM states in the state machine.
 */
enum class FSM_StateName {
    INVALID,
    PASSIVE,
    JOINT_PD,
    IMPEDANCE_CONTROL,
    STAND_UP,
    STAND_DOWN,
    BALANCE_STAND,
    LOCOMOTION,
    RECOVERY_STAND,
    VISION,
    BACKFLIP,
    FRONTJUMP
};

template<typename T>
class qrFSMState {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Constructor of FSM_State.
     * @param controlFSMData: data needed by the state. Usually some pointers to components in control loop.
     * The state will transfer the command calculated in this control loop to this %controlFSMData.
     * %controlFSMData is a member of Control_FSM
     * @param stateNameIn: an enum tyoe, the name of the state to be constructed.
     * @param stateStringIn: a string type, the string of the name of the state.
     */
    qrFSMState(qrControlFSMData<T> *controlFSMData, FSM_StateName stateNameIn,
              std::string stateStringIn);

    /**
     * @brief Do something when entering a state.
     * @attention Pure virtual method,
     */
    virtual void OnEnter() = 0;

    /**
     * @brief Normal behavior for the state.
     */
    virtual void Run() = 0;

    /**
     * @brief Check next transition of current state.
     * @return name of the next state
     */
    virtual FSM_StateName CheckTransition() {
        return FSM_StateName::INVALID;
    }

    /**
     * @brief Runs the transition behaviors and returns true when done transitioning.
     * @return whether transition is done
     */
    virtual qrTransitionData<T> Transition() {
        return transitionData;
    }

    /**
     * @brief Do something when existing the state.
     */
    virtual void OnExit() = 0;

    /**
     * @brief Turn on all safety checks.
     */
    void TurnOnAllSafetyChecks();

    /**
     * @brief Turn off all safety checks.
     */
    void TurnOffAllSafetyChecks();

    /**
     * @brief Enum type of the current state.
     */
    FSM_StateName stateName;

    /**
     * @brief Enum type of the next state.
     */
    FSM_StateName nextStateName;

    /**
     * @brief String of the current state.
     */
    std::string stateString;

    /**
     * @brief Time consumed by transition.
     */
    T transitionDuration;

    /**
     * @brief Time of the start of the transition.
     */
    T tStartTransition;

    /**
     * @brief Check roll and pitch.
     * This is a pre-control safety check.
     */
    bool checkSafeOrientation = false;

    /**
     * @brief Check for footstep distance
     * This is a post-control safety check.
     */
    bool checkPDesFoot = false;

    /**
     * @brief Check for executed force. The force shouldn't be so large.
     * This is a post-control safety check.
     */
    bool checkForceFeedForward = false;

    /**
     * @brief Check one-stance leg condition
     * This is a post-control safety check.
     */
    bool checkLegSingularity = false;

protected:

    /**
     * @brief Holds all of the relevant control data.
     */
    qrControlFSMData<T> *_data;

    /**
     * @brief Data needed for transition.
     */
    qrTransitionData<T> transitionData;

    /**
     * @brief Record the time when resetting this state.
     */
    float resetTime;

    /**
     * @brief Record the time since resetting this state.
     */
    float timeSinceReset;

};

#endif// QR_FSM_State_H

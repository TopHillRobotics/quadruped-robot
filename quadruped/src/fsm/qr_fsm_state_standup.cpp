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

#include "fsm/qr_fsm_state_standup.hpp"


template<typename T>
qrFSMStateStandUp<T>::qrFSMStateStandUp(qrControlFSMData<T> *_controlFSMData)
    : qrFSMState<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP")
{
    /* Disable the safety checks. Standup state do not need these check. */
    this->checkSafeOrientation = false;
    this->checkPDesFoot = false;
    this->checkForceFeedForward = false;
    
    isUp = true;
}


template<typename T>
void qrFSMStateStandUp<T>::OnEnter()
{
    printf("in OnEnter\n");
    printf("ENTER the standup fsm!!!\n");

    this->nextStateName = this->stateName;

    this->transitionData.Zero();

    standUp = 1;

    this->_data->quadruped->fsmMode = K_STAND_UP;
}


template<typename T>
void qrFSMStateStandUp<T>::Run()
{
    /* stand up, keep position or sit down according to the value of %standup. */
    if (standUp == 1) {
        Quadruped::Action::StandUp(this->_data->quadruped, 2.f, 3.f, 1.0f / this->_data->userParameters->controlFrequency);
        motorAngles = this->_data->quadruped->standUpMotorAngles;
        isUp = true;
    } else if (standUp == -1) {
        Quadruped::Action::SitDown(this->_data->quadruped, 1.5f, 1.0f / this->_data->userParameters->controlFrequency);
        motorAngles = this->_data->quadruped->sitDownMotorAngles;
        isUp = false;
    }
    /* After the action, keep current position */
    standUp = 0;

    if (this->_data->legCmd.size() != NumMotor) {
        this->_data->legCmd.clear();
        for (int i(0); i<NumMotor; ++i)
            this->_data->legCmd.push_back(Quadruped::qrMotorCommand());
    }

    for(int i(0); i< NumMotor; ++i) {
        this->_data->legCmd[i] = {motorAngles[i], this->_data->quadruped->motorKps[i], 0, this->_data->quadruped->motorKds[i], 0};    
    }
}


template<typename T>
FSM_StateName qrFSMStateStandUp<T>::CheckTransition()
{
    this->nextStateName = this->stateName;

    /* The %fsmMode will be set according to joy RC mode. Check next state by %fsmMode. */
    switch (int(this->_data->quadruped->fsmMode)) {
    case K_STAND_UP:
        if (!isUp)
            standUp = 1;
        break;
    case K_STAND_DOWN:
        if (isUp)
            standUp = -1;
        break;
    case K_BALANCE_STAND:
        this->nextStateName = FSM_StateName::BALANCE_STAND;
        break;
    case K_LOCOMOTION:
        this->nextStateName = FSM_StateName::LOCOMOTION;
        break;
    case LOCOMOTION_STAND:
        this->nextStateName = FSM_StateName::LOCOMOTION;
        break;
    case GAIT_TRANSITION:
        this->nextStateName = FSM_StateName::LOCOMOTION;
        break;
    case K_VISION:
        this->nextStateName = FSM_StateName::VISION;
        break;
    case K_PASSIVE:
        this->nextStateName = FSM_StateName::PASSIVE;
        break;
    default:
        std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                  << K_STAND_UP << " to "
                  << this->_data->quadruped->fsmMode << std::endl;
    }

    return this->nextStateName;
}


template<typename T>
qrTransitionData<T> qrFSMStateStandUp<T>::Transition()
{
    switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:
        this->transitionData.done = true;
        break;
    case FSM_StateName::BALANCE_STAND:
        this->transitionData.done = true;
        break;
    case FSM_StateName::LOCOMOTION:
        this->transitionData.done = true;
        this->transitionData.legCommand = this->_data->legCmd;
        break;
    case FSM_StateName::VISION:
        this->transitionData.done = true;
        break;
    default:
        printf("[CONTROL FSM] Something went wrong in transition\n");
    }

    /* Return transition data to the FSM. */
    return this->transitionData;
}


template<typename T>
void qrFSMStateStandUp<T>::OnExit()
{
    /* Standup state does nothing when exitting */
    printf("STAND_UP EXIT!\n");
}

template class qrFSMStateStandUp<float>;

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

#include "fsm/qr_control_fsm.hpp"


template<typename T>
qrControlFSM<T>::qrControlFSM(
    Quadruped::qrRobot *quadruped,
    Quadruped::qrStateEstimatorContainer *stateEstimators,
    Quadruped::qrGaitGenerator *gaitScheduler,
    Quadruped::qrDesiredStateCommand *desiredStateCommand,
    qrUserParameters *userParameters)
{
    data.quadruped = quadruped;
    data.stateEstimators = stateEstimators;
    data.gaitGenerator = gaitScheduler;
    data.desiredStateCommand = desiredStateCommand;
    data.userParameters = userParameters;

    /* Add all FSM states into the statelist and initialize the FSM. */
    statesList.invalid = nullptr;
    statesList.passive = new qrFSMStatePassive<T>(&data);
    statesList.standUp = new qrFSMStateStandUp<T>(&data);
    statesList.locomotion = new qrFSMStateLocomotion<T>(&data);

    safetyChecker = new qrSafetyChecker<T>(&data);

    Initialize();

    printf("FSM Init Finished");
}


template<typename T>
void qrControlFSM<T>::Initialize()
{
    /* set the FSM to standup state and set the operating mode to nomal. */
    currentState = statesList.standUp;
    currentState->OnEnter();
    nextState = currentState;
    operatingMode = FSM_OperatingMode::NORMAL;
}


template<typename T>
void qrControlFSM<T>::RunFSM(std::vector<Quadruped::qrMotorCommand>& hybridAction)
{

    /* If joy command has received, set up next FSM mode according to the received joy RC mode. */
    if(data.desiredStateCommand->getJoyCtrlStateChangeRequest()) {
        
        const Quadruped::RC_MODE ctrlState = data.desiredStateCommand->getJoyCtrlState();
        std::cout<< "[CONTROL FSM]: control mode = "<< ctrlState <<std::endl;
        if (ctrlState == Quadruped::RC_MODE::JOY_TROT ||
            ctrlState == Quadruped::RC_MODE::JOY_ADVANCED_TROT ||
            ctrlState == Quadruped::RC_MODE::JOY_WALK ||
            ctrlState == Quadruped::RC_MODE::HARD_CODE) {
            data.quadruped->fsmMode = GAIT_TRANSITION;
        } else if (ctrlState == Quadruped::RC_MODE::BODY_DOWN) {
            data.quadruped->fsmMode = K_STAND_DOWN;
        } else if (ctrlState == Quadruped::RC_MODE::BODY_UP) {
            data.quadruped->fsmMode = K_STAND_UP;
        } else if (ctrlState == Quadruped::RC_MODE::JOY_STAND) {
            data.quadruped->fsmMode = LOCOMOTION_STAND;
        } else if (ctrlState == Quadruped::RC_MODE::EXIT) {
            data.quadruped->fsmMode = K_PASSIVE;
        } else {
            throw std::domain_error("no such state!");
        }

        data.desiredStateCommand->setJoyCtrlStateChangeRequest(false);
    }

    if (operatingMode != FSM_OperatingMode::ESTOP) {

        /* Run normal controls of this state if no transition is detected.
         * If a transition is required, the %CheckTransition() will return a different state name.
         * Then the %operatingMode will set to TRANSITIONING
         */
        if (operatingMode == FSM_OperatingMode::NORMAL) {
            nextStateName = currentState->CheckTransition();
            if (nextStateName != currentState->stateName) {
                operatingMode = FSM_OperatingMode::TRANSITIONING;
                nextState = GetNextState(nextStateName);

            } else if (currentState->transitionDuration > 0.1) {
                operatingMode = FSM_OperatingMode::TRANSITIONING;
            } else {
                // Execute normal behaviour of the state.
                currentState->Run();
            }
        }

        /* If the state needs transitioning, current state will set up transition data. */
        if (operatingMode == FSM_OperatingMode::TRANSITIONING) {

            transitionData = currentState->Transition();
            data.legCmd = transitionData.legCommand;

            /* Check the robot state for safe operation. */
            SafetyPostCheck();

            /* After the transitioning(some transition requires duration), current state will set %done to true. */
            if (transitionData.done) {

                /* Exit the current state. */
                currentState->OnExit();

                /* Enter next state */
                currentState = nextState;
                currentState->OnEnter();
                operatingMode = FSM_OperatingMode::NORMAL;
            }
        } else {
            SafetyPostCheck();
        }

    } else {
        /* This part is used for emergency stop, will be added in the future. */
        currentState->OnEnter();
        nextStateName = currentState->stateName;
    }
            
    hybridAction = data.legCmd;
    
    /* Print the current state of the FSM if needed */
    // PrintInfo(0);

    iter++;
}


template<typename T>
FSM_OperatingMode qrControlFSM<T>::SafetyPreCheck()
{
    // Default is to return the current operating mode
    return operatingMode;
}


template<typename T>
FSM_OperatingMode qrControlFSM<T>::SafetyPostCheck()
{
    if (currentState->checkPDesFoot) {
        /* Foot should not step too far */
        safetyChecker->CheckPDesFoot();
    }

    if (currentState->checkForceFeedForward) {
        /* Force should not be too large. */
        safetyChecker->CheckForceFeedForward();
    }

    return operatingMode;
}


template<typename T>
qrFSMState<T> *qrControlFSM<T>::GetNextState(FSM_StateName stateName)
{
    /* Choose the correct FSM State by enumerated state name. */
    switch (stateName) {

        case FSM_StateName::INVALID:
          return statesList.invalid;

        case FSM_StateName::PASSIVE:
          return statesList.passive;

        case FSM_StateName::STAND_UP:
          return statesList.standUp;

        case FSM_StateName::LOCOMOTION:
            return statesList.locomotion;

        default:
            return statesList.invalid;
    }
}


template<typename T>
void qrControlFSM<T>::PrintInfo(int opt)
{
    switch (opt) {

    /* Normally print the information of FSM. */
    case 0:
        printIter++;
        if (printIter == printNum) {
            std::cout << "[CONTROL FSM] Printing FSM Info...\n";
            std::cout
                << "---------------------------------------------------------\n";
            std::cout << "Iteration: " << iter << "\n";
            if (operatingMode == FSM_OperatingMode::NORMAL) {
                std::cout << "Operating Mode: NORMAL in " << currentState->stateString
                          << "\n";

            } else if (operatingMode == FSM_OperatingMode::TRANSITIONING) {
                std::cout << "Operating Mode: TRANSITIONING from "
                          << currentState->stateString << " to "
                          << nextState->stateString << "\n";

            } else if (operatingMode == FSM_OperatingMode::ESTOP) {
                std::cout << "Operating Mode: ESTOP\n";
            }
            std::cout << std::endl;
            printIter = 0;
        }
        break;

    /* Print transition information before transition. */
    case 1:
        std::cout << "[CONTROL FSM] Transition initialized from "
                  << currentState->stateString << " to " << nextState->stateString
                  << "\n"
                  << std::endl;

        break;

    /* Print transition information after transition. */
    case 2:
        std::cout << "[CONTROL FSM] Transition finalizing from "
                  << currentState->stateString << " to " << nextState->stateString
                  << "\n"
                  << std::endl;

        break;
    }
}


template class qrControlFSM<float>;

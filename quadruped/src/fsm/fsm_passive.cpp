/*============================== Passive ==============================*/
/**
 * FSM State that calls no controls. Meant to be a safe state where the
 * robot should not do anything as all commands will be set to 0.
 */

#include "fsm/fsm_passive.hpp"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 * @param _controlFSMData holds all of the relevant control data
 */
template<typename T>
FSM_State_Passive<T>::FSM_State_Passive(ControlFSMData<T> *_controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::PASSIVE, "PASSIVE")
{
    // Do nothing
    // Set the pre controls safety checks
    this->checkSafeOrientation = false;

    // Post control safety checks
    this->checkPDesFoot = false;
    this->checkForceFeedForward = false;
}

template<typename T>
void FSM_State_Passive<T>::OnEnter()
{
    // Default is to not transition
    this->nextStateName = this->stateName;
    // Reset the transition data
    this->transitionData.zero();
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template<typename T>
void FSM_State_Passive<T>::Run()
{
    // Do nothing, all commands should begin as zeros
    //   testTransition();
    this->_data->legCmd.clear();
    for (int i(0); i < NumMotor; ++i) {
        this->_data->legCmd.push_back({0, 0, 0, 0, 0});
    }
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template<typename T>
TransitionData<T> FSM_State_Passive<T>::testTransition()
{
    this->transitionData.done = true;
    return this->transitionData;
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template<typename T>
FSM_StateName FSM_State_Passive<T>::CheckTransition()
{
    this->nextStateName = this->stateName;
    iter++;

    // Switch FSM control mode
    switch ((int)this->_data->quadruped->fsmMode) {
        case K_PASSIVE:// normal c (0)
            // Normal operation for state based transitions
            break;

        case K_JOINT_PD:
            // Requested switch to joint PD control
            this->nextStateName = FSM_StateName::JOINT_PD;
            break;

        case K_STAND_UP:
            // Requested switch to joint PD control
            this->nextStateName = FSM_StateName::STAND_UP;
            break;

        case K_RECOVERY_STAND:
            // Requested switch to joint PD control
            this->nextStateName = FSM_StateName::RECOVERY_STAND;
            break;

        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_PASSIVE << " to "
                      << this->_data->quadruped->fsmMode << std::endl;
    }

    // Get the next state
    return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template<typename T>
TransitionData<T> FSM_State_Passive<T>::Transition()
{
    // Finish Transition
    this->transitionData.done = true;

    // Return the transition data to the FSM
    return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template<typename T>
void FSM_State_Passive<T>::OnExit()
{
    // Nothing to clean up when exiting
}

// template class FSM_State_Passive<double>;
template class FSM_State_Passive<float>;

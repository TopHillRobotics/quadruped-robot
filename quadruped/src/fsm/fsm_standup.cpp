/*============================= Stand Up ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "fsm/fsm_standup.hpp"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 * @param _controlFSMData holds all of the relevant control data
 */
template<typename T>
FSM_State_StandUp<T>::FSM_State_StandUp(ControlFSMData<T> *_controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"),
      _ini_foot_pos(4)
{
    // Do nothing
    // Set the pre controls safety checks
    this->checkSafeOrientation = false;

    // Post control safety checks
    this->checkPDesFoot = false;
    this->checkForceFeedForward = false;
    
    isUp = true;
}

template<typename T>
void FSM_State_StandUp<T>::OnEnter()
{
    std::cout << "in OnEnter" << std::endl;
    ROS_INFO("ENTER the standup fsm!!!");
    // Default is to not transition
    this->nextStateName = this->stateName;

    // Reset the transition data
    this->transitionData.zero();
    standUp = 1;
    // Reset iteration counter
    iter = 0;

    for (size_t leg(0); leg < 4; ++leg) {
        _ini_foot_pos[leg] = this->_data->quadruped->GetFootPositionsInWorldFrame().col(leg);
    }
    this->_data->quadruped->fsmMode = K_STAND_UP;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template<typename T>
void FSM_State_StandUp<T>::Run()
{
    // std::cout << "standUp = " << standUp << std::endl;
    if (standUp == 1) {
        // std::cout << "begin fsm standup >>> " << std::endl;
        Quadruped::Action::StandUp(this->_data->quadruped, 2.f, 3.f, 1.0f / this->_data->userParameters->controlFrequency);
        motorAngles = this->_data->quadruped->standUpMotorAngles;
        isUp = true;
    } else if (standUp == -1) {
        // std::cout << "begin fsm sit down >>> " << std::endl;
        Quadruped::Action::SitDown(this->_data->quadruped, 1.5f, 1.0f / this->_data->userParameters->controlFrequency);
        motorAngles = this->_data->quadruped->sitDownMotorAngles;
        isUp = false;
    }
    
    standUp = 0; // keep current position

    if (this->_data->legCmd.size() != NumMotor) {
        this->_data->legCmd.clear();
        for (int i(0); i<NumMotor; ++i)
            this->_data->legCmd.push_back(Quadruped::MotorCommand());
    }
    for(int i(0); i< NumMotor; ++i) {
        this->_data->legCmd[i] = {motorAngles[i], this->_data->quadruped->motorKps[i], 0, 0, 0};    
    }
    // std::cout << "legCmd.size() =" << this->_data->legCmd.size()  << ", cmd[0]=" << this->_data->legCmd[0] << std::endl;
    
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template<typename T>
FSM_StateName FSM_State_StandUp<T>::CheckTransition()
{
    this->nextStateName = this->stateName;
    iter++;

    // Switch FSM control mode
    switch ((int)this->_data->quadruped->fsmMode) {
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
        case K_PASSIVE:// normal c
            this->nextStateName = FSM_StateName::PASSIVE;
            break;
        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_STAND_UP << " to "
                      << this->_data->quadruped->fsmMode << std::endl;
    }

    // Get the next state
    return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 * @return true if transition is complete
 */
template<typename T>
TransitionData<T> FSM_State_StandUp<T>::Transition()
{
    // Finish Transition
    switch (this->nextStateName) {
        case FSM_StateName::PASSIVE:// normal
            this->transitionData.done = true;
            break;
        case FSM_StateName::BALANCE_STAND:
            this->transitionData.done = true;
            break;
        case FSM_StateName::LOCOMOTION:
            this->transitionData.done = true;
            break;
        case FSM_StateName::VISION:
            this->transitionData.done = true;
            break;
        default:
            std::cout << "[CONTROL FSM] Something went wrong in transition"
                      << std::endl;
    }

    // Return the transition data to the FSM
    return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template<typename T>
void FSM_State_StandUp<T>::OnExit()
{
    // Nothing to clean up when exiting
}

// template class FSM_State_StandUp<double>;
template class FSM_State_StandUp<float>;

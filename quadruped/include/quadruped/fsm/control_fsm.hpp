#ifndef CONTROLFSM_H
#define CONTROLFSM_H

#include <iostream>
// Contains all of the control related data
// #include "control_fsm_data.hpp"
// Checks the robot state and commands for safety
#include "safety_checker.hpp"
// FSM States
// #include "fsm_state.hpp"
// #include "../FSM_States/FSM_State_BalanceStand.h"
#include "fsm_impedance.hpp"
#include "fsm_jointPD.hpp"
#include "fsm_locomotion.hpp"
#include "fsm_passive.hpp"
#include "fsm_standup.hpp"
// #include "../FSM_States/FSM_State_RecoveryStand.h"
// #include "../FSM_States/FSM_State_Vision.h"
// #include "../FSM_States/FSM_State_BackFlip.h"
// #include "../FSM_States/FSM_State_FrontJump.h"

/**
 * Enumerate all of the operating modes
 */
enum class FSM_OperatingMode {
    NORMAL,
    TRANSITIONING,
    ESTOP,
    EDAMP
};

/**
 *
 */
template<typename T>
struct FSM_StatesList {
    FSM_State<T> *invalid;
    FSM_State_Passive<T>* passive;
    FSM_State_JointPD<T>* jointPD;
    FSM_State_ImpedanceControl<T>* impedanceControl;
    FSM_State_StandUp<T>* standUp;
    //   FSM_State_BalanceStand<T>* balanceStand;
    FSM_State_Locomotion<T> *locomotion;
    //   FSM_State_RecoveryStand<T>* recoveryStand;
    //   FSM_State_Vision<T>* vision;
    //   FSM_State_BackFlip<T>* backflip;
    //   FSM_State_FrontJump<T>* frontJump;
};

/**
 * Control FSM handles the FSM states from a higher level
 */
template<typename T>
class ControlFSM {
public:
    ControlFSM(Quadruped::Robot *_quadruped,
               Quadruped::StateEstimatorContainer<T> *_stateEstimator,
               //  Quadruped::LegController<T>* _legController,
               Quadruped::GaitGenerator *_gaitScheduler,
               Quadruped::DesiredStateCommand *_desiredStateCommand,
               //  RobotControlParameters* controlParameters,
               //  VisualizationData* visualizationData,
               UserParameters *userParameters);

    ~ControlFSM() = default;

    // Initializes the Control FSM instance
    void Initialize();

    void Reset(float currentTime)
    {
        resetTime = currentTime;
        timeSinceReset = 0;
        statesList.locomotion->Reset(currentTime);
    }

    // Runs the FSM logic and handles the state transitions and normal runs
    void RunFSM(std::vector<Quadruped::MotorCommand> &hybridAction);

    // This will be removed and put into the SafetyChecker class
    FSM_OperatingMode SafetyPreCheck();

    //
    FSM_OperatingMode SafetyPostCheck();

    // Gets the next FSM_State from the list of created states when requested
    FSM_State<T> *GetNextState(FSM_StateName stateName);

    // Prints the current FSM status
    void PrintInfo(int opt);

    // Contains all of the control related data
    ControlFSMData<T> data;

    // FSM state information
    FSM_StatesList<T> statesList;// holds all of the FSM States
    FSM_State<T> *currentState;  // current FSM state
    FSM_State<T> *nextState;     // next FSM state
    FSM_StateName nextStateName; // next FSM state name

    // Checks all of the inputs and commands for safety
    SafetyChecker<T> *safetyChecker;

    TransitionData<T> transitionData;

    Quadruped::LocomotionController *GetLocomotionController()
    {
        return statesList.locomotion->GetLocomotionController();
    }

private:
    // Operating mode of the FSM
    FSM_OperatingMode operatingMode;

    // Choose how often to print info, every N iterations
    int printNum = 10000;// N*(0.001s) in simulation time

    // Track the number of iterations since last info print
    int printIter = 0;// make larger than printNum to not print

    int iter = 0;
    float resetTime;
    float timeSinceReset;
};

#include "fsm/control_fsm.hxx"

#endif// CONTROLFSM_H

/*============================ Control FSM ============================*/
/**
 * The Finite State Machine that manages the robot's controls. Handles
 * calls to the FSM State functions and manages transitions between all
 * of the states.
 */

// #include "ControlFSM.h"
// #include <rt/rt_rc_interface.h>

/**
 * Constructor for the Control FSM. Passes in all of the necessary
 * data and stores it in a struct. Initializes the FSM with a starting
 * state and operating mode.
 *
 * @param _quadruped the quadruped information
 * @param _stateEstimator contains the estimated states
 * @param _legController interface to the leg controllers
 * @param _gaitScheduler controls scheduled foot contact modes
 * @param _desiredStateCommand gets the desired COM state trajectories
 * @param _quadruped passes in the control parameters from the GUI
 */
template<typename T>
ControlFSM<T>::ControlFSM(Quadruped::Robot *_quadruped,
                          Quadruped::StateEstimatorContainer<T> *_stateEstimators,
                          //  Quadruped::LegController<T>* _legController,
                          Quadruped::GaitGenerator *_gaitScheduler,
                          Quadruped::DesiredStateCommand *_desiredStateCommand,
                          //  RobotControlParameters* _quadruped,
                          //  VisualizationData* visualizationData,
                          UserParameters *userParameters)
{
    // Add the pointers to the ControlFSMData struct
    data.quadruped = _quadruped;
    data.stateEstimators = _stateEstimators;
    //   data._legController = _legController;
    data.gaitGenerator = _gaitScheduler;
    data.desiredStateCommand = _desiredStateCommand;
    data.userParameters = userParameters;

    // Initialize and add all of the FSM States to the state list
    statesList.invalid = nullptr;
      statesList.passive = new FSM_State_Passive<T>(&data);
      statesList.jointPD = new FSM_State_JointPD<T>(&data);
      statesList.impedanceControl = new FSM_State_ImpedanceControl<T>(&data);
      statesList.standUp = new FSM_State_StandUp<T>(&data);
    //   statesList.balanceStand = new FSM_State_BalanceStand<T>(&data);
    statesList.locomotion = new FSM_State_Locomotion<T>(&data);
    //   statesList.recoveryStand = new FSM_State_RecoveryStand<T>(&data);
    //   statesList.vision = new FSM_State_Vision<T>(&data);
    //   statesList.backflip = new FSM_State_BackFlip<T>(&data);
    //   statesList.frontJump = new FSM_State_FrontJump<T>(&data);

    safetyChecker = new SafetyChecker<T>(&data);

    // Initialize the FSM with the Passive FSM State
    Initialize();
    ROS_INFO("FSM Init Finished");
}

/**
 * Initialize the Control FSM with the default settings. SHould be set to
 * Passive state and Normal operation mode.
 */
template<typename T>
void ControlFSM<T>::Initialize()
{
    // Initialize a new FSM State with the control data
    //   currentState = statesList.passive;
    // currentState = statesList.locomotion;
    currentState = statesList.standUp;
    
    // Enter the new current state cleanly
    currentState->OnEnter();

    // Initialize to not be in transition
    nextState = currentState;

    // Initialize FSM mode to normal operation
    operatingMode = FSM_OperatingMode::NORMAL;
}

/**
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
template<typename T>
void ControlFSM<T>::RunFSM(std::vector<Quadruped::MotorCommand>& hybridAction)
{
    // 1. Publish state estimator data to other computer
    // 2. Check the robot state for safe operation
    // operatingMode = SafetyPreCheck();
    // 3. CHECK THE desiredStateCommand
    if(data.desiredStateCommand->joyCtrlStateChangeRequest) {
        
        const Quadruped::RC_MODE ctrlState = data.desiredStateCommand->JoyCtrlState;
        std::cout<< "[CONTROL FSM]: control mode = "<< ctrlState <<std::endl;
        if (ctrlState == Quadruped::RC_MODE::JOY_TROT ||
            ctrlState == Quadruped::RC_MODE::JOY_ADVANCED_TROT ||
            ctrlState == Quadruped::RC_MODE::JOY_WALK
            || ctrlState == Quadruped::RC_MODE::HARD_CODE) {
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
        // if(ctrlState == RC_MODE::RECOVERY_STAND){
        //     data._quadruped->fsmMode = K_RECOVERY_STAND;
        // } else if(ctrlState == RC_MODE::LOCOMOTION) {
        //     data._quadruped->fsmMode = K_LOCOMOTION;
        // } else if(ctrlState == RC_MODE::QP_STAND) {
        //     data._quadruped->fsmMode = K_BALANCE_STAND;
        // }
        // } else if(ctrlState == RC_MODE::VISION) {
        //     data._quadruped->fsmMode = K_VISION;
        // } else if(ctrlState == RC_MODE::BACKFLIP || ctrlState == RC_MODE::BACKFLIP_PRE){
        //     data._quadruped->fsmMode = K_BACKFLIP;
        // }
        //data._quadruped->fsmMode = K_FRONTJUMP;
        data.desiredStateCommand->joyCtrlStateChangeRequest = false;
        // operatingMode == FSM_OperatingMode::TRANSITIONING;
    }
    
    // Run the robot control code if operating mode is not unsafe
    // std::cout << "operatingmode = " << static_cast<int>(operatingMode) << std::endl;
    if (operatingMode != FSM_OperatingMode::ESTOP) {
        // Run normal controls if no transition is detected
        if (operatingMode == FSM_OperatingMode::NORMAL) {
            // Check the current state for any transition
            nextStateName = currentState->CheckTransition(); // FSM_State_Locomotion

            // Detect a commanded transition
            if (nextStateName != currentState->stateName) {
                // Set the FSM operating mode to transitioning
                operatingMode = FSM_OperatingMode::TRANSITIONING;
                // Get the next FSM State by name
                nextState = GetNextState(nextStateName);
                // Print transition initialized info
                //printInfo(1);
            } else if (currentState->transitionDuration > 0.1) {
                // locomotion gait transition
                operatingMode = FSM_OperatingMode::TRANSITIONING;
            } else {
                // Run the iteration for the current state normally
                // MITTimer T1;
                currentState->Run();
                // printf("CURstate.Run SOLVE TIME: %.3f\n", T1.getMs());
    
            }
        }
        
        if (operatingMode == FSM_OperatingMode::TRANSITIONING) {
            // ROS_INFO("[control fsm]: GO INTO currentState->Transition()\n");
            transitionData = currentState->Transition(); // FSM_State_Locomotion
            data.legCmd = transitionData.legCommand;
            // Check the robot state for safe operation
            SafetyPostCheck();

            // Run the state transition
            if (transitionData.done) {
                // Exit the current state cleanly
                currentState->OnExit();
                // Print finalizing transition info
                //printInfo(2);
                // Complete the transition
                currentState = nextState;
                // std::cout << "currentState->OnEnter()" <<std::endl;
                // Enter the new current state cleanly
                currentState->OnEnter();
                // Return the FSM to normal operation mode
                operatingMode = FSM_OperatingMode::NORMAL;
            }
        } else {
            SafetyPostCheck();
            // hybridAction = data.legCmd;
        }

    } else {// if ESTOP
        // currentState = statesList.passive;
        currentState->OnEnter();
        nextStateName = currentState->stateName;
    }
    hybridAction = data.legCmd;
    // Print the current state of the FSM
    // PrintInfo(0);
    // Increase the iteration counter
    iter++;
}

/**
 * Checks the robot state for safe operation conditions. If it is in
 * an unsafe state, it will not run the normal control code until it
 * is safe to operate again.
 *
 * @return the appropriate operating mode
 */
template<typename T>
FSM_OperatingMode ControlFSM<T>::SafetyPreCheck()
{
    // Check for safe orientation if the current state requires it
    //   if (currentState->checkSafeOrientation && data._quadruped->fsmMode != K_RECOVERY_STAND) {
    //     if (!safetyChecker->CheckSafeOrientation()) {
    //       operatingMode = FSM_OperatingMode::ESTOP;
    //       std::cout << "broken: Orientation Safety Ceck FAIL" << std::endl;
    //     }
    //   }

    // Default is to return the current operating mode
    return operatingMode;
}

/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 *
 * @return the appropriate operating mode
 */
template<typename T>
FSM_OperatingMode ControlFSM<T>::SafetyPostCheck()
{
    // Check for safe desired foot positions
    if (currentState->checkPDesFoot) {
        safetyChecker->CheckPDesFoot();
    }

    // Check for safe desired feedforward forces
    if (currentState->checkForceFeedForward) {
        safetyChecker->CheckForceFeedForward();
    }

    // Default is to return the current operating mode
    return operatingMode;
}

/**
 * Returns the approptiate next FSM State when commanded.
 *
 * @param  next commanded enumerated state name
 * @return next FSM state
 */
template<typename T>
FSM_State<T> *ControlFSM<T>::GetNextState(FSM_StateName stateName)
{
    // Choose the correct FSM State by enumerated state name
    switch (stateName) {
            case FSM_StateName::INVALID:
              return statesList.invalid;

            case FSM_StateName::PASSIVE:
              return statesList.passive;

            case FSM_StateName::JOINT_PD:
              return statesList.jointPD;

            case FSM_StateName::IMPEDANCE_CONTROL:
              return statesList.impedanceControl;

            case FSM_StateName::STAND_UP:
              return statesList.standUp;

            // case FSM_StateName::BALANCE_STAND:
            //   return statesList.balanceStand;

        case FSM_StateName::LOCOMOTION:
            return statesList.locomotion;

            // case FSM_StateName::RECOVERY_STAND:
            //   return statesList.recoveryStand;

            // case FSM_StateName::VISION:
            //   return statesList.vision;

            // case FSM_StateName::BACKFLIP:
            //   return statesList.backflip;

            // case FSM_StateName::FRONTJUMP:
            //   return statesList.frontJump;

        default:
            return statesList.invalid;
    }
}

/**
 * Prints Control FSM info at regular intervals and on important events
 * such as transition initializations and finalizations. Separate function
 * to not clutter the actual code.
 *
 * @param printing mode option for regular or an event
 */
template<typename T>
void ControlFSM<T>::PrintInfo(int opt)
{
    switch (opt) {
        case 0:// Normal printing case at regular intervals
            // Increment printing iteration
            printIter++;

            // Print at commanded frequency
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
                // std::cout << "Gait Type: " << data._gaitScheduler->gaitData.gaitName
                //           << "\n";
                std::cout << std::endl;

                // Reset iteration counter
                printIter = 0;
            }

            // Print robot info about the robot's status
            // data._gaitScheduler->printGaitInfo();
            // data._desiredStateCommand->PrintStateCommandInfo();

            break;

        case 1:// Initializing FSM State transition
            std::cout << "[CONTROL FSM] Transition initialized from "
                      << currentState->stateString << " to " << nextState->stateString
                      << "\n"
                      << std::endl;

            break;

        case 2:// Finalizing FSM State transition
            std::cout << "[CONTROL FSM] Transition finalizing from "
                      << currentState->stateString << " to " << nextState->stateString
                      << "\n"
                      << std::endl;

            break;
    }
}

// template class ControlFSM<double>; This should be fixed... need to make
// RobotRunner a template
template class ControlFSM<float>;

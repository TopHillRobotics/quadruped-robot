#include "fsm/fsm_locomotion.hpp"
#include <ros/package.h>

using namespace Quadruped;
extern LocomotionController *SetUpController(Robot *quadruped, GaitGenerator *gaitGenerator,
                                             DesiredStateCommand *desiredStateCommand,
                                             StateEstimatorContainer<float> *stateEstimators,
                                             UserParameters *userParameters,
                                             std::string &homeDir);

extern void UpdateControllerParams(LocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed);

/**
 * @brief Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template<typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(ControlFSMData<T> *controlFSMData)
    : FSM_State<T>(controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION")
{
    std::string homeDir = ros::package::getPath("quadruped") + "/";
    locomotionController = SetUpController(controlFSMData->quadruped, controlFSMData->gaitGenerator,
                                           controlFSMData->desiredStateCommand, controlFSMData->stateEstimators,
                                           controlFSMData->userParameters, homeDir);
    ROS_INFO("LocomotionController Init Finished");
    // locomotionController->Reset();
    // UpdateControllerParams(locomotionController, {0., 0., 0.}, 0.);

    this->TurnOnAllSafetyChecks();
    // Turn off Foot pos command since it is set in WBC as operational task
    this->checkPDesFoot = false;
    standLoop = false;
    // Initialize GRF and footstep locations to 0s
    this->footFeedForwardForces = Mat34<T>::Zero();
    this->footstepLocations = Mat34<T>::Zero();
    ROS_INFO("BuildDynamicModel Init BEFORE");
    controlFSMData->quadruped->BuildDynamicModel();
    ROS_INFO("BuildDynamicModel Init Finished");
    
    wbcController = new WbcLocomotionCtrl<T>(controlFSMData->quadruped->model, controlFSMData);
    ROS_INFO("WbcLocomotionCtrl Init Finished");

    if (controlFSMData->userParameters->useWBC) {
        wbcData = &(controlFSMData->quadruped->stateDataFlow.wbcData);
        wbcData->pBody_des.setZero();
        wbcData->vBody_des.setZero();
        wbcData->aBody_des.setZero();
        for (int legId(0); legId < NumLeg; ++legId) {
            wbcData->Fr_des[legId].setZero();
        }
    }
}

template<typename T>
void FSM_State_Locomotion<T>::OnEnter()
{
    // Default is to not transition
    this->nextStateName = this->stateName;
    // Reset the transition data
    this->transitionData.zero();
    // cMPCOld->initialize();
     
    // reset the robot control_mode
    // this->_data->_gaitScheduler->gaitData._nextGait = LocomotionMode::VELOCITY;
    Quadruped::RC_MODE ctrlState = this->_data->desiredStateCommand->JoyCtrlState;
    if (ctrlState != Quadruped::RC_MODE::HARD_CODE) {
        this->_data->userParameters->controlFrequency = 1000;
        switch (ctrlState) {
            case Quadruped::RC_MODE::JOY_TROT:
                this->_data->quadruped->controlParams["mode"] = LocomotionMode::VELOCITY_LOCOMOTION;
                this->_data->gaitGenerator->gait = "trot";
                break;
            case Quadruped::RC_MODE::JOY_ADVANCED_TROT:
                this->_data->quadruped->controlParams["mode"] = LocomotionMode::ADVANCED_TROT;
                this->_data->gaitGenerator->gait = "advanced_trot";
                // this->_data->userParameters->controlFrequency = 500; // todo
                break;
            case Quadruped::RC_MODE::JOY_WALK:
                this->_data->quadruped->controlParams["mode"] = LocomotionMode::WALK_LOCOMOTION;
                this->_data->gaitGenerator->gait = "walk";
                break;
            default: // stand
                // this->_data->quadruped->controlParams["mode"] = LocomotionMode::ADVANCED_TROT;
                // this->_data->gaitGenerator->gait = "advanced_trot";
                this->_data->gaitGenerator->gait = "stand";
                break;
        }
        // need to update com offset, gait generator params and stance leg controller KP/KD.
        this->_data->quadruped->Reset();
        this->_data->quadruped->timeStep = 1.0 / this->_data->userParameters->controlFrequency;
        std::cout << "update timeStep = " << this->_data->quadruped->timeStep << std::endl;
    
        locomotionController->Reset();

        this->_data->stateEstimators->Reset(0);
    }
    
    ROS_INFO("[FSM LOCOMOTION] On Enter\n");
}

/**
 * @brief Calls the functions to be executed on each control loop iteration.
 */
template<typename T>
void FSM_State_Locomotion<T>::Run()
{
    // Call the locomotion control logic for this iteration
    // MITTimer locoT;
    locomotionController->Update();
    auto [hybridAction, qpSol] = locomotionController->GetAction();
    // printf("---locomotion SOLVE TIME: %.3f\n", locoT.getMs());

    this->_data->legCmd = std::move(hybridAction);
    // std::cout << "legCmd" << std::setprecision(3) << MotorCommand::convertToMatix(this->_data->legCmd) << std::endl;
    
    if (this->_data->quadruped->controlParams["mode"] == LocomotionMode::ADVANCED_TROT) {
        float tua = 0;
        for (int motorId(0); motorId < NumMotor; ++motorId) {
            if (motorId % 3 == 0) {
                tua = 1.0 * pow(-1, (motorId / 3 + 1) % 2);
            } else {
                tua = 0;
            }
            this->_data->legCmd[motorId].tua += tua;
        }
        
        if (this->_data->userParameters->useWBC && wbcData->allowAfterMPC) {
            MITTimer wbcT;
            wbcController->Run(wbcData);
            this->_data->quadruped->stateDataFlow.visualizer.sa[2].Update(wbcT.getMs());
            // printf("---wbcT SOLVE TIME: %.3f\n", wbcT.getMs());
        }
    }
    // return this->_data->legCmd;// hybridAction
}


/**
 * @brief Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template<typename T>
FSM_StateName FSM_State_Locomotion<T>::CheckTransition()
{
    // Get the next state
    iter++;
    // Switch FSM control mode
    if (LocomotionSafe()) {
        switch (this->_data->quadruped->fsmMode) {
            case K_LOCOMOTION:
                break;
            case GAIT_TRANSITION: // K_LOCOMOTION:
                // this->nextStateName = FSM_StateName::LOCOMOTION;
                this->transitionDuration = 2.0; // > 0 means gait transition happens
                iter = 0;
                ROS_INFO("FSM_State_Locomotion: reset iter for GAIT_TRANSITION!!!\n");
                break;
            case LOCOMOTION_STAND:
                // this->nextStateName = FSM_StateName::LOCOMOTION;
                this->transitionDuration = 1; // > 0 means gait transition happens
                iter = 0;
                ROS_INFO("FSM_State_Locomotion: reset iter for LOCOMOTION_STAND!!!\n");
                break;
            case K_BALANCE_STAND:
                // Requested change to BALANCE_STAND
                this->nextStateName = FSM_StateName::BALANCE_STAND;
                // Transition time is immediate
                this->transitionDuration = 0.0;
                break;
            case K_PASSIVE:
                // Requested change to BALANCE_STAND
                this->nextStateName = FSM_StateName::PASSIVE;
                this->transitionDuration = 0.0;
                break;
            case K_STAND_UP:
                this->nextStateName = FSM_StateName::STAND_UP;
                this->transitionDuration = 0.;
                break;
            case K_STAND_DOWN:
                this->nextStateName = FSM_StateName::STAND_UP;
                this->transitionDuration = 0.;
                break;
            case K_RECOVERY_STAND:
                this->nextStateName = FSM_StateName::RECOVERY_STAND;
                this->transitionDuration = 0.;
                break;
            case K_VISION:
                this->nextStateName = FSM_StateName::VISION;
                this->transitionDuration = 0.;
                break;
            default:
                std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                          << "K_LOCOMOTION" << " to "
                          << this->_data->quadruped->fsmMode << std::endl;
        }
    } else {
        this->nextStateName = FSM_StateName::RECOVERY_STAND;
        this->transitionDuration = 0.;
        // rc_control.mode = RC_mode::RECOVERY_STAND;
    }
    // Return the next state name to the FSM
    return this->nextStateName;
}

/**
 * @brief Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template<typename T>
TransitionData<T> FSM_State_Locomotion<T>::Transition()
{   
    // Switch FSM control mode
    switch (this->nextStateName) {
        case FSM_StateName::LOCOMOTION:
            // gait transition happens
            if ((int)this->_data->quadruped->fsmMode==GAIT_TRANSITION) {
                SwitchMode();
            } else { //LOCOMOTION_STAND using torque controller
                StandLoop();
            }
            iter++;
            break;
        case FSM_StateName::BALANCE_STAND:
            // LocomotionControlStep();
            iter++;
            if (iter >= this->transitionDuration * 1000) {
                this->transitionData.done = true;
            } else {
                this->transitionData.done = false;
            }
            break;
        case FSM_StateName::PASSIVE:
            this->TurnOffAllSafetyChecks();
            this->transitionData.done = true;
            break;
        case FSM_StateName::STAND_UP:
            this->transitionData.done = true;
            this->transitionData.legCommand = this->_data->legCmd;
            break;
        case FSM_StateName::RECOVERY_STAND:
            this->transitionData.done = true;
            break;
        case FSM_StateName::VISION:
            this->transitionData.done = true;
            break;
        default:
            std::cout << "[CONTROL FSM] Wrong in transition" << std::endl;
    }
    // Return the transition data to the FSM
    return this->transitionData;
}

template <typename T>
bool FSM_State_Locomotion<T>::SwitchMode() 
{   
    Robot *robot = this->_data->quadruped;
    // std::cout << "[fsm locomotion]: iter = " << iter << std::endl;
    if (iter >= this->transitionDuration * 1000) {
        robot->fsmMode = K_LOCOMOTION;
        iter = 0;
        standLoop = false;
        this->transitionData.done = true;
        this->transitionDuration = 0;
        ROS_INFO("transitionData.done");
        return true;
    }
    int N = robot->GetFootContacts().cast<int>().sum();
    if (iter < 1000 ) {
        UpdateControllerParams(locomotionController,
                                {0.f, 0.f, 0.f},
                                0.f);
        this->_data->desiredStateCommand->stateDes.segment(6,6) << 0,0,0,0,0,0;
        
        locomotionController->Update();
        auto [hybridAction, qpSol] = locomotionController->GetAction();
        this->transitionData.legCommand = std::move(hybridAction);
        // this->_data->legCmd = std::move(hybridAction);
        if (N == 4) {
            iter = 1000;
        }
    } else { // keep stance for 1 s
        // robot->Step(quadruped->standUpMotorAngles, MotorMode::POSITION_MODE);
        auto angles = robot->GetMotorAngles();
        float ratio = std::max(0.45f, (iter-1000)/1000.0f);
        for (int i = 0; i < NumMotor; ++i) {
            this->transitionData.legCommand[i] = {robot->standUpMotorAngles[i]*ratio + (1-ratio)*angles[i], robot->motorKps[i]*2, 0, robot->motorKds[i], 0};
        }
    }
    return true;
}

template <typename T>
bool FSM_State_Locomotion<T>::StandLoop()
{   
    Robot *robot = this->_data->quadruped;
    if (iter >= 1000) {
        robot->fsmMode = K_LOCOMOTION;
        standLoop = true;
        iter = 0;
        this->transitionData.done = true;
        this->transitionDuration = 0;
        ROS_INFO("transitionData.done");
        return true;
    }
    
    int N = robot->GetFootContacts().cast<int>().sum();
    if (iter < this->transitionDuration *1000) {
        UpdateControllerParams(locomotionController,
                                {0.f, 0.f, 0.f},
                                0.f);
        this->_data->desiredStateCommand->stateDes.segment(6,6) << 0,0,0,0,0,0;
        
        locomotionController->Update();
        auto [hybridAction, qpSol] = locomotionController->GetAction();
        this->transitionData.legCommand = std::move(hybridAction);
        if (N == 4) {
            iter = 1000;
        }
    }
    return true;
}

template<typename T>
bool FSM_State_Locomotion<T>::LocomotionSafe()
{
    return true;
}

/**
 * @brief Cleans up the state information on exiting the state.
 */
template<typename T>
void FSM_State_Locomotion<T>::OnExit()
{
    // Nothing to clean up when exiting
    iter = 0;
}

template class FSM_State_Locomotion<float>;

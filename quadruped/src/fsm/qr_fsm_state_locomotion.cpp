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

#include "fsm/qr_fsm_state_locomotion.hpp"
#include <ros/package.h>


using namespace Quadruped;


extern qrLocomotionController *SetUpController(qrRobot *quadruped, qrGaitGenerator *gaitGenerator,
                                             qrDesiredStateCommand *desiredStateCommand,
                                             qrStateEstimatorContainer *stateEstimators,
                                             qrUserParameters *userParameters,
                                             std::string &homeDir);

extern void UpdateControllerParams(qrLocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed);


template<typename T>
qrFSMStateLocomotion<T>::qrFSMStateLocomotion(qrControlFSMData<T> *controlFSMData):
    qrFSMState<T>(controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION")
{
    std::string homeDir = ros::package::getPath("quadruped") + "/";
    locomotionController = SetUpController(controlFSMData->quadruped, controlFSMData->gaitGenerator,
                                           controlFSMData->desiredStateCommand, controlFSMData->stateEstimators,
                                           controlFSMData->userParameters, homeDir);

    printf("LocomotionController Init Finished\n");

    this->TurnOnAllSafetyChecks();

    /* Turn off Foot pos command since it is set in WBC as operational task. */
    this->checkPDesFoot = false;

    printf("BuildDynamicModel Init BEFORE\n");
    controlFSMData->quadruped->BuildDynamicModel();
    printf("BuildDynamicModel Init Finished\n");
    
    wbcController = new qrWbcLocomotionController<T>(controlFSMData->quadruped->model, controlFSMData);
    printf("WbcLocomotionCtrl Init Finished\n");

    /* If use WBC controller, then initialize the WBC controllers. */
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
void qrFSMStateLocomotion<T>::OnEnter()
{
    this->nextStateName = this->stateName;
    this->transitionData.Zero();
     
    // reset the robot control_mode
    // this->_data->_gaitScheduler->gaitData._nextGait = LocomotionMode::VELOCITY;
    Quadruped::RC_MODE ctrlState = this->_data->desiredStateCommand->getJoyCtrlState();
    printf("[FSM] On Enter State: %d", int(ctrlState));

    if (ctrlState != Quadruped::RC_MODE::HARD_CODE) {
        /* This control frequency can be adjusted by user. */
        this->_data->userParameters->controlFrequency = 1000;

        /* JOY_TROT: trotting by Force Balance controller.
         * JOY_ADVANCED_TROT: trotting by MPC or MPC-WBC controller.
         * WALK_LOCOMOTION: use walk gait.
         * JOY_STAND: keep standing.
         */
        switch (ctrlState) {
        case Quadruped::RC_MODE::JOY_TROT:
            this->_data->quadruped->controlParams["mode"] = LocomotionMode::VELOCITY_LOCOMOTION;
            this->_data->gaitGenerator->gait = "trot";
            break;
        case Quadruped::RC_MODE::JOY_ADVANCED_TROT:
            // this->_data->userParameters->controlFrequency = 500;
            this->_data->quadruped->controlParams["mode"] = LocomotionMode::ADVANCED_TROT;
            this->_data->gaitGenerator->gait = "advanced_trot";
            break;
        case Quadruped::RC_MODE::JOY_WALK:
            this->_data->quadruped->controlParams["mode"] = LocomotionMode::WALK_LOCOMOTION;
            this->_data->gaitGenerator->gait = "walk";
            break;
        default:
            this->_data->gaitGenerator->gait = "stand";
            break;
        }

        /* Reset the locomotion controller entering the locomotion state. */
        this->_data->quadruped->Reset();
        this->_data->quadruped->timeStep = 1.0 / this->_data->userParameters->controlFrequency;

        std::cout << "Time Step: " << this->_data->quadruped->timeStep << std::endl;

        locomotionController->Reset();
        this->_data->stateEstimators->Reset();
    }
    printf("[FSM LOCOMOTION] On Enter\n");
}


template<typename T>
void qrFSMStateLocomotion<T>::Run()
{
    /* Call the locomotion control logic for this iteration and get the results, including commands and reaction force
     * Save the commands into FSM Data structure.
     */
    locomotionController->Update();
    auto [hybridAction, qpSol] = locomotionController->GetAction();
    this->_data->legCmd = std::move(hybridAction);

    
    if (this->_data->quadruped->controlParams["mode"] == LocomotionMode::ADVANCED_TROT) {

        /* Add a compensation torque to hip joints in MPC. */
        float tua_ = 0.9, tua = 0;
        for (int motorId(0); motorId < NumMotor; ++motorId) {
            if (motorId % 3 == 0) {
                tua = tua_ * pow(-1, (motorId / 3 + 1) % 2);
            } else {
                tua = 0;
            }
            this->_data->legCmd[motorId].tua += tua;
        }
        
        /* Run Whole Body Controller if WBC is enabled. */
        if (this->_data->userParameters->useWBC && wbcData->allowAfterMPC) {
            wbcController->Run(wbcData);
        }
    }
}


template<typename T>
FSM_StateName qrFSMStateLocomotion<T>::CheckTransition()
{
    iter++;

    if (LocomotionSafe()) {

        switch (this->_data->quadruped->fsmMode) {
        case K_LOCOMOTION:
            break;

        /* GAIT_TRANSITION and LOCOMOTION_STAND happen in the locomotion state.
         * Different gaits and MPC-WBC standing can be considered as substates of locomotion.
         */
        case GAIT_TRANSITION:
            this->transitionDuration = 2.0;
            iter = 0;
            printf("FSM_State_Locomotion: reset iter for GAIT_TRANSITION!!!\n");
            break;
        case LOCOMOTION_STAND:
            this->transitionDuration = 1.0;
            iter = 0;
            printf("FSM_State_Locomotion: reset iter for LOCOMOTION_STAND!!!\n");
            break;

        case K_BALANCE_STAND:
            this->nextStateName = FSM_StateName::BALANCE_STAND;
            this->transitionDuration = 0.0;
            break;
        case K_PASSIVE:
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
        this->transitionDuration = 0.0;

    }

    return this->nextStateName;
}


template<typename T>
qrTransitionData<T> qrFSMStateLocomotion<T>::Transition()
{   

    switch (this->nextStateName) {
    case FSM_StateName::LOCOMOTION:
        /* If transfer from trot to walk, gait transition happens and quadruped needs switch mode.
         * Or will enter MPC standing.
         */
        if ((int)this->_data->quadruped->fsmMode==GAIT_TRANSITION) {
            SwitchMode();
        } else {
            StandLoop();
        }
        iter++;
        break;
    case FSM_StateName::BALANCE_STAND:
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
        printf("[CONTROL FSM] Wrong in transition\n");
    }

    return this->transitionData;
}


template <typename T>
bool qrFSMStateLocomotion<T>::SwitchMode()
{   
    qrRobot *robot = this->_data->quadruped;

    if (iter >= this->transitionDuration * 1000) {
        robot->fsmMode = K_LOCOMOTION;
        iter = 0;
        this->transitionData.done = true;
        this->transitionDuration = 0;
        ROS_INFO("transitionData.done");
        return true;
    }

    int N = robot->GetFootContact().cast<int>().sum();

    if (iter < 1000) {
        /* Slow Down the velocity of the quadruped in 1000 iterations if the quadruped is trotting. */
        UpdateControllerParams(locomotionController, {0.f, 0.f, 0.f}, 0.f);
        this->_data->desiredStateCommand->stateDes.segment(6, 6) << 0, 0, 0, 0, 0, 0;
        
        locomotionController->Update();

        auto [hybridAction, qpSol] = locomotionController->GetAction();

        this->transitionData.legCommand = std::move(hybridAction);
        /* If four feet are on the groud, then continue to next stage. */
        if (N == 4) {
            iter = 1000;
        }
    } else {
        /* Keep stance for 1s. */
        auto angles = robot->GetMotorAngles();
        float ratio = std::max(0.45f, (iter-1000)/1000.0f);
        for (int i = 0; i < NumMotor; ++i) {
            this->transitionData.legCommand[i] = {robot->standUpMotorAngles[i]*ratio + (1-ratio)*angles[i], robot->motorKps[i], 0, robot->motorKds[i], 0};
        }
    }
    return true;
}


template <typename T>
bool qrFSMStateLocomotion<T>::StandLoop()
{    
    /* Similar logic to %SwitchMode(). */
    qrRobot *robot = this->_data->quadruped;
    if (iter >= 1000) {
        robot->fsmMode = K_LOCOMOTION;
        iter = 0;
        this->transitionData.done = true;
        this->transitionDuration = 0;
        ROS_INFO("transitionData.done");
        return true;
    }
    
    int N = robot->GetFootContact().cast<int>().sum();
    if (iter < this->transitionDuration * 1000) {
        UpdateControllerParams(locomotionController, {0.f, 0.f, 0.f}, 0.f);
        this->_data->desiredStateCommand->stateDes.segment(6, 6) << 0, 0, 0, 0, 0, 0;
        
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
bool qrFSMStateLocomotion<T>::LocomotionSafe()
{
    return true;
}


template<typename T>
void qrFSMStateLocomotion<T>::OnExit()
{
    /* Standup state does nothing when exitting */
    iter = 0;
}

template class qrFSMStateLocomotion<float>;

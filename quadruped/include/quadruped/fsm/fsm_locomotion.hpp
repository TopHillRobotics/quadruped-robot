#ifndef FSM_STATE_LOCOMOTION_H
#define FSM_STATE_LOCOMOTION_H

#include "control_fsm_data.hpp"
#include "controllers/locomotion_controller.h"
#include "controllers/wbc/wbc_locomotion_controller.hpp"
#include "fsm_state.hpp"

/**
 * @brief FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */
template<typename T>
class FSM_State_Locomotion : public FSM_State<T> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FSM_State_Locomotion(ControlFSMData<T> *controlFSMData);

    // Behavior to be carried out when entering a state
    void OnEnter();

    // Run the normal behavior for the state
    virtual void Run();

    // Checks for any transition triggers
    FSM_StateName CheckTransition();

    // Manages state specific transitions
    TransitionData<T> Transition();

    bool SwitchMode();

    Quadruped::LocomotionController *GetLocomotionController()
    {
        return locomotionController;
    }

    // Behavior to be carried out when exiting a state
    void OnExit();

    void Reset(float currentTime)
    {
        this->resetTime = currentTime;
        this->timeSinceReset = 0;
        locomotionController->Reset();
    }

private:
    bool LocomotionSafe();
    bool StandLoop();
    
    Quadruped::LocomotionController *locomotionController;
    WbcLocomotionCtrl<T> *wbcController;
    WbcCtrlData *wbcData;
    bool standLoop;
    unsigned long iter = 0; // Keep track of the control iterations
};

#endif// FSM_STATE_LOCOMOTION_H

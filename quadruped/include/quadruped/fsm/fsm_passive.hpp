#ifndef FSM_STATE_PASSIVE_H
#define FSM_STATE_PASSIVE_H

#include "fsm/fsm_state.hpp"

template<typename T>
class FSM_State_Passive : public FSM_State<T> {
public:
    FSM_State_Passive(ControlFSMData<T> *_controlFSMData);

    // Behavior to be carried out when entering a state
    void OnEnter();

    // Run the normal behavior for the state
    virtual void Run();

    // Checks for any transition triggers
    FSM_StateName CheckTransition();

    // Manages state specific transitions
    TransitionData<T> Transition();

    // Behavior to be carried out when exiting a state
    void OnExit();

    TransitionData<T> testTransition();

private:
    // Keep track of the control iterations
    int iter = 0;
};

#endif// FSM_STATE_PASSIVE_H

#ifndef FSM_STATE_JOINTPD_H
#define FSM_STATE_JOINTPD_H

#include "control_fsm_data.hpp"
#include "fsm_state.hpp"


template <typename T>
class FSM_State_JointPD : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_JointPD(ControlFSMData<T>* _controlFSMData);

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

 private:
  // Keep track of the control iterations
  int iter = 0;
  DVec<T> _ini_jpos;
};

#endif  // FSM_STATE_JOINTPD_H

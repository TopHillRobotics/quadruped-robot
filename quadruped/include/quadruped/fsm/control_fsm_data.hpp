#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include "robots/robot.h"
#include "controllers/desired_state_command.hpp"
#include "gait/gait.h"
#include "estimators/state_estimator.hpp"

/** 
 * @brief ControlFSMData 
 */
template <typename T>
struct ControlFSMData {
  Quadruped::Robot* quadruped;
  Quadruped::StateEstimatorContainer<T>* stateEstimators;
  Quadruped::GaitGenerator* gaitGenerator;
  Quadruped::DesiredStateCommand* desiredStateCommand;
  // RobotControlParameters* controlParameters;
  UserParameters* userParameters;
  // VisualizationData* visualizationData;
  std::vector<Quadruped::MotorCommand> legCmd;
};

// template struct ControlFSMData<double>;
template struct ControlFSMData<float>;

#endif  // CONTROLFSM_H

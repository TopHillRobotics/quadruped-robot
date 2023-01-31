#ifndef _BASE_STATE_ESTIMATOR_
#define _BASE_STATE_ESTIMATOR_

#include "robots/robot.h"

/*!
 * Inputs for state estimation.
 * If robot code needs to inform the state estimator of something,
 * it should be added here. (You should also a setter method to
 * StateEstimatorContainer)
 */
// template <typename T>
struct StateEstimatorData {
//   StateEstimate<T>* result;  // where to write the output to
//   VectorNavData* vectorNavData;
//   CheaterState<double>* cheaterState;
//   LegControllerData<T>* legControllerData;
//   Vec4<T>* contactPhase;
//   RobotControlParameters* parameters;
};

/*!
 * All Estimators should inherit from this class
 */
class BaseEstimator {
 public:
  virtual void Update(float currentTime) = 0;
  virtual void Reset(float currentTime) = 0;

  void SetData(StateEstimatorData data) { _stateEstimatorData = data; }

  virtual ~BaseEstimator() = default;
  StateEstimatorData _stateEstimatorData;
};


#endif // _BASE_STATE_ESTIMATOR_
#ifndef _STATE_ESTIMATOR_
#define _STATE_ESTIMATOR_

#include "estimators/anomaly_detection.h"
#include "estimators/base_estimator.h"
#include "estimators/ground_estimator.h"
#include "estimators/robot_estimator.h"

namespace Quadruped {
    /**
    * @brief Main State Estimator Class
    *   Contains all GenericEstimators, and can run them
    *   Also updates visualizations.
    */
    template<typename T>
    class StateEstimatorContainer {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        StateEstimatorContainer(Robot *quadrupedIn,
                                GaitGenerator *gaitGeneratorIn,
                                UserParameters *userParametersIn,
                                std::string terrainConfigPath,
                                std::string homeDir);
 
        void Reset(float t)
        {
            resetTime = 0;
            timeSinceReset = 0.;
            groundEstimator->Reset(timeSinceReset);
            contactDetection->Reset(timeSinceReset);
            robotEstimator->Reset(timeSinceReset);
            std::cout << "StateEstimatorContainer Reset" << std::endl;
        }
        
        void Update()
        {
            // CheetahVisualization* visualization = nullptr) {
            // for (auto estimator : _estimators) {
            //     estimator->Update(quadruped->GetTimeSinceReset());
            // }
            // if (visualization) {
            //   visualization->quat = _data.result->orientation.template cast<float>();
            //   visualization->p = _data.result->position.template cast<float>();
            //   // todo contact!
            // }
            timeSinceReset = quadruped->GetTimeSinceReset() - resetTime;
            //std::cout << "--state estimator update time-------- " << timeSinceReset << std::endl;
            // contactDetection->Update(timeSinceReset);
            groundEstimator->Update(timeSinceReset);
            robotEstimator->Update(timeSinceReset);
        }

        /*!
        * Set the contact phase
        */
        //   void setContactPhase(Vec4<T>& phase) {
        //     *_data.contactPhase = phase;
        //   }

        //   /*!
        //    * Add an estimator of the given type
        //    * @tparam EstimatorToAdd
        //    */
        //   template <typename EstimatorToAdd>
        //   void addEstimator() {
        //     auto* estimator = new EstimatorToAdd();
        //     estimator->setData(_data);
        //     estimator->setup();
        //     _estimators.push_back(estimator);
        //   }

        //   /*!
        //    * Remove all estimators of a given type
        //    * @tparam EstimatorToRemove
        //    */
        //   template <typename EstimatorToRemove>
        //   void removeEstimator() {
        //     int nRemoved = 0;
        //     _estimators.erase(
        //         std::remove_if(_estimators.begin(), _estimators.end(),
        //                        [&nRemoved](GenericEstimator<T>* e) {
        //                          if (dynamic_cast<EstimatorToRemove*>(e)) {
        //                            delete e;
        //                            nRemoved++;
        //                            return true;
        //                          } else {
        //                            return false;
        //                          }
        //                        }),
        //         _estimators.end());
        //   }
        
        void RemoveAllEstimators()
        {
            for (auto estimator : _estimators) {
                delete estimator;
            }
            _estimators.clear();
        }

        ~StateEstimatorContainer() = default;
        // {
        //     for (auto estimator : _estimators) {
        //         delete estimator;
        //     }
        // }
        inline ContactDetection* GetContactDetection()
        {
            return contactDetection;
        }

        inline RobotEstimator* GetRobotEstimator()
        {
            return robotEstimator;
        }

        inline GroundSurfaceEstimator* GetGroundEstimator()
        {
            return groundEstimator;
        }
    
    private:
        Robot *quadruped;
        GaitGenerator *gaitGenerator;
        UserParameters *userParameters;
        std::vector<BaseEstimator*> _estimators;
        GroundSurfaceEstimator *groundEstimator;
        ContactDetection *contactDetection;
        RobotEstimator *robotEstimator;

        Vec4<T> _phase;
        float resetTime;
        float timeSinceReset;
    };

}// namespace Quadruped

#include "estimators/state_estimator.hxx"

#endif// _STATE_ESTIMATOR_

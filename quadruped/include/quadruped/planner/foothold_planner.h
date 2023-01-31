/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: planning the foothold/offset of swing leg.
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-11-08
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*         add ComputeHeuristicFootHold member function. @ Zhu Yijie 2022.04.01
*/

#ifndef ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_Foothold_PLANNER_H_
#define ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_Foothold_PLANNER_H_

#include "fsm/control_fsm_data.hpp"
#include "planner/foot_stepper.h"

namespace Quadruped {
    /**
     * @brief plan the foothold fpr next swing stage.
     */
    class FootholdPlanner {
    public:

        FootholdPlanner(Robot *quadrupedIn, GaitGenerator* gaitGeneratorIn,
                    StateEstimatorContainer<float>* stateEstimatorsIn,
                    UserParameters* userParametersIn,
                    DesiredStateCommand* desiredStateCommandIn);

        ~FootholdPlanner() = default;

        void Reset(float t);

        void Update()
        {}
        /**
         * @brief only be called at the moment right before lift up legs.
         */
        void UpdateOnce(Eigen::Matrix<float, 3, 4> currentFootholds, std::vector<int> legIds={});

        bool Loadterrain( std::string& configPathIn);

        // for postion mode 
        Eigen::Matrix<float, 3, 4> ComputeNextFootholds(Eigen::Matrix<float, 3, 4>& currentFootholds,
                                                        Eigen::Matrix<float, 6, 1>& currentComPose,
                                                        Eigen::Matrix<float, 6, 1>& desiredComPose,
                                                        std::vector<int>& legIds);

        // for walk mode 
        Eigen::Matrix<float, 3, 4> ComputeFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds,
                                                          Eigen::Matrix<float, 6, 1> currentComPose,
                                                          Eigen::Matrix<float, 6, 1> desiredComPose,
                                                          std::vector<int> legIds);

        inline const Eigen::Matrix<float, 6, 1> &GetDesiredComPose() const
        {
            return desiredComPose;
        }

        inline const Eigen::Matrix<float, 3, 4> &GetFootholdsOffset() const
        {
            return desiredFootholdsOffset;
        }

        inline Eigen::Matrix<float, 6, 1> GetComGoal(Eigen::Matrix<float, 6, 1> currentComPose)
        {
            desiredComPose << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f;
            return desiredComPose;
        }

        /**
         * @brief for walk mode in world frame
         */
        inline Vec3<float> GetFootholdInWorldFrame(int legId) {
            return desiredFootholds.col(legId);
        }

        void ComputeHeuristicFootHold(std::vector<u8> swingFootIds);
        void ComputeMITFootHold(int legId);
    
    public:

        Robot *robot;
        GaitGenerator* gaitGenerator;
        GroundSurfaceEstimator *groundEsitmator;
        UserParameters* userParameters;
        DesiredStateCommand* desiredStateCommand;
        FootStepper *footstepper;

        float resetTime;
        float timeSinceReset;

        YAML::Node footStepperConfig;
        float gapWidth;
        float footHoldOffset;
        std::vector<float> gaps;
        Terrain& terrain;
        constexpr static int N = 50; // default map size
        
        long long unsigned stepCount;
        Eigen::Matrix<float, 6, 1> comPose;
        Eigen::Matrix<float, 6, 1> desiredComPose;
        Eigen::Matrix<float, 3, 4> desiredFootholdsOffset;
        Eigen::Matrix<float, 3, 4> desiredFootholds;
        Vec4<float> phase;
        Vec3<float> swingKp;
        int moveDown[4] = {0,0,0,0};
        Vec12<float> firstSwingBaseState;

    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_Foothold_PLANNER_H_

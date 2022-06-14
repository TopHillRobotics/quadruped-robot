/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Walk gait generator
* Author: Zhu Yijie
* Create: 2022-01-25
* Notes: xx
* Modify: 
*/

#ifndef ASCEND_QUADRUPED_CPP_Walk_GAIT_GENERATOR_H
#define ASCEND_QUADRUPED_CPP_Walk_GAIT_GENERATOR_H

#include <math.h>
#include <vector>
#include "types.h"
#include "robots/robot.h"
#include "mpc_controller/openloop_gait_generator.h"

namespace Quadruped {
    /**
     * @brief the gait generator for walk mode
     */
    class WalkGaitGenerator: public OpenloopGaitGenerator {
    public:

        WalkGaitGenerator() {};

        WalkGaitGenerator(Robot *robot, std::string configFilePath);

        WalkGaitGenerator(Robot *robot,
                            std::vector<SubLegState> stateSwitchQue,
                            std::vector<float> stateRatioQue,
                            Eigen::Matrix<float, 4, 1> stanceDuration,
                            Eigen::Matrix<float, 4, 1> dutyFactor,
                            Eigen::Matrix<int, 4, 1> initialLegState,
                            Eigen::Matrix<float, 4, 1> initialLegPhase,
                            float contactDetectionPhaseThreshold=0.1f);

        virtual ~WalkGaitGenerator() = default;

        virtual void Reset(float currentTime);

        /** @brief should be called before compute swing/stance controller.
         * todo  (a) this is a cyclic startage for gait locomotion, but there is another non-cyclic
         * todo   planner to deal with sudden situation or free-gait swiching.
         */
        virtual void Update(float currentTime);

// private:
        std::vector<SubLegState> stateSwitchQue; // should start from stand state
        std::vector<float> stateRatioQue;
        std::vector<float> accumStateRatioQue; // accumulation of state ration in complete walk cycle
        Vec4<int> stateIndexOfLegs;
        Vec4<float> curStateRadio;
        float trueSwingStartPhaseInSwingCycle;
        
    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_Walk_GAIT_GENERATOR_H
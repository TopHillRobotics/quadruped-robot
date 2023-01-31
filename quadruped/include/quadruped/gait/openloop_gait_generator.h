/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Openloop gait generator
* Author: Xie Ming Cheng
* Create: 2021-10-25
* Notes: xx
* Modify: add head comment and add some function comments. @ Xie_mingcheng 2021.11.22
*/

#ifndef ASCEND_QUADRUPED_CPP_OPENLOOP_GAIT_GENERATOR_H
#define ASCEND_QUADRUPED_CPP_OPENLOOP_GAIT_GENERATOR_H

#include <math.h>
#include <vector>
#include "robots/robot.h"
#include "gait/gait.h"

namespace Quadruped {
    class OpenloopGaitGenerator : public GaitGenerator {
    public:
        OpenloopGaitGenerator();

        OpenloopGaitGenerator(Robot *robot, std::string configFilePath);

        OpenloopGaitGenerator(Robot *robot,
                              Eigen::Matrix<float, 4, 1> stanceDuration,
                              Eigen::Matrix<float, 4, 1> dutyFactor,
                              Eigen::Matrix<int, 4, 1> initialLegState,
                              Eigen::Matrix<float, 4, 1> initialLegPhase,
                              float contactDetectionPhaseThreshold=0.1f);

        virtual ~OpenloopGaitGenerator() = default;

        virtual void Reset(float currentTime);

        /** @brief should be called before compute swing/stance controller.
         * todo  (a) this is a cyclic stratage for gait locomotion, but there is another non-cyclic
         * todo   planner to deal with sudden situation or free-gait swiching.
         */
        virtual void Update(float currentTime);

        /** @brief if foot does not get into desired state, how to plan the next desired state */
        virtual void Schedule(float currentTime);
    
    private: 
        float cumDt = 0;
        float waitTime = 1.0;
    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_OPENLOOP_GAIT_GENERATOR_H
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
#include "types.h"
#include "robots/robot.h"

namespace Quadruped {
    class OpenloopGaitGenerator {
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
         * todo  (a) this is a cyclic startage for gait locomotion, but there is another non-cyclic
         * todo   planner to deal with sudden situation or free-gait swiching.
         */
        virtual void Update(float currentTime);

// private:
        std::string configFilePath;
        YAML::Node config; // load from config file
        Robot *robot;
        Eigen::Matrix<float, 4, 1> stanceDuration;
        Eigen::Matrix<float, 4, 1> swingDuration;
        // the time period ratio for stance stage, i.e. stanceDur/(stanceDur+swingDur)
        Eigen::Matrix<float, 4, 1> dutyFactor;
        Eigen::Matrix<float, 4, 1> initialLegPhase;
        Eigen::Matrix<int, 4, 1> initialLegState;
        Eigen::Matrix<int, 4, 1> nextLegState;
        Eigen::Matrix<int, 4, 1> legState;
        // the phase w.r.t a certain stage, NOT the total periodic/cyclic duration.
        Eigen::Matrix<float, 4, 1> normalizedPhase; // the phase for the desired state.
        Eigen::Matrix<int, 4, 1> desiredLegState;  // stance/swing/early stane/lost stance
        Eigen::Matrix<int, 4, 1> lastLegState;
        Eigen::Matrix<int, 4, 1> curLegState; // curLegState is the planned current state, while legState is the state actually detected via senros.
        Eigen::Matrix<float, 4, 1> initStateRadioInCycle;
        Vec4<float> fullCyclePeriod;
        
        // when the leg status is swing, used for identifying effectiveness of the contact dection judgement
        float contactDetectionPhaseThreshold; 
        long long count=0;
        std::vector<float> moveBaseRatioPoint;
        float moveBaseTime;
        float moveBasePhase;
        float trueSwingStartPhaseInSwingCycle;
        
        Eigen::Matrix<int, 4, 1> detectedLegState;
        Vec4<float> detectedEventTickPhase;        
    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_OPENLOOP_GAIT_GENERATOR_H
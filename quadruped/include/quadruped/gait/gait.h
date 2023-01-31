/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Gait generator
* Author: Yijie Zhu
* Create: 2022-03-24
* Notes: xx
* Modify: 
*/

#ifndef ASCEND_QUADRUPED_CPP_GAIT_GENERATOR_H
#define ASCEND_QUADRUPED_CPP_GAIT_GENERATOR_H

#include "robots/robot.h"

namespace Quadruped {
    class GaitGenerator {
    public:
        GaitGenerator() {}

        GaitGenerator(Robot *robot, std::string configFilePath);

        GaitGenerator(Robot *robot,
                    Eigen::Matrix<float, 4, 1> stanceDuration,
                    Eigen::Matrix<float, 4, 1> dutyFactor,
                    Eigen::Matrix<int, 4, 1> initialLegState,
                    Eigen::Matrix<float, 4, 1> initialLegPhase,
                    float contactDetectionPhaseThreshold=0.1f);

        virtual ~GaitGenerator() = default;

        virtual void Reset(float currentTime)
        {
            resetTime = currentTime;
            lastTime = currentTime;
            normalizedPhase.setZero();
            curLegState = initialLegState;
            lastLegState = curLegState;
            legState = curLegState;
            desiredLegState = curLegState;
            firstStanceAngles = robot->standUpMotorAngles;
            contactStartPhase.setZero();
        };

        /** @brief should be called before compute swing/stance controller.
         * todo  (a) this is a cyclic startage for gait locomotion, but there is another non-cyclic
         * todo   planner to deal with sudden situation or free-gait swiching.
         */
        virtual void Update(float currentTime) = 0;

        virtual void Schedule(float currentTime) {}
        
        YAML::Node config; // load from config file
        Robot *robot;
        std::string gait;
        float resetTime;
        float timeSinceReset;
        float lastTime;
        Eigen::Matrix<float, 4, 1> stanceDuration;
        Eigen::Matrix<float, 4, 1> swingDuration;
        // the time period ratio for stance stage, i.e. stanceDur/(stanceDur+swingDur)
        Eigen::Matrix<float, 4, 1> dutyFactor;
        Eigen::Matrix<float, 4, 1> phaseInFullCycle;
        Eigen::Matrix<float, 4, 1> initialLegPhase;
        Eigen::Matrix<float, 4, 1> offset; // true global initial leg phase, [0, stance duty] for stance, [stance duty, 1.0] for swing
        
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
        Vec4<bool> allowSwitchLegState;
        // when the leg status is swing, used for identifying effectiveness of the contact dection judgement
        float contactDetectionPhaseThreshold; 
        long long count=0;
        std::vector<float> moveBaseRatioPoint;
        float moveBaseTime;
        float moveBasePhase;
        float trueSwingStartPhaseInSwingCycle;
        float trueSwingStartPhaseInFullCycle;
        float trueSwingEndPhaseInFullCycle;
        unsigned long gaitCycle=0;
        Eigen::Matrix<int, 4, 1> detectedLegState;
        Vec4<float> detectedEventTickPhase; 

        Vec4<bool> firstSwing = {false, false, false, false};
        Vec4<float> contactStartPhase;
        Vec4<float> swingTimeRemaining = {0.f,0.f,0.f,0.f};  
        Vec4<bool> firstStance = {false, false, false, false};
        Eigen::Matrix<float,12,1> firstStanceAngles;
    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_GAIT_GENERATOR_H
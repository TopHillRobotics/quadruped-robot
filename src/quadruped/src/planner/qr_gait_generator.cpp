
// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "planner/qr_gait_generator.h"

    qrGaitGenerator::qrGaitGenerator() {}

    qrGaitGenerator::qrGaitGenerator(qrRobot *robot,
                                     Eigen::Matrix<float, 4, 1> stanceDuration,
                                     Eigen::Matrix<float, 4, 1> dutyFactor,
                                     Eigen::Matrix<int, 4, 1> initialLegState,
                                     Eigen::Matrix<float, 4, 1> initialLegPhase,
                                     float contactDetectionPhaseThreshold)
        {
            this->robot = robot;
            this->robotState = robot->GetRobotState();
            this->stanceDuration = stanceDuration;
            this->dutyFactor = dutyFactor;
            this->initialLegState = initialLegState;
            this->initialLegPhase = initialLegPhase;
            this->contactDetectionPhaseThreshold = contactDetectionPhaseThreshold;
            this->swingDuration = stanceDuration.cwiseQuotient(dutyFactor) - stanceDuration;

            this->lastLegState = initialLegState;

            for (int legId = 0; legId < initialLegState.size(); legId++) {
                if (initialLegState[legId] == LegState::SWING) {
                    initStateRatioInCycle[legId] = 1 - dutyFactor[legId];
                    nextLegState[legId] = LegState::STANCE;
                } else {
                    initStateRatioInCycle[legId] = dutyFactor[legId];
                    nextLegState[legId] = LegState::SWING;
                }
            }

            Reset(0);
        }


    qrGaitGenerator::qrGaitGenerator(qrRobot *robot,std::string configFilePath)
        {

            this->configFilePath = configFilePath;
            config = YAML::LoadFile(configFilePath);

            this->robot = robot;
            std::string gaitName = config["gait_params"]["gaitName"].as<std::string>();

            std::vector<float> stanceDurationList = config["gait_params"][gaitName]["stance_duration"].as<std::vector<float>>();
            stanceDuration = Eigen::MatrixXf::Map(&stanceDurationList[0], 4, 1);
            std::vector<float> dutyFactorList = config["gait_params"][gaitName]["duty_factor"].as<std::vector<float >>();
            dutyFactor = Eigen::MatrixXf::Map(&dutyFactorList[0], 4, 1);
            std::vector<int> initialLegStateList = config["gait_params"][gaitName]["initial_leg_state"].as<std::vector<int >>();
            initialLegState = Eigen::MatrixXi::Map(&initialLegStateList[0], 4, 1);
            std::vector<float> initialLegPhaseList = config["gait_params"][gaitName]["init_phase_full_cycle"].as<std::vector<float >>();
            initialLegPhase = Eigen::MatrixXf::Map(&initialLegPhaseList[0], 4, 1);
            contactDetectionPhaseThreshold = config["gait_params"][gaitName]["contact_detection_phase_threshold"].as<float>();
            
            for (int legId = 0; legId < initialLegState.size(); legId++) {
                // when dutyFactor is close 0, the leg stay in air.
                if (math::almostEqual(dutyFactor[legId],0.f, 0.001f)) {
                    swingDuration[legId] = 1e3;
                    initialLegState[legId] = LegState::USERDEFINED_SWING;
                    curLegState[legId] =  LegState::USERDEFINED_SWING;
                    lastLegState[legId] = LegState::USERDEFINED_SWING;
                    nextLegState[legId] = LegState::USERDEFINED_SWING;
                } else {
                    swingDuration[legId] = stanceDuration[legId]/dutyFactor[legId] - stanceDuration[legId];
                    lastLegState = initialLegState;
                    if (initialLegState[legId] == LegState::SWING) {
                        initStateRatioInCycle[legId] = 1 - dutyFactor[legId];
                        nextLegState[legId] = LegState::STANCE;
                    } else {
                        initStateRatioInCycle[legId] = dutyFactor[legId];
                        nextLegState[legId] = LegState::SWING;
                    }
                }
            }

            Reset(0);
        }

    void qrGaitGenerator::Reset(float currentTime) 
    {
        normalizedLegPhase = Eigen::Matrix<float, 4, 1>::Zero();
        lastLegState = initialLegState;
        curLegState = lastLegState;
        legState = initialLegState;
        desiredLegState = initialLegState;
    }

    void qrGaitGenerator::Update(float currentTime) 
    {
        Eigen::Matrix<bool, 4, 1> contactState = robotState->GetFootContact();
        float fullCyclePeriod, augmentedTime, phaseInFullCycle, ratio;
        

        for (int legId = 0; legId < initialLegState.size(); legId++) {
            if (initialLegState[legId] == LegState::USERDEFINED_SWING) {
                desiredLegState[legId] = initialLegState[legId];
                legState[legId] = desiredLegState[legId];
                continue;
            }
            
            // if (!robot->stop || (robot->stop && lastLegState[legId]==LegState::SWING)) {
            //     lastLegState[legId] = desiredLegState[legId];
            //     curLegState[legId] = desiredLegState[legId];
            // }
            
            fullCyclePeriod = stanceDuration[legId] / dutyFactor[legId];
            augmentedTime = initialLegPhase[legId] * fullCyclePeriod + currentTime;
            phaseInFullCycle = fmod(augmentedTime, fullCyclePeriod) / fullCyclePeriod;
            
            ratio = initStateRatioInCycle[legId];

            if (phaseInFullCycle < ratio) {
                desiredLegState[legId] = initialLegState[legId];
                normalizedLegPhase[legId] = phaseInFullCycle / ratio;
                
            } else {
                desiredLegState[legId] = nextLegState[legId];
                normalizedLegPhase[legId] = (phaseInFullCycle - ratio) / (1 - ratio);
            }
            
            legState[legId] = desiredLegState[legId];

            // No contact detection at the beginning of each SWING/STANCE phase.
            if (normalizedLegPhase[legId] < contactDetectionPhaseThreshold) {
                continue;
            }
            
            if (legState[legId] == LegState::SWING && contactState[legId]) {
                legState[legId] = LegState::EARLY_CONTACT;
            }

            if (legState[legId] == LegState::STANCE && !contactState[legId]) {
                legState[legId] = LegState::LOSE_CONTACT;
            }
        }
    }

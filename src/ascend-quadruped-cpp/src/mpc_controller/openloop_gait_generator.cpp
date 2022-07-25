/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Openloop gait generator
* Author: Xie Ming Cheng
* Create: 2021-10-25
* Notes: xx
* Modify: add head comment and add some function comments. @ Xie_mingcheng 2021.11.22
*/

#include "mpc_controller/openloop_gait_generator.h"

using namespace Eigen;
using namespace std;
namespace Quadruped {
    OpenloopGaitGenerator::OpenloopGaitGenerator() {}
    
    OpenloopGaitGenerator::OpenloopGaitGenerator(Robot *robot,
                                                 Eigen::Matrix<float, 4, 1> stanceDuration,
                                                 Eigen::Matrix<float, 4, 1> dutyFactor,
                                                 Eigen::Matrix<int, 4, 1> initialLegState,
                                                 Eigen::Matrix<float, 4, 1> initialLegPhase,
                                                 float contactDetectionPhaseThreshold)
    {
        this->robot = robot;
        this->stanceDuration = stanceDuration;
        this->dutyFactor = dutyFactor;
        this->initialLegState = initialLegState;
        this->initialLegPhase = initialLegPhase;
        this->contactDetectionPhaseThreshold = contactDetectionPhaseThreshold;
        this->swingDuration = stanceDuration.cwiseQuotient(dutyFactor) - stanceDuration;

        for (int legId = 0; legId < initialLegState.size(); legId++) {
            if (initialLegState[legId] == LegState::SWING) {
                initStateRadioInCycle[legId] = 1 - dutyFactor[legId];
                nextLegState[legId] = LegState::STANCE;
            } else {
                initStateRadioInCycle[legId] = dutyFactor[legId];
                nextLegState[legId] = LegState::SWING;
            }
        }

        this->Reset(0);
    }

    OpenloopGaitGenerator::OpenloopGaitGenerator(Robot *robot, string configFilePath)
    {

        this->configFilePath = configFilePath;
        config = YAML::LoadFile(configFilePath);

        this->robot = robot;
        string gait = config["gait_params"]["gait"].as<string>();
        cout << "OpenLoopGaitGenerator Set gait: " << gait << endl;

        vector<float> stanceDurationList = config["gait_params"][gait]["stance_duration"].as<vector<float >>();
        stanceDuration = Eigen::MatrixXf::Map(&stanceDurationList[0], 4, 1);
        vector<float> dutyFactorList = config["gait_params"][gait]["duty_factor"].as<vector<float >>();
        dutyFactor = Eigen::MatrixXf::Map(&dutyFactorList[0], 4, 1);
        vector<int> initialLegStateList = config["gait_params"][gait]["initial_leg_state"].as<vector<int >>();
        initialLegState = Eigen::MatrixXi::Map(&initialLegStateList[0], 4, 1);
        vector<float> initialLegPhaseList = config["gait_params"][gait]["init_phase_full_cycle"].as<vector<float >>();
        initialLegPhase = Eigen::MatrixXf::Map(&initialLegPhaseList[0], 4, 1);
        contactDetectionPhaseThreshold = config["gait_params"][gait]["contact_detection_phase_threshold"].as<float>();
        
        for (int legId = 0; legId < initialLegState.size(); legId++) {
            // when dutyFactor is about 0, this leg stay in air.
            if (robotics::math::almostEqual(dutyFactor[legId],0.f, 0.001f)) {
                swingDuration[legId] = 1e3;
                initialLegState[legId] = LegState::USERDEFINED_SWING;
                curLegState[legId] =  LegState::USERDEFINED_SWING;
                lastLegState[legId] = LegState::USERDEFINED_SWING;
                nextLegState[legId] = LegState::USERDEFINED_SWING;
                printf("Leg [%i] is userdefined leg!\n", legId);
            } else {
                swingDuration[legId] = stanceDuration[legId]/dutyFactor[legId] - stanceDuration[legId];
                lastLegState = initialLegState;
                if (initialLegState[legId] == LegState::SWING) {
                    initStateRadioInCycle[legId] = 1 - dutyFactor[legId];
                    nextLegState[legId] = LegState::STANCE;
                } else {
                    initStateRadioInCycle[legId] = dutyFactor[legId];
                    nextLegState[legId] = LegState::SWING;
                }
            }
        }
        // std::cout << "initialLegState" << initialLegState <<std::endl;
        Reset(0);
    }

    void OpenloopGaitGenerator::Reset(float currentTime)
    {
        normalizedPhase = Eigen::Matrix<float, 4, 1>::Zero();
        lastLegState = initialLegState;
        curLegState = lastLegState;
        legState = initialLegState;
        desiredLegState = initialLegState;
    }

    void OpenloopGaitGenerator::Update(float currentTime)
    {
        Eigen::Matrix<bool, 4, 1> contactState = robot->GetFootContacts();
        float fullCyclePeriod, augmentedTime, phaseInFullCycle, ratio;
        
        count++;

        for (int legId = 0; legId < initialLegState.size(); legId++) {
            if (initialLegState[legId] == LegState::USERDEFINED_SWING) {
                desiredLegState[legId] = initialLegState[legId];
                legState[legId] = desiredLegState[legId];
                continue;
            }
            
            
            if (!robot->stop || (robot->stop && lastLegState[legId]==LegState::SWING)) {
                lastLegState[legId] = desiredLegState[legId];
                curLegState[legId] = desiredLegState[legId];
            }
            
            fullCyclePeriod = stanceDuration[legId] / dutyFactor[legId];
            augmentedTime = initialLegPhase[legId] * fullCyclePeriod + currentTime;
            phaseInFullCycle = fmod(augmentedTime, fullCyclePeriod) / fullCyclePeriod;
            // std::cout << "phaseInFullCycle = " <<phaseInFullCycle <<std::endl;
            
            ratio = initStateRadioInCycle[legId];
            if (phaseInFullCycle < ratio) {
                desiredLegState[legId] = initialLegState[legId];
                normalizedPhase[legId] = phaseInFullCycle / ratio;
                
            } else {
                desiredLegState[legId] = nextLegState[legId];
                normalizedPhase(legId) = (phaseInFullCycle - ratio) / (1 - ratio);
            }
            // printf("leg %d normalizedPhase = %f\n", legId, normalizedPhase(legId));
            
            legState[legId] = desiredLegState[legId];

            if (normalizedPhase[legId] < contactDetectionPhaseThreshold) {
                continue;
            }
            if (legState[legId] == LegState::SWING && contactState[legId]) {
                legState[legId] = LegState::EARLY_CONTACT;
            }
            if (legState[legId] == LegState::STANCE && !contactState[legId]) {
                legState[legId] = LegState::LOSE_CONTACT;
            }
        }
        // std::cout << "desiredLegState " << desiredLegState.transpose() << std::endl;
        // std::cout << "legState " << legState.transpose() << std::endl;
    }
} // namespace Quadruped

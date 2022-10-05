// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

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

using namespace Eigen;
using namespace std;

qrGaitGenerator::qrGaitGenerator() {}

qrGaitGenerator::qrGaitGenerator(qrRobot *robot,
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

qrGaitGenerator::qrGaitGenerator(qrRobot *robot, string configFilePath)
{

    this->configFilePath = configFilePath;
    config = YAML::LoadFile(configFilePath);

    this->robot = robot;
    string gait = config["gait_params"]["gait"].as<string>();
    cout << "qrGaitGenerator Set gait: " << gait << endl;
    this->CreateGait(gait);
    Reset(0);
}
void qrGaitGenerator::CreateGait(string gaitType)
{

    this->curGaitType = gaitType;
    vector<float> stanceDurationList = config["gait_params"][gaitType]["stance_duration"].as<vector<float >>();
    stanceDuration = Eigen::MatrixXf::Map(&stanceDurationList[0], 4, 1);
    vector<float> dutyFactorList = config["gait_params"][gaitType]["duty_factor"].as<vector<float >>();
    dutyFactor = Eigen::MatrixXf::Map(&dutyFactorList[0], 4, 1);
    vector<int> initialLegStateList = config["gait_params"][gaitType]["initial_leg_state"].as<vector<int >>();
    initialLegState = Eigen::MatrixXi::Map(&initialLegStateList[0], 4, 1);
    vector<float> initialLegPhaseList = config["gait_params"][gaitType]["init_phase_full_cycle"].as<vector<float >>();
    initialLegPhase = Eigen::MatrixXf::Map(&initialLegPhaseList[0], 4, 1);
    contactDetectionPhaseThreshold = config["gait_params"][gaitType]["contact_detection_phase_threshold"].as<float>();

    for (int legId = 0; legId < initialLegState.size(); legId++) {
        // when dutyFactor is about 0, the leg seems like keeping swinging
        // user can use this to make special swing trajectories
        if (math::almostEqual(dutyFactor[legId],0.f, 0.001f)) {
            swingDuration[legId] = 1e3;
            initialLegState[legId] = LegState::USERDEFINED_SWING;
            curLegState[legId] =  LegState::USERDEFINED_SWING;
            lastLegState[legId] = LegState::USERDEFINED_SWING;
            nextLegState[legId] = LegState::USERDEFINED_SWING;
            printf("Leg [%i] is userdefined leg!\n", legId);
        } else {
            // assume the block means one period, '\' means stance, '_' means swing,then if duty factor is 0.7:
            // when the leg starts with stance,its period is |\\\\\\\___|
            // otherwise, it will look like |___\\\\\\\|
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
    this->nextGaitType = this->curGaitType;
}

void qrGaitGenerator::ModifyGait() {
    if (this->nextGaitType != this->curGaitType) {
        this->curGaitType = this->nextGaitType;
        this->CreateGait(this->curGaitType);
    }
}

void qrGaitGenerator::Reset(float currentTime)
{
    normalizedPhase = Eigen::Matrix<float, 4, 1>::Zero();
    lastLegState = initialLegState;
    curLegState = lastLegState;
    legState = initialLegState;
    desiredLegState = initialLegState;
}

void qrGaitGenerator::Update(float currentTime)
{
    this->ModifyGait();
    Eigen::Matrix<bool, 4, 1> contactState = robot->GetFootContacts();
    float fullCyclePeriod, augmentedTime, ratio;

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
        phaseInFullCycle[legId] = fmod(augmentedTime, fullCyclePeriod) / fullCyclePeriod;
        
        ratio = initStateRadioInCycle[legId];
        if (phaseInFullCycle[legId] < ratio) {
            desiredLegState[legId] = initialLegState[legId];
            normalizedPhase[legId] = phaseInFullCycle[legId] / ratio;
            
        } else {
            desiredLegState[legId] = nextLegState[legId];
            normalizedPhase(legId) = (phaseInFullCycle[legId] - ratio) / (1 - ratio);
        }
        
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
}
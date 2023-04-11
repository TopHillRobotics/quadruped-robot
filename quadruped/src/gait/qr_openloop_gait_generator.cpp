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

#include "gait/qr_openloop_gait_generator.h"

using namespace Eigen;
using namespace std;
namespace Quadruped {
qrOpenLoopGaitGenerator::qrOpenLoopGaitGenerator() {}


qrOpenLoopGaitGenerator::qrOpenLoopGaitGenerator(qrRobot *robot,
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


qrOpenLoopGaitGenerator::qrOpenLoopGaitGenerator(qrRobot *robot, string configFilePath)
{
    this->robot = robot;
    config = YAML::LoadFile(configFilePath);

    gait = config["gait_params"]["gait"].as<string>();

    Reset(0);

    if (fullCyclePeriod[0]!=fullCyclePeriod[1] || fullCyclePeriod[1]!=fullCyclePeriod[2] || fullCyclePeriod[2]!=fullCyclePeriod[3]) {
        throw std::domain_error("open loop gait error!");
    }
    // std::cout << "initialLegState" << initialLegState <<std::endl;
}


void qrOpenLoopGaitGenerator::Reset(float currentTime)
{
    /* update params */
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
    waitTime = config["gait_params"]["wait_time"].as<float>();

    for (int legId = 0; legId < initialLegState.size(); legId++) {
        /* when dutyFactor is about 0, this leg stay in air. */
        if (robotics::math::almostEqual(dutyFactor[legId],0.f, 0.001f)) {
            swingDuration[legId] = 1e3;
            fullCyclePeriod[legId] = 1e3;
            swingTimeRemaining[legId] = 0.;
            initialLegState[legId] = LegState::USERDEFINED_SWING;
            curLegState[legId] =  LegState::USERDEFINED_SWING;
            lastLegState[legId] = LegState::USERDEFINED_SWING;
            nextLegState[legId] = LegState::USERDEFINED_SWING;
            printf("Leg [%i] is userdefined leg!\n", legId);
        } else {
            fullCyclePeriod[legId] = stanceDuration[legId]/dutyFactor[legId];
            swingDuration[legId] = fullCyclePeriod[legId] - stanceDuration[legId];
            lastLegState = initialLegState;
            if (initialLegState[legId] == LegState::SWING) {
                // initStateRadioInCycle[legId] = 1 - dutyFactor[legId];
                initStateRadioInCycle[legId] = dutyFactor[legId];
                nextLegState[legId] = LegState::STANCE;
                offset[legId] = initialLegPhase[legId] - (1 - dutyFactor[legId]);
            } else {
                initStateRadioInCycle[legId] = dutyFactor[legId];
                nextLegState[legId] = LegState::SWING;
                offset[legId] = initialLegPhase[legId];
            }
        }
    }

    qrGaitGenerator::Reset(currentTime);
    std::cout << "stanceDuration = " << stanceDuration << std::endl;
    std::cout << "dutyfactor: " << this->dutyFactor << std::endl;
}


void qrOpenLoopGaitGenerator::Update(float currentTime)
{
    timeSinceReset = currentTime;
    Schedule(currentTime);

    float augmentedTime, ratio;

    for (int legId = 0; legId < NumLeg; legId++) {
        if (curLegState[legId] == LegState::USERDEFINED_SWING) {
            continue;
        }
        if (allowSwitchLegState.cast<int>().sum() == 4) {
            if (!robot->stop || (robot->stop && lastLegState[legId]==LegState::SWING)) {
                lastLegState[legId] = curLegState[legId];
                curLegState[legId] = desiredLegState[legId];
            }

            augmentedTime = initialLegPhase[legId] * fullCyclePeriod[legId] + timeSinceReset;
            phaseInFullCycle[legId] = fmod(augmentedTime, fullCyclePeriod[legId]) / fullCyclePeriod[legId];
            // std::cout << "phaseInFullCycle = " << phaseInFullCycle[legId] <<std::endl;

            /* update desired leg state */
            ratio = initStateRadioInCycle[legId];
            if (phaseInFullCycle[legId] < ratio) {
                desiredLegState[legId] = LegState::STANCE;
                normalizedPhase[legId] = phaseInFullCycle[legId] / ratio;
            } else {
                desiredLegState[legId] = LegState::SWING;
                normalizedPhase(legId) = (phaseInFullCycle[legId] - ratio) / (1 - ratio);
                if (curLegState[legId] == LegState::STANCE) {
                    firstSwing[legId] = true;
                    contactStartPhase[legId] = 0;
                    firstStance[legId] = false;
                    swingTimeRemaining[legId] = swingDuration[legId];
                } else {
                    firstSwing[legId] = false;
                    swingTimeRemaining[legId] = swingDuration[legId] * (1-normalizedPhase(legId));
                }
            }

            // if (count > 0) {
            //     desiredLegState[legId] = LegState::SWING; // todo
            //     legState[legId] = desiredLegState[legId];
            //     // normalizedPhase[legId] = phaseInFullCycle[legId];
            //     normalizedPhase[legId] = 1.0;
            // }
            // printf("leg %d normalizedPhase = %f\n", legId, normalizedPhase(legId));
            if (legState[legId]==LegState::EARLY_CONTACT && desiredLegState[legId]==LegState::SWING) {
                continue;
            } else {
                legState[legId] = desiredLegState[legId];
            }

            if (normalizedPhase[legId] < contactDetectionPhaseThreshold) {
                continue;
            }
            //todo
            // for (int legId(0); legId< 4; ++legId) {
            //     robot->footContact[legId] = (desiredLegState[legId] == LegState::STANCE);
            //     // robot->footForce[legId] = 0;
            // }

            Eigen::Matrix<bool, 4, 1> contactState = robot->GetFootContact();
            if (legState[legId] == LegState::SWING && contactState[legId]) {
                legState[legId] = LegState::EARLY_CONTACT;
                contactStartPhase[legId] = phaseInFullCycle[legId] - 1.0f;
            }
            // if (legState[legId] == LegState::STANCE && !contactState[legId]) {
            //     legState[legId] = LegState::LOSE_CONTACT;
            // } // todo

            if (curLegState[legId] == LegState::SWING && (legState[legId] == LegState::EARLY_CONTACT ||  legState[legId] == LegState::STANCE)) {
                firstStance[legId] = true;
                firstSwing[legId] = false;
                firstStanceAngles = robot->GetMotorAngles();
                // swingTimeRemaining[legId] = swingDuration[legId];
            }
        }

    }

    lastTime = currentTime;
}


void qrOpenLoopGaitGenerator::Schedule(float currentTime)
{
    if (resetTime+fullCyclePeriod[0] < timeSinceReset) {
            resetTime = timeSinceReset;
            gaitCycle++;
    }
    timeSinceReset -= resetTime; // 0<=timeSinceReset<=fullCyclePeriod

    Eigen::Matrix<bool, 4, 1> contactState = robot->GetFootContact();
    allowSwitchLegState << true, true, true, true;

    if (gait=="advanced_trot") {
        for (int legId = 0; legId < NumLeg; ++legId) {
            if(curLegState[legId]==LegState::SWING
                && desiredLegState[legId]==LegState::STANCE
                && !contactState[legId]) {
                // LOSE_CONTACT
                // if (normalizedPhase[legId]>0.3) {
                    allowSwitchLegState[legId] = false;
                // }
            }
        }
        // int num = contactState.cast<int>().sum();
        // if (num<2) allowSwitchLegState << false,false, false, false;
        if (allowSwitchLegState.cast<int>().sum() < 4) {
            float dt_ = currentTime-lastTime;
            cumDt += dt_;
            if (cumDt > waitTime){ // only leave 1s for cushioning effect
                allowSwitchLegState.setOnes();
                return;
            }
            resetTime += dt_;
            // std::cout << "not switch leg state!" <<std::endl;
        } else {
            cumDt = 0;
        }
    }

}
} // Namespace Quadruped

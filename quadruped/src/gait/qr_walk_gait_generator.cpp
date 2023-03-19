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

#include "gait/qr_walk_gait_generator.h"

using namespace Eigen;
using namespace std;


namespace Quadruped {

qrWalkGaitGenerator::qrWalkGaitGenerator(qrRobot *robot,
                                    std::vector<SubLegState> stateSwitchQue,
                                    std::vector<float> stateRatioQue,
                                    Eigen::Matrix<float, 4, 1> stanceDuration,
                                    Eigen::Matrix<float, 4, 1> dutyFactor,
                                    Eigen::Matrix<int, 4, 1> initialLegState,
                                    Eigen::Matrix<float, 4, 1> initialLegPhase,
                                    float contactDetectionPhaseThreshold)
{
    this->robot = robot;
    this->stateSwitchQue = stateSwitchQue;
    this->stateRatioQue = stateRatioQue;
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


qrWalkGaitGenerator::qrWalkGaitGenerator(qrRobot *robot, string configFilePath)
{

    config = YAML::LoadFile(configFilePath);

    this->robot = robot;
    // string gait = config["gait_params"]["gait"].as<string>();
    string gait = "walk";
    cout << "GaitGenerator Set gait: " << gait << endl;

    /* read some parameters in yaml file, you can find an example at /src/robot/quadruped/config/a1
       /openloop_gait_generateor.yaml */
    vector<float> stanceDurationList = config["gait_params"][gait]["stance_duration"].as<vector<float >>();
    stanceDuration = Eigen::MatrixXf::Map(&stanceDurationList[0], 4, 1);
    vector<float> dutyFactorList = config["gait_params"][gait]["duty_factor"].as<vector<float >>();
    dutyFactor = Eigen::MatrixXf::Map(&dutyFactorList[0], 4, 1);
    vector<int> initialLegStateList = config["gait_params"][gait]["initial_leg_state"].as<vector<int >>();
    initialLegState = Eigen::MatrixXi::Map(&initialLegStateList[0], 4, 1);
    vector<float>
        initialLegPhaseList = config["gait_params"][gait]["init_phase_full_cycle"].as<vector<float >>();
    initialLegPhase = Eigen::MatrixXf::Map(&initialLegPhaseList[0], 4, 1);
    contactDetectionPhaseThreshold = config["gait_params"][gait]["contact_detection_phase_threshold"].as<float>();

    if (gait == "walk") {
        std::cout << "initialLegPhase" << initialLegPhase << std::endl;
        vector<string> stateSwitchStrQue = config["gait_params"][gait]["state_switch_que"].as<vector<string>>();
        vector<float> stateRatioQue_ = config["gait_params"][gait]["state_ratio"].as<vector<float >>();
        moveBaseRatioPoint = config["gait_params"][gait]["move_base_ratio_point"].as<vector<float>>();

        assertm(stateSwitchStrQue.size() == stateRatioQue_.size(), "size different!");
        float standRatioInSwing = 0.f;
        int id = -1; // the id of state in walk cycle
        for(string& item:stateSwitchStrQue) {
            id++;
            if (stateRatioQue_[id]<0.01) {
                continue;
            }
            std::cout<<"item = "<< item<<"\n";
            if(item == "full_stance") {
                stateSwitchQue.push_back(SubLegState::FULL_STANCE);
                standRatioInSwing += stateRatioQue_[id];
            } else if (item == "load_force") {
                stateSwitchQue.push_back(SubLegState::LOAD_FORCE);
                standRatioInSwing += stateRatioQue_[id];
            } else if (item == "unload_force") {
                stateSwitchQue.push_back(SubLegState::UNLOAD_FORCE);
                standRatioInSwing += stateRatioQue_[id];
            } else if (item == "true_swing") {
                stateSwitchQue.push_back(SubLegState::TRUE_SWING);
                trueSwingStartPhaseInSwingCycle = standRatioInSwing;
                trueSwingStartPhaseInFullCycle = dutyFactor[0]+ (1-dutyFactor[0])*trueSwingStartPhaseInSwingCycle;

            }
            else {
                throw std::invalid_argument( "received invalid value!");
            }
            stateRatioQue.push_back(stateRatioQue_[id]);
        }
        trueSwingEndPhaseInFullCycle = trueSwingStartPhaseInFullCycle + (1-dutyFactor[0])*(1-standRatioInSwing);
        std::cout << "standRatioInSwing " << standRatioInSwing << std::endl;
        std::cout << "true swing begin at "<< trueSwingStartPhaseInFullCycle << ", end at "<< trueSwingEndPhaseInFullCycle << " in full cycle."<< std::endl;

        /* compute accumulation of state ratio from first full stand.
                       the sum of all ratio should be equals to 1.0 */
        accumStateRatioQue.push_back(0.);
        for(int i=0; i< stateRatioQue.size(); ++i) {
            accumStateRatioQue.push_back(accumStateRatioQue.back()+stateRatioQue[i]);
        }
        assertm(robotics::math::almostEqual(accumStateRatioQue.back(), 1.0f, 1e-4f), "not vaild ratio definition");

    } else {
        throw std::invalid_argument("not walk gait!");
    }

    for (int legId = 0; legId < NumLeg; ++legId) {
        if (robotics::math::almostEqual(dutyFactor[legId],0.f, 0.001f)) {
            swingDuration[legId] = 1e3;
            initialLegState[legId] = LegState::USERDEFINED_SWING;
            lastLegState[legId] = LegState::USERDEFINED_SWING;
            nextLegState[legId] = LegState::USERDEFINED_SWING;
            printf("Leg [%i] is user-defined leg!\n", legId);
        } else {
            /* check initialLegPhase matches with initialLegState */
            fullCyclePeriod[legId] = stanceDuration[legId]/dutyFactor[legId];
            swingDuration[legId] = fullCyclePeriod[legId] - stanceDuration[legId];
            moveBaseTime = fullCyclePeriod[0] * (moveBaseRatioPoint[1]-moveBaseRatioPoint[0]); // 8*  1.75

            if (initialLegState[legId] == LegState::SWING) {
                float pahseInSwingCycle = (initialLegPhase[legId] - dutyFactor[legId])/dutyFactor[legId];
                int i=0;
                while(i<stateRatioQue.size() && pahseInSwingCycle > accumStateRatioQue[i])
                {
                    i++;
                }
                /* make sure the leg is in which state process index (start from 0). */
                stateIndexOfLegs[legId] = std::max(i-1,0);
                curStateRadio[legId] = stateRatioQue[stateIndexOfLegs[legId]];
                // curLegState[legId] = LegState::SWING;
                initStateRadioInCycle[legId] = dutyFactor[legId];
                nextLegState[legId] = LegState::STANCE;
            } else {
                stateIndexOfLegs[legId] = 0;
                initStateRadioInCycle[legId] = dutyFactor[legId];
                nextLegState[legId] = LegState::SWING;
                // curLegState[legId] = LegState::STANCE;
            }
            // nextLegState[legId] = stateSwitchQue[(stateIndexOfLegs[legId]+1)%stateSwitchQue.size()];
            std::cout << "legId="<< legId<<"index = " <<stateIndexOfLegs[legId]<<std::endl;

        }

    }

    this->Reset(0);
    std::cout << "stateRatioQue ";
    for(int i=0;i<stateRatioQue.size(); ++i) {
         std::cout << stateRatioQue[i] <<" ";
    }
    std::cout << "ACCstateRatioQue ";
    for(int i=0;i<accumStateRatioQue.size(); ++i) {
         std::cout << accumStateRatioQue[i] <<" ";
    }
    std::cout<<"\n";
    std::cout << "stateSwitchQue ";
    for(int i=0;i<stateSwitchQue.size(); ++i) {
         std::cout << stateSwitchQue[i] <<" ";
    }
    std::cout<<"\n";
}


void qrWalkGaitGenerator::Reset(float currentTime)
{
    qrGaitGenerator::Reset(currentTime);
}


void qrWalkGaitGenerator::Update(float currentTime)
{
    lastLegState = curLegState;
    Eigen::Matrix<bool, 4, 1> contactState = robot->GetFootContact();

    float augmentedTime, ratio;

    for (int legId = 0; legId < NumLeg; ++legId) {

        if (initialLegState[legId] == LegState::USERDEFINED_SWING) {
            desiredLegState[legId] = initialLegState[legId];
            curLegState[legId] = initialLegState[legId];
            legState[legId] = desiredLegState[legId];
            continue;
        }

        if (!robot->stop || (robot->stop && curLegState[legId]==LegState::SWING)) {
            curLegState[legId] = desiredLegState[legId];
        }

        augmentedTime = initialLegPhase[legId] * fullCyclePeriod[legId] + currentTime;
        phaseInFullCycle[legId] = fmod(augmentedTime, fullCyclePeriod[legId]) / fullCyclePeriod[legId];
        std::cout << "phaseInFullCycle " << phaseInFullCycle[legId] << std::endl;

        ratio = dutyFactor[legId]; // initStateRadioInCycle[legId];
        if (phaseInFullCycle[legId] <= dutyFactor[legId]) { // stance
            if(curLegState[legId]!=LegState::STANCE) { // compute the next stateIdx
                stateIndexOfLegs[legId] = 0;
            }
            desiredLegState[legId] = LegState::STANCE;
            legState[legId] = desiredLegState[legId];
            normalizedPhase[legId] = phaseInFullCycle[legId] / ratio;
            // moveBasePhase =0;
        } else { // swing
            desiredLegState[legId] = LegState::SWING;
            legState[legId] = desiredLegState[legId];
            normalizedPhase(legId) = (phaseInFullCycle[legId]-ratio) / (1.0-ratio);
        }
        /*SUB SWING */
        if (desiredLegState[legId]== LegState::SWING) {
            int stateIdx = stateIndexOfLegs[legId];
            float subRatio = stateRatioQue[stateIndexOfLegs[legId]];
            float startRatioPoint = accumStateRatioQue[stateIdx];
            float endRatioPoint = accumStateRatioQue[stateIdx+1]; // startRatioPoint < endRatioPoint
            // printf("startRatioPoint %f, endRatioPoint %f\n", startRatioPoint, endRatioPoint);
            float phaseInSwingCycle = (phaseInFullCycle[legId]-dutyFactor[legId]) / (1.0-dutyFactor[legId]);
            printf("[leg %d] phaseInSwingCycle = %f\n", legId, phaseInSwingCycle);
            // phaseInSwingCycle = max(0.f, min(1.0f, phaseInSwingCycle));
            if (phaseInSwingCycle <= endRatioPoint && phaseInSwingCycle >= startRatioPoint) { // not finish this state process
                desiredLegState[legId] = stateSwitchQue[stateIdx];
                normalizedPhase[legId] = (phaseInSwingCycle-startRatioPoint) / (endRatioPoint - startRatioPoint);
            } else { // go into next state process
                    stateIdx+=1;
                    // printf("go into next state process %d\n", stateIdx);
                    desiredLegState[legId] = stateSwitchQue[stateIdx];
                    stateIndexOfLegs[legId] = stateIdx;
                    normalizedPhase(legId) = (phaseInSwingCycle - accumStateRatioQue[stateIdx]) / stateRatioQue[stateIdx];

            }
            if (phaseInSwingCycle < trueSwingStartPhaseInSwingCycle) {
                moveBasePhase = phaseInSwingCycle / trueSwingStartPhaseInSwingCycle;
            } else {
                moveBasePhase = 1.0f;
            }
        }
        // printf("leg %d normalizedPhase = %f\n", legId, normalizedPhase(legId));
        if (desiredLegState[legId]!=LegState::STANCE) {
            detectedLegState[legId] = LegState::SWING;
        } else {
            detectedLegState[legId] = LegState::STANCE;
        }


        if (normalizedPhase[legId] < contactDetectionPhaseThreshold) {
            continue;
        }
        if (desiredLegState[legId] == SubLegState::TRUE_SWING && contactState[legId]) {
            detectedLegState[legId] = LegState::EARLY_CONTACT;
            detectedEventTickPhase[legId] = phaseInFullCycle[legId];
        } else if (desiredLegState[legId] == LegState::STANCE && !contactState[legId]) {
            detectedLegState[legId] = LegState::LOSE_CONTACT;
            detectedEventTickPhase[legId] = phaseInFullCycle[legId];
        }

    }
    std::cout << "foot contact: " << robot->GetFootContact().transpose() << std::endl;
    std::cout << "curlegState " << curLegState.transpose() <<std::endl;
    std::cout << "desiredLegState " << desiredLegState.transpose() <<std::endl;
}

} // Namespace Quadruped

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


#ifndef QR_GAIT_GENERATOR_H
#define QR_GAIT_GENERATOR_H

#include <math.h>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "common/qr_types.h"
#include "common/qr_cpp_types.h"
#include "common/qr_algebra.h"

/**
 * @brief The qrGaitGenerator class generate the corresponding gait 
 * with the defined gait parameters and reset and update them regularly.
 */
namespace Quadruped {
    class qrGaitGenerator {
        public:
            /**
             * @brief Default constructor that constructs a qrGaitGenerator object.
             * 
             */
            qrGaitGenerator();
            //TODO: add robot param

            /**
             * @brief Construct a qrGaitGenerator object using a given config file.
             * @param configFilePath the given config file.
             */
            qrGaitGenerator(std::string configFilePath);

            /**
             * @brief Construct a qrGaitGenerator object using defined param
             * @param stanceDuration define the amount of stance time in a gait cycle.
             * @param dutyFactor define stand duration as a proportion of the whole gait cycle
             * @param initialLegState define the state of the leg at initialization e.g. SWING/STAND
             * @param initialLegPhase define the control order between legs by phase difference
             * @param contactDetectionPhaseThreshold when the leg status is swing, used for identifying effectiveness of the contact dection judgement
            */
            qrGaitGenerator(Eigen::Matrix<float, 4, 1> stanceDuration,
                            Eigen::Matrix<float, 4, 1> dutyFactor,
                            Eigen::Matrix<int, 4, 1> initialLegState,
                            Eigen::Matrix<float, 4, 1> initialLegPhase,
                            float contactDetectionPhaseThreshold = 0.1f);

            virtual ~qrGaitGenerator() = default;

            /**
             * @brief reset of the gait parameters based on the current clock
             * @param currentTime the current clock
             */
            virtual void Reset(float currentTime);

            /**
             * @brief update of the gait parameters based on the current clock
             * @param currentTime the current clock
             */
            virtual void Update(float currentTime);

            /**
             * @brief the config file path 
             */
            std::string configFilePath;

            /**
             * @brief the yaml object for load yaml config file 
             */
            YAML::Node config;

            // Robot *robot;

            /**
             * @brief define the amount of stance time in a gait cycle.
             */
            Eigen::Matrix<float, 4, 1> stanceDuration;

            /**
             * @brief define the amount of swing time in a gait cycle.
             */
            Eigen::Matrix<float, 4, 1> swingDuration;

            /**
             * @brief the time period ratio for stance stage, i.e. dutyFactor = stanceDurtion/(stanceDurtion+swingDurtion).
             */
            Eigen::Matrix<float, 4, 1> dutyFactor;

            /**
             * @brief define the control order between legs by phase difference 
             */
            Eigen::Matrix<float, 4, 1> initialLegPhase;

            /**
             * @brief define the state of the leg at initialization e.g. SWING/STAND.
             */
            Eigen::Matrix<int, 4, 1> initialLegState;

            /**
             * @brief define whether the current leg should be standing or swinging after switching. 
             * e.g. SWING->STAND/STAND->SWING
             */
            Eigen::Matrix<int, 4, 1> nextLegState;

            /**
             * @brief define whether the current state of the leg should be standing or swinging
             */
            Eigen::Matrix<int, 4, 1> legState;

            /**
             * @brief the phase for the desired state. 
             * e.g. when leg state is stand, normalizedPhase = currentPhase / dutyFactor
             */
            Eigen::Matrix<float, 4, 1> normalizedPhase;

            /**
             * @brief define whether the current state of the leg should be standing or swinging at each moment 
             * e.g. legState = STAND, when normalizedPhase < 1.0 ,desiredLegState = legState
             * e.g. legState = STAND, when normalizedPhase >= 1.0, desiredLegState = nextLegState
             */
            Eigen::Matrix<int, 4, 1> desiredLegState;

            /**
             * @brief record the previous state of the current leg, 
             * used to determine whether the state has changed in Velocity Mode
             */
            Eigen::Matrix<int, 4, 1> lastLegState;

            /**
             * @brief record the previous state of the current leg, 
             * used to determine whether the state has changed in Position Mode or Walk Mode
             */
            Eigen::Matrix<int, 4, 1> curLegState;

            /**
             * @brief define stand duration or swing duration as a proportion of the whole gait cycle
             * e.g. if legState = STAND, value is dutyfactor else value is 1 - dutyfactor.
             */
            Eigen::Matrix<float, 4, 1> initStateRadioInCycle;

            /**
             * @brief duration of a gait cycle
             */
            Vec4<float> fullCyclePeriod;
            
            /**
             * @brief when the leg status is swing, used for identifying effectiveness of the contact dection judgement
             * 
             */
            float contactDetectionPhaseThreshold; 
    };
} // namespace Quadruped
#endif //QR_GAIT_GENERATOR_H
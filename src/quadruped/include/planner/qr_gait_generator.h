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

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "common/qr_types.h"
#include "common/qr_eigen_types.h"
#include "common/qr_algebra.h"
#include "robot/qr_robot.h"

/**
 * @brief The qrGaitGenerator class generates the corresponding gait 
 * uisng the defined gait parameters. This class also resets and updates these parameters regularly.
 */
    class qrGaitGenerator {
        public:

            /**
             * @brief Default constructor that constructs a qrGaitGenerator object.
             * 
             */
            qrGaitGenerator();
            

            /**
             * @brief Construct a qrGaitGenerator object using a given config file.
             * @param robot the class of robot state
             * @param configFilePath the given config file.
             */
            qrGaitGenerator(qrRobot *robot, std::string configFilePath);

            /**
             * @brief Construct a qrGaitGenerator object using the given parameters.
             * @param robot the class of robot state
             * @param stanceDuration specifies the amount of stance time for each leg in a gait cycle.
             * @param dutyFactor specifies the duty factor for each leg. dutyFactor represents the fraction of stance phase in the gait cycle.
             * @param initialLegState specifies the state (SWING or STANCE) of each leg at the initialization of generating a gait.
             * @param initialLegPhase specifies the relative phase for each leg at the initialization of generating a gait.
             * @param contactDetectionPhaseThreshold specifies the contact threshold when the leg state switches from SWING to STANCE.
            */
            qrGaitGenerator(qrRobot *robot,
                            Vec4<float> stanceDuration,
                            Vec4<float> dutyFactor,
                            Vec4<int> initialLegState,
                            Vec4<float> initialLegPhase,
                            float contactDetectionPhaseThreshold = 0.1f);

            /**
             * @brief Deconstruct a qrGaitGenerator object.
             */
            virtual ~qrGaitGenerator() = default;

            /**
             * @brief Reset the gait parameters using the given time.
             * @param currentTime the given time.
             */
            virtual void Reset(float currentTime);

            /**
             * @brief Update of the gait parameters using the given time.
             * @param currentTime the given time.
             */
            virtual void Update(float currentTime);
        
            qrRobot *robot;

            qrRobotState *robotState;

            /**
             * @brief the config file for loading the gait parameters.
             */
            std::string configFilePath;

            /**
             * @brief The yaml object for loading a yaml config file. 
             */
            YAML::Node config;

            /**
             * @brief the amount of stance time for each leg in a gait cycle.
             */
            Vec4<float> stanceDuration;

            /**
             * @brief the amount of swing time for each leg in a gait cycle.
             */
            Vec4<float> swingDuration;

            /**
             * @brief the fraction of the cycle for stance phase. 
             * dutyFactor = stanceDuration / (stanceDuration + swingDuration).
             * @note In a periodic gait, dutyFactor is the same for all the legs.
             */
            Vec4<float> dutyFactor;

            /**
             * @brief the relative phase for each leg at the initialization of generating a gait.
             * @note The one leg is assigned relative phase 0 and the others have the relative phases in the range [0,1).
             */
            Vec4<float> initialLegPhase;

            /**
             * @brief the relative phase for the desired state. 
             * @note when a leg state is STANCE, normalizedLegPhase = currentLegPhase / dutyFactor
             */
            Vec4<float> normalizedLegPhase;

            /**
             * @brief the state (SWING or STANCE) of each leg at the initialization of generating a gait.
             */
            Vec4<int> initialLegState;

            /**
             * @brief The new state of each leg when the current leg state switches, either from STAND to SWING, or from SWING to STAND. 
             * @note If the current state is SWING, the next state will be STAND. If the current state is STANCE, the next state will be SWING.
             */
            Vec4<int> nextLegState;

            /**
             * @brief The current state of each leg (either STANCE or SWING)
             */
            Vec4<int> legState;

            /**
             * @brief the desired state (either STANCE or SWING) of each leg at a given moment.
             * e.g. if legState = STANCE and normalizedLegPhase < 1.0,  desiredLegState = legState
             * e.g. if legState = STANCE and normalizedLegPhase >= 1.0, desiredLegState = nextLegState
             */
            Vec4<int> desiredLegState;

            /**
             * @brief the previous state of each leg, 
             * @note This is used to determine if the state changed in Velocity Mode
             */
            Vec4<int> lastLegState;

            /**
             * @brief The previous  (current) states of each leg, 
             * @note this is used to determine if the state changed in Position Mode or Walk Mode
             */
            Vec4<int> curLegState;

            /**
             * @brief the fraction of the cycle for stance phase or for swing phase at a moment.
             * @note If legState = STANCE, the value is dutyfactor; if legState = SWING, the value is (1 - dutyfactor).
             */
            Vec4<float> initStateRatioInCycle;

            /**
             * @brief the duration of a gait cycle. fullCyclePeriod = stanceDuration + swingDuration.
             */
            Vec4<float> fullCyclePeriod;
            
            /**
             * @brief the contact threshold when the leg state switches from SWING to STANCE.
             * 
             */
            float contactDetectionPhaseThreshold; 
    };
#endif //QR_GAIT_GENERATOR_H

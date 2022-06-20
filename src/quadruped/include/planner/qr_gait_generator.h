// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: Xinyu Zhang   email: tophill.robotics@gmail.com

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


namespace Quadruped {
    
    /// The qrGaitGenerator class manages all gait genration, including stance 
    /// and swing durations. The world also contains efficient memory
    /// management facilities.
    class qrGaitGenerator 
    {
        public:
            /// Default constructor that constructs a qrGaitGenerator object.
            /// @param gravity the world gravity vector.
            qrGaitGenerator();
        
            /// Construct a qrGaitGenerator object using a given config file.
            /// @param configFilePath the given config file.
            qrGaitGenerator(std::string configFilePath);

            /// Construct a qrGaitGenerator object using a given config file.
            /// @param stanceDuration the amount of stance time  in a gait.
            /// @param dutyFactor the percent of the total cycle which a given foot is on the ground.
            /// @param initialLegState the initial leg state (STANCE or SWING).
            /// @param initialLegPhase the given config file.
            /// @param contactDetectionPhaseThreshold the given config file.
            qrGaitGenerator(Eigen::Matrix<float, 4, 1> stanceDuration,
                            Eigen::Matrix<float, 4, 1> dutyFactor,
                            Eigen::Matrix<int, 4, 1> initialLegState,
                            Eigen::Matrix<float, 4, 1> initialLegPhase,
                            float contactDetectionPhaseThreshold = 0.1f);

            /// Deconstruct a qrGaitGenerator.
            virtual ~qrGaitGenerator() = default;


            // 
            virtual void Reset(float currentTime);

            // 
            virtual void Update(float currentTime);

        private:
            /// The loaded gait config file name.
            std::string configFilePath;
            
            /// load from config file
            YAML::Node config; 
            // Robot *robot;

            /// The period of time that the foot is in contact with the ground.
            Eigen::Matrix<float, 4, 1> stanceDuration;

            /// The period of time that the foot is not in contact with the ground. 
            Eigen::Matrix<float, 4, 1> swingDuration;
            
            /// The percent of the total cycle which a given foot is on the ground.
            /// i.e. stanceDuration / (stanceDuration + swingDuration)
            Eigen::Matrix<float, 4, 1> dutyFactor;

            /// The relative (normalized) phase at beginning. 
            /// Here, phase is the percent of the total cycle  which a given foot is on the ground. 
            /// The one leg is assigned relative phase 0 and the others have the relative phases in the range [0,1].
            Eigen::Matrix<float, 4, 1> initialLegPhase;
            
            /// The desired relative phase w.r.t a given leg. 
            /// Note that this is NOT the total periodic/cyclic duration.
            Eigen::Matrix<float, 4, 1> desiredLegPhase;

            // The phase w.r.t a given stage, NOT the total periodic/cyclic duration.
            Eigen::Matrix<float, 4, 1> normalizedPhase; // the phase for the desired state.


            /// The initial leg state (STANCE or SWING).
            Eigen::Matrix<int, 4, 1> initialLegState;

            /// stance/swing/early stane/lost stance
            Eigen::Matrix<int, 4, 1> desiredLegState;

            /// The next leg state (STANCE or SWING).
            Eigen::Matrix<int, 4, 1> nextLegState;

            /// The previous leg state (STANCE or SWING).
            Eigen::Matrix<int, 4, 1> lastLegState;

            // The planned current state, while legState is the state actually detected via senros.
            // curLegState is the planned current state, while legState is the state actually detected via senros.
            Eigen::Matrix<int, 4, 1> curLegState; 
            
            /// The current leg state (STANCE or SWING) detected via senros.
            Eigen::Matrix<int, 4, 1> legState;

          
            //Eigen::Matrix<int, 4, 1> desiredLegState;  // stance/swing/early stane/lost stance
            //Eigen::Matrix<int, 4, 1> lastLegState;
            //Eigen::Matrix<int, 4, 1> curLegState; // curLegState is the planned current state, while legState is the state actually detected via senros.
            Eigen::Matrix<float, 4, 1> initStateRadioInCycle;
            Vec4<float> fullCyclePeriod;
            
            // when the leg status is swing, used for identifying effectiveness of the contact dection judgement
            float contactDetectionPhaseThreshold; 
    };
} // namespace Quadruped
#endif //QR_GAIT_GENERATOR_H
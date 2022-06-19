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

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <math.h>
#include <string>
#include <vector>
#include "common/qr_types.h"
#include "common/qr_cpp_types.h"
#include "common/qr_algebra.h"

class qrGaitGenerator {
    public:
        qrGaitGenerator();
        //TODO: add robot param
        qrGaitGenerator(std::string configFilePath);

        qrGaitGenerator(Eigen::Matrix<float, 4, 1> stanceDuration,
                        Eigen::Matrix<float, 4, 1> dutyFactor,
                        Eigen::Matrix<int, 4, 1> initialLegState,
                        Eigen::Matrix<float, 4, 1> initialLegPhase,
                        float contactDetectionPhaseThreshold = 0.1f);

        virtual ~qrGaitGenerator() = default;
        virtual void Reset(float currentTime);
        virtual void Update(float currentTime);
    private:
        std::string configFilePath;
        // load from config file
        YAML::Node config; 
        // Robot *robot;
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
};
#endif //QR_GAIT_GENERATOR_H
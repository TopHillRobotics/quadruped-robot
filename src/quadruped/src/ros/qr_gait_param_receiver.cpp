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

#include "ros/qr_gait_param_receiver.h"

QrGaitParamReceiver::QrGaitParamReceiver(ros::NodeHandle &nhIn)
    : nh(nhIn)
{
    ROS_INFO("gait param topic: %s", gaitParamTopic.c_str());
    gaitParamSub = nh.subscribe(gaitParamTopic, 10, &QrGaitParamReceiver::GaitParamCallback, this);
}

void QrGaitParamReceiver::GaitParamCallback(const unitree_legged_msgs::GaitParameter::ConstPtr &input)
{
    this->gaitName = input->gaitName;
    this->stanceDuration << input->stanceDuration[0],
                            input->stanceDuration[1],
                            input->stanceDuration[2],
                            input->stanceDuration[3];
                            
    this->dutyFactor << input->dutyFactor[0],
                        input->dutyFactor[1],
                        input->dutyFactor[2],
                        input->dutyFactor[3];

    this->initialLegState << input->initLegState[0],
                            input->initLegState[1],
                            input->initLegState[2],
                            input->initLegState[3];

    this->initialLegPhase << input->initFullCycle[0],
                            input->initFullCycle[1],
                            input->initFullCycle[2],
                            input->initFullCycle[3];

    this->contactDetectionPhaseThreshold = input->contactDetectionPhaseThreshold;
    this->flag = true;
}



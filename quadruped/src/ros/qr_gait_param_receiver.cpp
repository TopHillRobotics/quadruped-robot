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

#include "ros/qr_gait_param_receiver.h"

qrGaitParamReceiver::qrGaitParamReceiver(ros::NodeHandle &nhIn)
    : nh(nhIn)
{
    ROS_INFO("gait param topic: %s", gaitParamTopic.c_str());
    gaitParamSub = nh.subscribe(gaitParamTopic, 10, &qrGaitParamReceiver::GaitParamCallback, this);
}

Vec4<float> qrGaitParamReceiver::GetStanceDuration() {
    return stanceDuration;
}

Vec4<float> qrGaitParamReceiver::GetDutyFactor() {
    return dutyFactor;
}

Vec4<int> qrGaitParamReceiver::GetInitialLegState() {
    return initialLegState;
}

Vec4<float> qrGaitParamReceiver::GetInitialLegPhase() {
    return initialLegPhase;
}

float qrGaitParamReceiver::GetContactDetectionPhaseThreshold() {
    return contactDetectionPhaseThreshold;
}

bool qrGaitParamReceiver::GetFlag() {
    return flag;
}

std::string qrGaitParamReceiver::GetGaitName() {
    return gaitName;
}

void qrGaitParamReceiver::SetFlag() {
    flag = false;
}

void qrGaitParamReceiver::GaitParamCallback(const unitree_legged_msgs::GaitParameter::ConstPtr &msg)
{
    this->gaitName = msg->gaitName;

    this->stanceDuration << msg->stanceDuration[0],
                            msg->stanceDuration[1],
                            msg->stanceDuration[2],
                            msg->stanceDuration[3];
                            
    this->dutyFactor << msg->dutyFactor[0],
                        msg->dutyFactor[1],
                        msg->dutyFactor[2],
                        msg->dutyFactor[3];

    this->initialLegState << msg->initLegState[0],
                            msg->initLegState[1],
                            msg->initLegState[2],
                            msg->initLegState[3];

    this->initialLegPhase << msg->initFullCycle[0],
                            msg->initFullCycle[1],
                            msg->initFullCycle[2],
                            msg->initFullCycle[3];

    this->contactDetectionPhaseThreshold = msg->contactDetectionPhaseThreshold;
    
    this->flag = true;
}


// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:  tophill.robotics@gmail.com

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

#ifndef QR_GAIT_PARAM_RECEIVER_H
#define QR_GAIT_PARAM_RECEIVER_H

#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <unitree_legged_msgs/GaitParameter.h>
#include "common/qr_eigen_types.h"
/**
 * @brief  A qrGaitParamReceiver object receives the gait parameters from ROS topic when a gait updates.
 */
class qrGaitParamReceiver {
    public:
        /**
         * @brief Construct a qrGaitParamReceiver object using a given ROS nodeHandle.
         * @param nhIn specifies the ROS node to communicate
         */
        qrGaitParamReceiver(ros::NodeHandle &nhIn);

        /**
         * @brief Deconstruct a qrGaitParamReceiver object.
         */
        ~qrGaitParamReceiver() = default;

        /**
         * @brief Receive the updated information from a ROS topic.
         * @param msg the gait msg from a ROS topic
         */
        void GaitParamCallback(const unitree_legged_msgs::GaitParameter::ConstPtr &msg);
        
        /**
        * @brief Get the gait name.
        * @return string: gaitName  
        */
        std::string GetGaitName();

        /**
         * @brief Get the stance duration.
         * @return Vec4<float>: stanceDuration
         */
        Vec4<float> GetStanceDuration();

        /**
         * @brief Get the duty factor.
         * @return Vec4<float>: dutyFactor
         */
        Vec4<float> GetDutyFactor();

        /**
        * @brief Get the state of each leg at the initialization.
        * @return Vec4<float>: initialLegState
        */
        Vec4<int> GetInitialLegState();

        /**
        * @brief Get the relative phase of each leg at the initialization.
        * @return Vec4<float>: initialLegPhase
        */
        Vec4<float> GetInitialLegPhase();
        
        /**
        * @brief Get the threshold for switching from SWING to STANCE using contact detection.
        * @return float: contactDetectionPhaseThreshold  
        */
        float GetContactDetectionPhaseThreshold();

        /**
        * @brief Get the current flag
        * @return bool: flag  
        */
        bool GetFlag();

          /**
        * @brief Set flag to be false
        */
        void SetFlag();


        ros::NodeHandle &nh;
        ros::Subscriber gaitParamSub;
        std::string gaitParamTopic = "/gaitParam";

    private:
        /**
         * @brief Whether the flag parameter has changed
         */
        bool flag = false;

        std::string gaitName;
        Vec4<float>  stanceDuration;
        Vec4<float>  dutyFactor;
        Vec4<int>    initialLegState;
        Vec4<float>  initialLegPhase;
        float contactDetectionPhaseThreshold=0.1f;
};
#endif //QR_GAIT_PARAM_RECEIVER_H

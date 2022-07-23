/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Recive the switch mode from ROS.
* Author: Zhu Linsen
* Create: 2022-01-29
* Notes: xx
* Modify: init the file. @ Zhu Linsen
*/

#ifndef ASCEND_QUADRUPED_CPP_SWITCHMODERECEIVER_H
#define ASCEND_QUADRUPED_CPP_SWITCHMODERECEIVER_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Int8.h>

namespace Quadruped {
    /**
     * @brief receive locomotion switch command from state machine.
     */
    class SwitchModeReceiver {

    public:
        SwitchModeReceiver(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn);

        ~SwitchModeReceiver() = default;

        void SwitchModeCallback(const std_msgs::Int8::ConstPtr &input);

        inline int GetSwitchMode()
        {
            return switchMode; // z component.
        }

        ros::NodeHandle &nh;
        ros::NodeHandle &privateNh;
        ros::Subscriber switchModeSub;
        std::string switchModeTopic = "/switch_mode";
    private:
        int switchMode = 2;

    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_SWITCHMODERECEIVER_H
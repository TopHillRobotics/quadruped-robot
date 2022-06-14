/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Recive the switch mode from ROS.
* Author: Zhu Linsen
* Create: 2022-01-29
* Notes: xx
* Modify: init the file. @ Zhu Linsen
*/

#include "ros/switch_mode_receiver.h"
namespace Quadruped {
    SwitchModeReceiver::SwitchModeReceiver(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn)
        : nh(nhIn), privateNh(privateNhIn)
    {
        ROS_INFO("switch mode topic: %s", switchModeTopic.c_str());
        switchModeSub = nh.subscribe(switchModeTopic, 10, &SwitchModeReceiver::SwitchModeCallback, this);
    }

    void SwitchModeReceiver::SwitchModeCallback(const std_msgs::Int8::ConstPtr &input)
    {
        switchMode = input->data;
    }
} // namespace Quadruped


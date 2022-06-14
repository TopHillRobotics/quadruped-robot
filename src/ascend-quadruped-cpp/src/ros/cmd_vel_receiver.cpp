/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: ROS communication with location module.
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-11-08
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#include "ros/cmd_vel_receiver.h"
namespace Quadruped {
    CmdVelReceiver::CmdVelReceiver(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn)
        : nh(nhIn), privateNh(privateNhIn)
    {
        ROS_INFO("command velocity topic: %s", cmdVelTopic.c_str());
        cmdVelSub = nh.subscribe(cmdVelTopic, 10, &CmdVelReceiver::CmdVelCallback, this);
    }

    void CmdVelReceiver::CmdVelCallback(const geometry_msgs::Twist::ConstPtr &input)
    {
        linearVel << input->linear.x, input->linear.y, input->linear.z;
        angularVel << input->angular.x, input->angular.y, input->angular.z;
        // ROS_INFO_STREAM("[CmdVelReceiver] received velocity command:" << " linear=" << input->linear.x << " angular=" << input->angular.z);
    }
} // namespace Quadruped


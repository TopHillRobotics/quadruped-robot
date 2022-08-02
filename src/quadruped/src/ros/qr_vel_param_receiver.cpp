/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: ROS communication with location module.
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-11-08
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#include "ros/qr_vel_param_receiver.h"
namespace Quadruped {
    qrVelocityParamReceiver::qrVelocityParamReceiver (ros::NodeHandle &nhIn)
        : nh(nhIn)
    {
        ROS_INFO("command velocity topic: %s", cmdVelTopic.c_str());
        cmdVelSub = nh.subscribe(cmdVelTopic, 10, &qrVelocityParamReceiver::CmdVelCallback, this);
    }

    void qrVelocityParamReceiver::CmdVelCallback(const geometry_msgs::Twist::ConstPtr &input)
    {
        linearVel << input->linear.x, input->linear.y, input->linear.z;
        angularVel << input->angular.x, input->angular.y, input->angular.z;
    }
} // namespace Quadruped


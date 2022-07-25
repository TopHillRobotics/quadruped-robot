/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: ROS communication with location module.
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-11-08
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#include "ros/qr_slam_param_receiver.h"
namespace Quadruped {
    qrSlamParamReceiver::qrSlamParamReceiver(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn)
        : nh(nhIn), privateNh(privateNhIn)
    {
        ROS_INFO("slam pose topic: %s", slamPoseTopic.c_str());
        poseSub = nh.subscribe(slamPoseTopic, 10, &qrSlamParamReceiver::PoseCallback, this);
    }

    void qrSlamParamReceiver::PoseCallback(const geometry_msgs::Pose::ConstPtr &input)
    {
        posePosition << input->position.x, input->position.y, input->position.z;
        poseOrientation << input->orientation.w, input->orientation.x, input->orientation.y, input->orientation.z;
        // ROS_INFO_STREAM("[qrSlamParamReceiver] slam pose:" << " x=" << input->position.x << " y=" << input->position.y);
    }
} // namespace Quadruped
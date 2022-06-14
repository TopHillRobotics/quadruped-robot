/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: ROS communication with location module.
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-11-08
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_POSERECEIVER_H
#define ASCEND_QUADRUPED_CPP_POSERECEIVER_H

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
namespace Quadruped {
    /**
     * @brief receive robot body pose from slam.
     */
    class SLAMPoseReceiver {

    public:
        SLAMPoseReceiver(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn);

        ~SLAMPoseReceiver() = default;

        void PoseCallback(const geometry_msgs::Pose::ConstPtr &input);

        Eigen::Matrix<float, 3, 1> posePosition = Eigen::Matrix<float, 3, 1>::Zero();
        // w x y z
        Eigen::Matrix<float, 4, 1> poseOrientation = Eigen::Matrix<float, 4, 1>::Zero();

        ros::NodeHandle &nh;
        ros::NodeHandle &privateNh;
        ros::Subscriber poseSub;
        std::string slamPoseTopic = "/slam_pose";
    private:

    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_POSERECEIVER_H
/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: ROS communication with location module.
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-11-08
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_CMDVELRECEIVER_H
#define ASCEND_QUADRUPED_CPP_CMDVELRECEIVER_H

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

namespace Quadruped {
    /**
     * @brief receive command velocity from high level planning module.
     */
    class CmdVelReceiver {

    public:
        CmdVelReceiver(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn);

        ~CmdVelReceiver() = default;

        void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &input);

        inline Eigen::Matrix<float, 3, 1> GetLinearVelocity()
        {
            return linearVel;
        }

        inline float GetAngularVelocity()
        {
            return angularVel[2]; // z component.
        }

        geometry_msgs::Twist cmdVel;
        ros::NodeHandle &nh;
        ros::NodeHandle &privateNh;
        ros::Subscriber cmdVelSub;
        std::string cmdVelTopic = "/cmd_vel";
    private:
        Eigen::Matrix<float, 3, 1> linearVel = Eigen::Matrix<float, 3, 1>::Zero();
        Eigen::Matrix<float, 3, 1> angularVel = Eigen::Matrix<float, 3, 1>::Zero();

    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_CMDVELRECEIVER_H
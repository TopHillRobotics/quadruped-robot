/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Publishing controller information over ROS to gazebo for vis.
* Author: Zhu Yijie
* Create: 2022-1-05
* Notes: None.
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_INCLUDE_ROS_CONTROL2GAZEBO_MSG_H_
#define ASCEND_QUADRUPED_CPP_INCLUDE_ROS_CONTROL2GAZEBO_MSG_H_

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <gazebo_msgs/GetLinkState.h>

#include "xpp_msgs/topic_names.h"
#include "xpp_msgs/TerrainInfo.h"
#include "xpp_msgs/RobotParameters.h"

#include "xpp_msgs/RobotStateCartesian.h"
#include "xpp_states/convert.h"
#include "xpp_states/robot_state_cartesian.h"

#include "mpc_controller/locomotion_controller.h"

namespace Quadruped {
    /**
     * @brief collect messages from gazebo and transpond to visualization module.
     */
    class Controller2GazeboMsg {
    public:
        Controller2GazeboMsg(Robot *robotIn, LocomotionController *locomotionController, ros::NodeHandle &nhIn);
        // void PublishPose();
        void PublishGazeboStateCallback();
    private:
        Robot *robot;
        LocomotionController *locomotionController;
        PosePlanner *posePlanner;

        ros::NodeHandle &nh;
        // ros::Publisher posePublish;
        ros::Publisher gazeboStatePublish;
        ros::Publisher gazeboParamPublish;
        ros::Publisher posePlannerPublish;
        ros::ServiceClient baseStateClient;
        tf::TransformBroadcaster desiredPoseFramebr;
        tf::Transform transform;
        Vec3<float> rIB; // body position in world frame.
        Quat<float> quat; // quaterion of body orientation
        
        // sensor_msgs::JointState jointMsg;
        // geometry_msgs::Pose plannedPose;
        ros::Time currentTime;
        ros::Time lastTime;
    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_INCLUDE_ROS_CONTROL2GAZEBO_MSG_H_

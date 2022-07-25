// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:tophill.robotics@gmail.com

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

#include "ros/qr_visual_gazebo_msg.h"

namespace Quadruped {

    qrVisualGazeboMsg::qrVisualGazeboMsg(Robot *robotIn,
                                                   qrLocomotionController *locomotionControllerIn,
                                                   ros::NodeHandle &nhIn)
        : robot(robotIn), nh(nhIn)
    {
        locomotionController = nullptr;
        posePlanner = locomotionControllerIn->GetPosePlanner();
        // locomotionControllerIn->GetRobotEstimator();
        // posePublish = nh.advertise<geometry_msgs::Pose>("/visual/pose_planner/xyz_line", 20);
        gazeboStatePublish = nh.advertise<xpp_msgs::RobotStateCartesian>(xpp_msgs::robot_state_current, 10);
        gazeboParamPublish = nh.advertise<xpp_msgs::RobotParameters>(xpp_msgs::robot_parameters, 10);
        posePlannerPublish = nh.advertise<geometry_msgs::Pose>(xpp_msgs::robot_state_desired,10);
        baseStateClient = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    
        lastTime = ros::Time::now();
        ROS_INFO("qrVisualGazeboMsg init success...");
    }

    void qrVisualGazeboMsg::PublishGazeboStateCallback()
    {
        auto curTime = ros::Time::now();
                
        const auto & q = robot->GetBaseOrientation();
        const auto & pose = robot->GetBasePosition();
        const auto & jointAngles = robot->GetMotorAngles();
        const auto& forces = robot->state.footForce;
        const auto& contacts = robot->GetFootContacts();
        const auto & footPos = robot->state.GetFootPositionsInWorldFrame();
        
        xpp::RobotStateCartesian robotstate(4);
        
        robotstate.t_global_ = robot->GetTimeSinceReset();
        if (true) {
            robotstate.base_.lin.p_.x() = pose[0];
            robotstate.base_.lin.p_.y() = pose[1];
            robotstate.base_.lin.p_.z() = pose[2];
            robotstate.base_.ang.q = Eigen::Quaterniond(q[0],q[1],q[2],q[3]);
        } else {
            gazebo_msgs::GetLinkState gls_request;
            if (baseStateClient.exists()) { 
                gls_request.request.link_name = std::string("a1_gazebo::base");
                gls_request.request.reference_frame=std::string("world"); 
                // ros::service::waitForService("/gazebo/get_link_state", -1);
                baseStateClient.call(gls_request);
                if (!gls_request.response.success) {
                     ROS_INFO("Get Gazebo link state not success!\n");      
                }
            } else { 
                ROS_INFO("Get Gazebo link state goes wrong!\n"); 
            }
            
            const auto & pose_ = gls_request.response.link_state.pose; 
            std::cout<< "pose = " << pose_ <<std::endl;
            robotstate.base_.lin.p_.x() = pose_.position.x;
            robotstate.base_.lin.p_.y() = pose_.position.y;
            robotstate.base_.lin.p_.z() = pose_.position.z;
            robotstate.base_.ang.q = Eigen::Quaterniond(pose_.orientation.w,
                                                        pose_.orientation.x,
                                                        pose_.orientation.y,
                                                        pose_.orientation.z);
        }
        auto& joint_states = robotstate.joint_states;
        
        for (int legId =0; legId<4; ++legId) {
            robotstate.ee_motion_.at(legId).p_.x() = footPos(0, legId);  // foot pose in world frame
            robotstate.ee_motion_.at(legId).p_.y() = footPos(1, legId);
            robotstate.ee_motion_.at(legId).p_.z() = footPos(2, legId);
            robotstate.ee_forces_.at(legId).z() = forces[legId]; // N
            robotstate.ee_contact_.at(legId) = contacts[legId];
            for(int j=0;j<3;++j) {
                joint_states.at(3*legId + j) = double(jointAngles[3*legId + j]);
            }
        }

        xpp_msgs::RobotParameters workspace;
        workspace.base_mass = 10.0;
        workspace.ee_names  = {"RF"}; // swing foot names
        geometry_msgs::Point p;
        p.x = 0;
        p.y = 0;
        p.z = -1;
        workspace.nominal_ee_pos = {p};

        if ((rIB - posePlanner->rIB).norm()>1e-3) {        
            // ROS_INFO("SEND POSE PLANNER'S POSE");
            rIB = posePlanner->rIB;
            quat = posePlanner->quat;
            geometry_msgs::Pose poseMsg;
            poseMsg.position.x = rIB[0];
            poseMsg.position.y = rIB[1];
            poseMsg.position.z = rIB[2];
            poseMsg.orientation.w = quat[0];
            poseMsg.orientation.x = quat[1];
            poseMsg.orientation.y = quat[2];
            poseMsg.orientation.z = quat[3];
        
            posePlannerPublish.publish(poseMsg);
        }
        gazeboParamPublish.publish(workspace);
        gazeboStatePublish.publish(xpp::Convert::ToRos(robotstate));
        
        transform.setOrigin( tf::Vector3(rIB[0], rIB[1],rIB[2]));
        transform.setRotation( tf::Quaternion(quat[0], quat[1], quat[2], quat[3]) );
        desiredPoseFramebr.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "desiredPoseFrame"));

    }
}

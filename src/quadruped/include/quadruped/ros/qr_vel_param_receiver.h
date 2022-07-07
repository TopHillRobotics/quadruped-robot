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

#ifndef QR_VEL_PARAM_RECEIVER_H
#define QR_VEL_PARAM_RECEIVER_H

#include <iostream>
#include <string>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include "common/qr_types.h"
#include "common/qr_eigen_types.h"

/**
 * @brief A qrVelocityParamReceiver object recieves the velocity parameters from a ROS topic
 * 
 */
class qrVelocityParamReceiver {
    public:
        /**
         * @brief Construct a qrVelocityParamReceiver object using a  ROS nodeHandle.
         * @param nhIn specifies the ROS node to communicate
         */
        qrVelocityParamReceiver(ros::NodeHandle &nhIn, std::string FilePath);

        /**
         * @brief Deconstruct a qrVelocityParamReceiver object.
         */
        ~qrVelocityParamReceiver() = default;

        /**
        * @brief Receive the updated information from a ROS topic
        * @param msg the vel  msg from a ROS topic
        */
        void VelocityParamCallback(const geometry_msgs::Twist::ConstPtr &msg);

        /**
        * @brief Get the linear velocity.
        * @return Vec3<float>: linearVel  
        */
        Vec3<float> GetLinearVelocity();

       /**
        * @brief Get the angular velocity.
        * @return Vec3<float>: angularVel  
        */
        Vec3<float> GetAngularVelocity();

        /**
        * @brief Get the angular velocity.
        * @param axis: 0: x, 1: y, 2: z
        * @return float: the z coordinate of the angular vel
        */
        float GetAngularVelocity(int axis);
        
        geometry_msgs::Twist velParam;
        ros::NodeHandle &nh;
        ros::Subscriber velParamSub;
        std::string velParamTopic = "/velocity_param";

    private:
        Vec3<float> linearVel = Vec3<float>::Zero();
        Vec3<float> angularVel = Vec3<float>::Zero();

};
#endif //QR_VEL_PARAM_RECEIVER_H

// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

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

#ifndef QR_CMDVELRECEIVER_H
#define QR_CMDVELRECEIVER_H

#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>


namespace Quadruped {

/**
 * @brief Receive command velocity from high level planning module.
 */
class qrCmdVelReceiver {

public:

    qrCmdVelReceiver(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn);

    ~qrCmdVelReceiver() = default;

    void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &input);

    inline Eigen::Matrix<float, 3, 1> GetLinearVelocity() {
        return linearVel;
    }

    inline float GetAngularVelocity() {
        return angularVel[2];
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

} // Namespace Quadruped

#endif // QR_CMDVELRECEIVER_H

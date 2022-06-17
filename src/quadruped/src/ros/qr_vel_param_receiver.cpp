// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: Xinyu Zhang   email: tophill.robotics@gmail.com

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

#include "ros/qr_vel_param_receiver.h"

QrVelocityParamReceiver::QrVelocityParamReceiver(ros::NodeHandle &nhIn)
    : nh(nhIn)
{
    ROS_INFO("velocity param topic: %s", velParamTopic.c_str());
    velParamSub = nh.subscribe(velParamTopic, 10, &QrVelocityParamReceiver::VelocityParamCallback, this);
}


Eigen::Matrix<float, 3, 1> QrVelocityParamReceiver::GetLinearVelocity() {
    return linearVel;
}

float QrVelocityParamReceiver::GetAngularVelocity() {
    return angularVel[2];
}

void QrVelocityParamReceiver::VelocityParamCallback(const geometry_msgs::Twist::ConstPtr &input)
{
    linearVel << input->linear.x, input->linear.y, input->linear.z;
    angularVel << input->angular.x, input->angular.y, input->angular.z;
}



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

#include "ros/qr_vel_param_receiver.h"

qrVelocityParamReceiver::qrVelocityParamReceiver(ros::NodeHandle &nhIn,std::string pathToNode)
    : nh(nhIn)
{
    ROS_INFO("velocity param topic: %s", velParamTopic.c_str());
    velParamSub = nh.subscribe(velParamTopic, 10, &qrVelocityParamReceiver::VelocityParamCallback, this);

    YAML::Node motionConfig = YAML::LoadFile("/home/xmc/Documents/quadruped-robot/src/demo/a1sim/trot_velocity_motion/config/motion_config.yaml");
    std::vector<float> linear = motionConfig["const_twist"]["linear"].as<std::vector<float >>();
    linearVel << linear[0], linear[1], linear[2];
    angularVel[2] = motionConfig["const_twist"]["angular"].as<float>();
}


Vec3<float> qrVelocityParamReceiver::GetLinearVelocity() {
    return linearVel;
}

Vec3<float> qrVelocityParamReceiver::GetAngularVelocity() {
    return angularVel;
}

float qrVelocityParamReceiver::GetAngularVelocity(int axis) {
    return angularVel[axis];
}

void qrVelocityParamReceiver::VelocityParamCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    linearVel << msg->linear.x, msg->linear.y, msg->linear.z;
    angularVel << msg->angular.x, msg->angular.y, msg->angular.z;
}



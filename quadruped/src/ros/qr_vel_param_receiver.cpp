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

#include "ros/qr_vel_param_receiver.h"
qrVelocityParamReceiver::qrVelocityParamReceiver (ros::NodeHandle &nhIn, std::string pathToNode)
    : nh(nhIn)
{
    ROS_INFO("command velocity topic: %s", cmdVelTopic.c_str());
    cmdVelSub = nh.subscribe(cmdVelTopic, 10, &qrVelocityParamReceiver::CmdVelCallback, this);
    
    bool isSim;
    nh.getParam("isSim", isSim);
    std::string prefix = isSim ? "sim" : "real";
    YAML::Node mainConfig = YAML::LoadFile(pathToNode + "/" + prefix + "_config/main.yaml");
    std::vector<float> linear = mainConfig["const_twist"]["linear"].as<std::vector<float >>();
    linearVel << linear[0], linear[1], linear[2];
    angularVel[2] = mainConfig["const_twist"]["angular"].as<float>();
}

void qrVelocityParamReceiver::CmdVelCallback(const geometry_msgs::Twist::ConstPtr &input)
{
    linearVel << input->linear.x, input->linear.y, input->linear.z;
    angularVel << input->angular.x, input->angular.y, input->angular.z;
}


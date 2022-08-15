// The MIT License

// Copyright (c) 2022
// qrRobot Motion and Vision Laboratory at East China Normal University
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

#ifndef QR_MSG_CONVERTER_H
#define QR_MSG_CONVERTER_H

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <yaml-cpp/yaml.h>
#include <map>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

class qrJoy2Twist
{
public:
    qrJoy2Twist(ros::NodeHandle &nh, std::string pathToNode);
    ~qrJoy2Twist();
    void JoyCmdCallback(const sensor_msgs::Joy::ConstPtr &joyMsg);
private:
    ros::NodeHandle &nh;
    std::string cmdVelTopic = "/velocity_param";
    std::string joyCmdTopic = "/joy";
    ros::Subscriber joySubscriber;
    ros::Publisher twistPublisher;
    geometry_msgs::Twist msg;
    double joyCmdVelXMax;
    double joyCmdVelYMax;
    double joyCmdVelZMax;
    double joyCmdVelRollMax;
    double joyCmdVelPitchMax;
    double joyCmdVelYawMax;
};

#endif //QR_MSG_CONVERTER_H
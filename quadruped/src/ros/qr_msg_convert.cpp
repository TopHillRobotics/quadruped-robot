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

#include "ros/qr_msg_convert.h"

qrJoy2Twist::qrJoy2Twist(ros::NodeHandle &nh, std::string pathToNode):nh(nh)
{
    twistPublisher = nh.advertise<geometry_msgs::Twist>(cmdVelTopic, 1);
    joySubscriber = nh.subscribe(joyCmdTopic, 10, &qrJoy2Twist::JoyCmdCallback, this);
    // joy config
    YAML::Node joyConfig = YAML::LoadFile(pathToNode + "/joy.yaml");
    joyCmdVelXMax = joyConfig["vel_linear_max"]["x"].as<float>();
    joyCmdVelYMax = joyConfig["vel_linear_max"]["y"].as<float>();
    joyCmdVelZMax = joyConfig["vel_linear_max"]["z"].as<float>();
    joyCmdVelRollMax = joyConfig["vel_angular_max"]["r"].as<float>();
    joyCmdVelPitchMax = joyConfig["vel_angular_max"]["p"].as<float>();
    joyCmdVelYawMax = joyConfig["vel_angular_max"]["y"].as<float>();
    // ros::spin();
}

void qrJoy2Twist::JoyCmdCallback(const sensor_msgs::Joy::ConstPtr &joyMsg)
{
    msg.linear.x = joyMsg->axes[1] * joyCmdVelXMax;
    msg.linear.y = joyMsg->axes[0] * joyCmdVelYMax;
    msg.linear.z = joyMsg->axes[7] * joyCmdVelZMax;
    msg.angular.x = joyMsg->axes[6] * joyCmdVelRollMax;
    msg.angular.y = joyMsg->axes[4] * joyCmdVelPitchMax;
    msg.angular.z = joyMsg->axes[3] * joyCmdVelYawMax;
    twistPublisher.publish(msg);
}

void qrJoy2Twist::BringUpJoyNode()
{
    system(std::string("rosrun joy joy_node -dev /dev/input/js0").c_str());
}
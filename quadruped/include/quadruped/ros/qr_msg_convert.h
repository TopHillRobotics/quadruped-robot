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

/**
 * @brief Class qrJoy2Twist is used to convert the message received from the joystick
 *        to Twist message which could be received by qrVelocityParamReceiver.
 */
class qrJoy2Twist
{
public:

    /**
     * @brief Constructor of qrJoy2Twist.
     * @param nh The ros node which this class create from.
     * @param pathToNode The path where the node is running.
     */
    qrJoy2Twist(ros::NodeHandle &nh, std::string pathToNode);

    /**
     * @brief Default destructor of qrJoy2Twist.
     */
    ~qrJoy2Twist();

    /**
     * @brief The callback function of the subscriber.
     * @param joyMsg The joy message received from the joystick.
     */
    void JoyCmdCallback(const sensor_msgs::Joy::ConstPtr &joyMsg);

    /**
     * @brief bring up joy node.
     */
    void BringUpJoyNode();

private:

    /**
     * @brief The ros node which this class create from.
     */
    ros::NodeHandle &nh;

    /**
     * @brief The topic which the converted message to publish to.
     */
    std::string cmdVelTopic = "/velocityParam";

    /**
     * @brief The topic which the joystick message to subscribe.
     */
    std::string joyCmdTopic = "/joy";

    /**
     * @brief The joystick message subscriber.
     */
    ros::Subscriber joySubscriber;

    /**
     * @brief The Twist message publisher.
     */
    ros::Publisher twistPublisher;

    /**
     * @brief The Twist message.
     */
    geometry_msgs::Twist msg;

    /**
     * @brief The max velocity in X direction.
     */
    double joyCmdVelXMax;

    /**
     * @brief The max velocity in Y direction.
     */
    double joyCmdVelYMax;

    /**
     * @brief The max velocity in Z direction.
     */
    double joyCmdVelZMax;

    /**
     * @brief The max velocity in roll.
     */
    double joyCmdVelRollMax;

    /**
     * @brief The max velocity in pitch.
     */
    double joyCmdVelPitchMax;

    /**
     * @brief The max velocity in yaw.
     */
    double joyCmdVelYawMax;
};

#endif //QR_MSG_CONVERTER_H

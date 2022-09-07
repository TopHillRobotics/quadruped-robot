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

#ifndef QR_TELEKEYBOARD_H
#define QR_TELEKEYBOARD_H

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

/**
 * @brief Class qrTeleKeyboard is used to convert the message received from the keyboard
 *        to Twist message which could be received by qrVelocityParamReceiver.
 */
class qrTeleKeyboard
{
public:

    /**
     * @brief Constructor of qrTeleKeyboard.
     * @param nh The ros node which this class create from.
     */
    qrTeleKeyboard(ros::NodeHandle &nhIn);

    /**
     * @brief Default destructor of qrJoy2Twist.
     */
    ~qrTeleKeyboard(){}

    /**
     * @brief Get the event from the keyboard and convert it to corresponding char.
     *        For non-blocking keyboard inputs.
     * @return The number of the event's index.
     */
    int getch();

    /**
     * @brief Keep receiving the event from the keyboard. Keep running until input 
     *        CTRL + v.
     */
    void run();

    /**
     * @brief If the keyboard stop receiving event.
     */
    bool finish;
    
private:

    /**
     * @brief The topic which the converted message to publish to.
     */
    std::string cmdTopic = "/velocityParam";

    /**
     * @brief The ros node which this class create from.
     */
    ros::NodeHandle &nh;
};

#endif //QR_TELEKEYBOARD_H

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

#ifndef QR_SWITCH_MODE_RECEIVER_H
#define QR_SWITCH_MODE_RECEIVER_H

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Int8.h>


namespace Quadruped {

/**
 * @brief Receive locomotion switch command from state machine.
 */
class qrSwitchModeReceiver {

public:

    qrSwitchModeReceiver(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn);

    ~qrSwitchModeReceiver() = default;

    void SwitchModeCallback(const std_msgs::Int8::ConstPtr &input);

    inline int GetSwitchMode() {
        return switchMode;
    };

    ros::NodeHandle &nh;

    ros::NodeHandle &privateNh;

    ros::Subscriber switchModeSub;

    std::string switchModeTopic = "/switch_mode";

private:

    int switchMode = 2;

};

} // Namespace Quadruped

#endif // QR_SWITCH_MODE_RECEIVER_H

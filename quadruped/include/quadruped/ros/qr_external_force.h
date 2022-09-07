// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:  tophill.robotics@gmail.com

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


#ifndef QR_EXTERNAL_FORCE_H
#define QR_EXTERNAL_FORCE_H

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_UP    0x41
#define KEYCODE_DOWN  0x42
#define KEYCODE_LEFT  0x44
#define KEYCODE_RIGHT 0x43
#define KEYCODE_SPACE 0x20


int kfd = 0;
struct termios cooked, raw;
void quit(int sig);

class qrTeleForceCmd
{
public:
    qrTeleForceCmd();
   
    void keyLoop();
    void pubForce(double x, double y, double z);
private:
    double Fx, Fy, Fz;
    ros::NodeHandle n;
    ros::Publisher force_pub;
    geometry_msgs::Wrench Force;
    int kfd = 0;
    struct termios cooked, raw;
    int mode = 1; // pulsed mode or continuous mode
};
#endif //QR_EXTERNAL_FORCE_H

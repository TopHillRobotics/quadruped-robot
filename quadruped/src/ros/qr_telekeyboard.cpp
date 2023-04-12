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

#include "ros/qr_telekeyboard.h"

qrTeleKeyboard::qrTeleKeyboard(ros::NodeHandle &nhIn):nh(nhIn)
{
    finish = false;
    mutex = false;
}

int qrTeleKeyboard::getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

void qrTeleKeyboard::run()
{
    ros::Publisher pub = nh.advertise<sensor_msgs::Joy>(cmdTopic, 1);
    sensor_msgs::Joy joy_msg;
    char key = ' ';
    //initialize joy cmd
    joy_msg.buttons = {0,0,0,0,0,0,0,0,0,0,0};
    joy_msg.axes = {0,0,0,0,0,0,0,0};

    //joy's A X B Y Rb
    std::map<char, int> buttons_binding;
    buttons_binding['k'] = 0;//joy A
    buttons_binding['l'] = 1;//joy B
    buttons_binding['j'] = 2;//joy X
    buttons_binding['i'] = 3;//joy Y
    buttons_binding['o'] = 4;//joy RB
    buttons_binding['u'] = 5;//joy RL

    std::map<char, std::tuple<int, float>> rockers_binding;
    rockers_binding['w'] = {4, 0.8};
    rockers_binding['a'] = {3, 0.8};
    rockers_binding['s'] = {4, -0.8};
    rockers_binding['d'] = {3, -0.8};
    rockers_binding['q'] = {0, 1.0};
    rockers_binding['e'] = {0, -1.0};

    while (ros::ok()) {
        key = getch();
        mutex = true;

        if (buttons_binding.count(key) == 1) {
            joy_msg.buttons[buttons_binding[key]] = 1;
        }
        else if (rockers_binding.count(key) == 1) {
            joy_msg.axes[std::get<0>(rockers_binding[key])] =
                std::get<1>(rockers_binding[key]);
        }

        // receive Ctrl+C
        if (key == '\x03') {
            std::cout << "Break from keyboard." << std::endl;
            finish = true;
            break;
        }


        pub.publish(joy_msg);

        //clear the joy_msg
        joy_msg.buttons = {0,0,0,0,0,0,0,0,0,0,0};
        joy_msg.axes = {0,0,0,0,0,0,0,0};

        ros::spinOnce();
        mutex = false;
    }
}

void qrTeleKeyboard::run_default()
{
    ros::Publisher pub = nh.advertise<sensor_msgs::Joy>(cmdTopic, 1);
    sensor_msgs::Joy joy_msg;
    joy_msg.buttons = {0,0,0,0,0,0,0,0,0,0,0};
    joy_msg.axes = {0,0,0,0,0,0,0,0};
    while (ros::ok()) {
        if (finish)
            break;
        if (!mutex)
            pub.publish(joy_msg);
        sleep(1);
        ros::spinOnce();
    }
}
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
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(cmdTopic, 1);
    geometry_msgs::Twist twMsg;

    // initialize speed.
    twMsg.linear.x = 0;
    twMsg.linear.y = 0;
    twMsg.linear.z = 0;
    twMsg.angular.x = 0;
    twMsg.angular.y = 0;
    twMsg.angular.z = 0;
    pub.publish(twMsg);

    std::map<char, std::vector<double>> moveBindings;
    moveBindings['w'] = std::vector<double>{1, 0, 0};
    moveBindings['s'] = std::vector<double>{-1, 0, 0};
    moveBindings['a'] = std::vector<double>{0, 1, 0};
    moveBindings['d'] = std::vector<double>{0, -1, 0};
    moveBindings['r'] = std::vector<double>{0, 0, 1};
    moveBindings['f'] = std::vector<double>{0, 0, -1};

    std::map<char,std::vector<double>> rotationBindings;
    rotationBindings['i'] = std::vector<double>{0, 1, 0};
    rotationBindings['k'] = std::vector<double>{0, -1, 0};
    rotationBindings['j'] = std::vector<double>{0, 0, 1};
    rotationBindings['l'] = std::vector<double>{0, 0, -1};
    rotationBindings['u'] = std::vector<double>{1, 0, 0};
    rotationBindings['o'] = std::vector<double>{-1, 0, 0};

    char key = ' ';
    std::vector<double> moveCmd = {0, 0, 0, 0};
    std::vector<double> rotationCmd = {0, 0, 0};
    double l_alpha = 0.15;
    double r_alpha = 0.3;
    while(true){
        key = getch();

        if(moveBindings.count(key) == 1){
            for(int i = 0; i < 3; ++i){
                moveCmd[i] = l_alpha * moveBindings[key][i] + (1 - l_alpha) * moveCmd[i];
            }
        } else if(rotationBindings.count(key) == 1){
            for(int i = 0; i < 3; ++i){
                rotationCmd[i] = r_alpha * rotationBindings[key][i] + (1 - r_alpha) * rotationCmd[i];
            }
        } else {
            for(int i = 0; i < 4; ++i){
                moveCmd[i] = 0;
            }
            for(int i = 0; i < 3; ++i){
                rotationCmd[i] = 0;
            }
        }
        // receive CTRL + v
        if(key == '\x03' || finish){
            std::cout << "Break from keyboard." << std::endl;
            finish = false;
            break;
        }
        twMsg.linear.x = moveCmd[0];
        twMsg.linear.y = moveCmd[1];
        twMsg.linear.z = moveCmd[2];
        twMsg.angular.x = rotationCmd[0];
        twMsg.angular.y = rotationCmd[1];
        twMsg.angular.z = rotationCmd[2];

        pub.publish(twMsg);
        ros::spinOnce();
    }
}

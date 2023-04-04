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

#ifndef QR_TOOLS_H
#define QR_TOOLS_H

#include <iostream>
#include <iomanip>
#include <string>
#include <limits.h>
#include <unistd.h>
#include <termios.h>

#include <ros/ros.h>

#include "utils/qr_visualization.h"
#include "utils/qr_print.hpp"


namespace robotics {

namespace utils {

/**
 * @brief Query the path of the executable file.
 */
std::string GetExePath();

/**
 * @brief Refer to https://answers.ros.org/question/63491/keyboard-key-pressed
 */
int getch1();

} // Namespace utils

} // Namespace robotics

#ifdef _useros
/**
 * @brief A toolkit for loading yaml config file correctly.
 */
std::string GetHomeDir(std::string homeName = "quadruped-robot/");
#else
/**
 * @brief A toolkit for loading yaml config file correctly.
 */
std::string GetHomeDir(std::string homeName = "quadruped-robot/");
#endif

#endif // QR_TOOLS_H

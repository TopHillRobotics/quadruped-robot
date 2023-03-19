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

#include "utils/qr_tools.h"


namespace robotics {

namespace utils {

/**
 * @brief Query the path of the executable file.
 */
std::string GetExePath()
{
    char result[PATH_MAX];
    std::size_t count = readlink("/proc/self/exe", result, PATH_MAX);
    return std::string(result, (count > 0) ? count : 0);
}


/**
 * @brief Refer to https://answers.ros.org/question/63491/keyboard-key-pressed
 */
int getch1()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering 
    newt.c_cc[VMIN] = 0; newt.c_cc[VTIME] = 0;     
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}

} // Namespace utils

} // Namespace robotics


#ifdef _useros
/**
 * @brief A toolkit for loading yaml config file correctly.
 */
std::string GetHomeDir(std::string homeName)
{
    std::string exepath = robotics::utils::GetExePath();
    std::size_t found = exepath.find(homeName);
    std::string homeDir = exepath.substr(0, found) + homeName + "src/quadruped/";
    std::cout << "[USE ROS]: " << homeDir << std::endl;
    return homeDir;
}
#else
/**
 * @brief A toolkit for loading yaml config file correctly.
 */
std::string GetHomeDir(std::string homeName)
{
    std::string exepath = robotics::utils::GetExePath();
    std::size_t found = exepath.find(homeName);
    std::string homeDir = exepath.substr(0, found) + homeName;
    std::cout << "[NO ROS]: " << homeDir << std::endl;
    return homeDir;
}
#endif

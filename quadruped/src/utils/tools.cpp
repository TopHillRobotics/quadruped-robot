/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: motor object on robots.
* Author: Zhu Yijie
* Create: 2021-11-22
* Notes: xx
* Modify: init the file. @ Zhu Yijie;
*/

#include "utils/tools.h"

namespace robotics {
    namespace utils {
        /** @brief query the path of the executable file. */
        std::string GetExePath()
        {
            char result[PATH_MAX];
            std::size_t count = readlink("/proc/self/exe", result, PATH_MAX);
            return std::string(result, (count > 0) ? count : 0);
        }

        /** @brief refer to https://answers.ros.org/question/63491/keyboard-key-pressed */
        int getch1() {
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

    } // utils
} // robotics


#ifdef _useros
/** @brief a toolkit for loading yaml config file correctly. */
std::string GetHomeDir(std::string homeName)
{
    std::string exepath = robotics::utils::GetExePath();
    std::size_t found = exepath.find(homeName);
    std::string homeDir = exepath.substr(0, found) + homeName + "src/ascend-quadruped-cpp/";
    std::cout << homeDir << std::endl;
    return homeDir;
}
#else
/** @brief a toolkit for loading yaml config file correctly. */
std::string GetHomeDir(std::string homeName)
{
    std::string exepath = robotics::utils::GetExePath();
    std::size_t found = exepath.find(homeName);
    std::string homeDir = exepath.substr(0, found) + homeName;
    std::cout << homeDir << std::endl;
    return homeDir;
}
#endif

/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: motor object on robots.
* Author: Zhu Yijie
* Create: 2021-11-22
* Notes: xx
* Modify: init the file. @ Zhu Yijie;
*/

#ifndef ASCEND_UTILS_TOOLS_H
#define ASCEND_UTILS_TOOLS_H

#include <iostream>
#include <string>
#include <limits.h>
#include <unistd.h>
#include <termios.h>
#include <ros/ros.h>
#include "cnpy.h"
#include <matplotlibcpp.h>
namespace plt = matplotlibcpp;


namespace robotics {
    namespace utils {
        /** @brief query the path of the executable file. */
        std::string GetExePath();
        

        /** @brief refer to https://answers.ros.org/question/63491/keyboard-key-pressed */
        int getch1();
        
    } // utils
} // robotics

#endif // ASCEND_UTILS_TOOLS_H
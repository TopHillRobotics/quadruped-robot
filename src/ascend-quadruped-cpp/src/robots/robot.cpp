/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Base class of quadruped robots.
* Author: Zhao Yao
* Create: 2021-11-3
* Notes: xx
* Modify: init the file. @ Zhao Yao 2021.11.19
*/
#include "robots/robot.h"
namespace Quadruped {
    Robot::Robot(std::string path):config(new RobotConfig(path)),state(RobotState(config))
    {
    }
} // namespace Quadruped

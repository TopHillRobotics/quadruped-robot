/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Base class of quadruped robots.
* Author: Zhao Yao
* Create: 2021-11-3
* Notes: xx
* Modify: init the file. @ Zhao Yao 2021.11.19
*/
#include "robots/qr_robot.h"
namespace Quadruped {
    qrRobot::qrRobot(std::string path):config(new RobotConfig(path)),state(RobotState(config))
    {
    }
} // namespace Quadruped

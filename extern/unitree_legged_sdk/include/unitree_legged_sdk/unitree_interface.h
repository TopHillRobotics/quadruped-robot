/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <array>
#include <math.h>
#include <iostream>


using namespace UNITREE_LEGGED_SDK;

class RobotInterface {
public:
    RobotInterface() : safe(LeggedType::A1), udp(LOWLEVEL) {
        // InitEnvironment();
        std::array<float, 60> initCmd = {0};
        SendCommand(initCmd);
        std::cout << "-------robot interface init-------" << std::endl;
        sleep(3);
        LowState initState = ReceiveObservation();
        std::cout << "gravity: " << initState.imu.accelerometer[2] << std::endl;
    }

    LowState ReceiveObservation();

    void SendCommand(std::array<float, 60> motorcmd);

    void Initialize();

    UDP udp;
    Safety safe;
    LowState state = {0};
    LowCmd cmd = {0};
};

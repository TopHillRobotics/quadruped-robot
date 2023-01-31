/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
#ifndef UNITREE_INTERFACE_H
#define UNITREE_INTERFACE_H

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <array>
#include <math.h>
#include <iostream>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;

class RobotInterface {
public:
    RobotInterface() : safe(LeggedType::Go1), udp(LOWLEVEL)
    {   
        std::cout << "-------robot interface v3.4 init-------" << std::endl;
        udp.InitCmdData(cmd);
    }

    void UDPRecv()
    {
        udp.Recv();
    }

    void UDPSend()
    {  
        udp.Send();
    }

    LowState ReceiveObservation();

    void SendCommand(std::array<float, 60> motorcmd);

    void Initialize1(void *func, void* robot) {

        // LoopFunc loop_control("control_loop", this->dt, boost::bind(func, this));
        // LoopFunc loop_udpSend("udp_send",     this->dt, 3, boost::bind(&RobotInterface::UDPSend, this));
        // LoopFunc loop_udpRecv("udp_recv",     this->dt, 3, boost::bind(&RobotInterface::UDPRecv, this));

        // loop_udpSend.start();
        // loop_udpRecv.start();
        // loop_control.start();

    }

    void Initialize2() {

        // LoopFunc loop_control("control_loop", this->dt, boost::bind(&RobotInterface::ReceiveObservation, this));
        LoopFunc loop_udpSend("udp_send",     this->dt, 1, boost::bind(&RobotInterface::UDPSend, this));
        LoopFunc loop_udpRecv("udp_recv",     this->dt, 1, boost::bind(&RobotInterface::UDPRecv, this));

        loop_udpSend.start();
        loop_udpRecv.start();
        // loop_control.start();

    }

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    int motiontime = 0;
    float dt = 0.001;     // 0.001~0.01
};


#endif // UNITREE_INTERFACE_H
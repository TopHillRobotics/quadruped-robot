/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_interface.h"
#include <array>
#include <math.h>


using namespace UNITREE_LEGGED_SDK;

LowState RobotInterface::ReceiveObservation() {
    udp.Recv();
    udp.GetRecv(state);
    return state;
}

void RobotInterface::SendCommand(std::array<float, 60> motorcmd) {
    cmd.levelFlag = 0xff;
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        cmd.motorCmd[motor_id].mode = 0x0A;
        cmd.motorCmd[motor_id].q = motorcmd[motor_id * 5];
        cmd.motorCmd[motor_id].Kp = motorcmd[motor_id * 5 + 1];
        cmd.motorCmd[motor_id].dq = motorcmd[motor_id * 5 + 2];
        cmd.motorCmd[motor_id].Kd = motorcmd[motor_id * 5 + 3];
        cmd.motorCmd[motor_id].tau = motorcmd[motor_id * 5 + 4];
    }
    safe.PositionLimit(cmd);
    udp.SetSend(cmd);
    udp.Send();
}

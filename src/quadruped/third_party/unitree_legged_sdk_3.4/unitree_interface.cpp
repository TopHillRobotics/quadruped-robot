#include "unitree_legged_sdk/unitree_interface.h"

using namespace UNITREE_LEGGED_SDK;

LowState RobotInterface::ReceiveObservation() {
    udp.GetRecv(state);
    // printf("[%u]: state.imu.rpy = %f %f %f\n", state.tick, state.imu.rpy[0],state.imu.rpy[1],state.imu.rpy[2]);
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
    // safe.PositionLimit(cmd);
    // safe.PositionProtect(cmd, state, 0.087);
    // safe.PowerProtect(cmd, state, 1);
    udp.SetSend(cmd);
}


// int main()
// {
//     RobotInterface robotInterface;

//     // robotInterface.Initialize2();
//     LoopFunc loop_control("control_loop", robotInterface.dt, boost::bind(&RobotInterface::ReceiveObservation, &robotInterface));
//     LoopFunc loop_udpSend("udp_send",     robotInterface.dt, 2, boost::bind(&RobotInterface::UDPSend, &robotInterface));
//     LoopFunc loop_udpRecv("udp_recv",     robotInterface.dt, 2, boost::bind(&RobotInterface::UDPRecv, &robotInterface));
    
//     loop_udpSend.start();
//     loop_udpRecv.start();
//     loop_control.start();
    
//     while(1){
//     //     robotInterface.ReceiveObservation();
//         sleep(10);
//     }
//     // loop_udpSend.shutdown();
//     // loop_udpRecv.shutdown();
// }
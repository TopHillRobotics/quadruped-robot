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

#include "robots/qr_robot_real.h"

qrRobotReal::qrRobotReal(std::string robotName, LocomotionMode mode):
    qrRobot(robotName + "_real", mode)
{

    usleep(300000); // must wait 300ms, to get first state
    timeStep = 0.001;
    this->ResetTimer();
    lastResetTime = GetTimeSinceReset();

    UpdateRobotState();
    std::cout << "-------qrRobotReal init Complete-------" << std::endl;
}

void qrRobotReal::SendCommand(const std::array<float, 60> motorcmd)
{
    robotInterface.SendCommand(motorcmd);
}

void qrRobotReal::UpdateRobotState()
{
    lowstate = robotInterface.ReceiveObservation();

    // update motorState
    for(int motorId = 0; motorId < qrRobotConfig::numMotors; ++motorId){
        state.motorState[motorId].q = lowstate.motorState[motorId].q;
        state.motorState[motorId].dq = lowstate.motorState[motorId].dq;
    }

    // update imu
    // set quaternion information
    state.imu.quaternion[0] = lowstate.imu.quaternion[0];
    state.imu.quaternion[1] = lowstate.imu.quaternion[1];
    state.imu.quaternion[2] = lowstate.imu.quaternion[2];
    state.imu.quaternion[3] = lowstate.imu.quaternion[3];

    Eigen::Matrix<float, 4, 1> quaternion = {state.imu.quaternion[0],
                                             state.imu.quaternion[1],
                                             state.imu.quaternion[2],
                                             state.imu.quaternion[3]};
    Eigen::Matrix<float, 3, 1> rpy = math::quatToRPY(quaternion);

    // set roll pitch yaw information
    state.imu.rpy[0] = rpy[0];
    state.imu.rpy[1] = rpy[1];
    state.imu.rpy[2] = rpy[2];

    // set angular acceleration
    state.imu.gyroscope[0] = lowstate.imu.gyroscope[0];
    state.imu.gyroscope[1] = lowstate.imu.gyroscope[1];
    state.imu.gyroscope[2] = lowstate.imu.gyroscope[2];

    // set linear acceleration
    state.imu.accelerometer[0] = lowstate.imu.accelerometer[0];
    state.imu.accelerometer[1] = lowstate.imu.accelerometer[1];
    state.imu.accelerometer[2] = lowstate.imu.accelerometer[2];

    // update foot force
    for(int i = 0; i < qrRobotConfig::numLegs; ++i) {
        state.footForce[i] = lowstate.footForce[i];
    }
    state.Update();
}

void qrRobotReal::ReceiveObservation()
{
    UpdateRobotState();
}

void qrRobotReal::ApplyAction(const Eigen::MatrixXf &motorCommands,
                        MotorMode motorControlMode)
{
    // std::cout << "apply:\n" << motorCommands << std::endl;
    std::array<float, 60> motorCommandsArray = {0};
    if (motorControlMode == POSITION_MODE) {// in position mode, motor needs q, kp. kd
        Eigen::Matrix<float, 1, 12> motorCommandsShaped = motorCommands.transpose();
        for (int motorId = 0; motorId < qrRobotConfig::numMotors; motorId++) {
            motorCommandsArray[motorId * 5] = motorCommandsShaped[motorId];
            motorCommandsArray[motorId * 5 + 1] = config->motorKps[motorId];
            motorCommandsArray[motorId * 5 + 2] = 0;
            motorCommandsArray[motorId * 5 + 3] = config->motorKds[motorId];
            motorCommandsArray[motorId * 5 + 4] = 0;
        }
    } else if (motorControlMode == TORQUE_MODE) {// in torque mode, motor needs joint torque
        Eigen::Matrix<float, 1, 12> motorCommandsShaped = motorCommands.transpose();
        for (int motorId = 0; motorId < qrRobotConfig::numMotors; motorId++) {
            motorCommandsArray[motorId * 5] = 0;
            motorCommandsArray[motorId * 5 + 1] = 0;
            motorCommandsArray[motorId * 5 + 2] = 0;
            motorCommandsArray[motorId * 5 + 3] = 0;
            motorCommandsArray[motorId * 5 + 4] = motorCommandsShaped[motorId];
        }
    } else if (motorControlMode == HYBRID_MODE) {// in hybrid mode, motor needs q, dq, kp, kd and torque
        Eigen::Matrix<float, 5, 12> motorCommandsShaped = motorCommands;
        for (int motorId = 0; motorId < qrRobotConfig::numMotors; motorId++) {
            motorCommandsArray[motorId * 5] = motorCommandsShaped(POSITION, motorId);
            motorCommandsArray[motorId * 5 + 1] = motorCommandsShaped(KP, motorId);
            motorCommandsArray[motorId * 5 + 2] = motorCommandsShaped(VELOCITY, motorId);
            motorCommandsArray[motorId * 5 + 3] = motorCommandsShaped(KD, motorId);
            motorCommandsArray[motorId * 5 + 4] = motorCommandsShaped(TORQUE, motorId);
        }
    } 

    for(int index=0; index< motorCommandsArray.size(); index++){
        if(isnan(motorCommandsArray[index])) {
                motorCommandsArray[index] = 0.f;
        }
    }
    SendCommand(motorCommandsArray);
}

void qrRobotReal::Step(const Eigen::MatrixXf &action,
                    MotorMode motorControlMode)
{
    ReceiveObservation();
    ApplyAction(action, motorControlMode);
}

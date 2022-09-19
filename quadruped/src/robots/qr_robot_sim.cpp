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

#include "robots/qr_robot_sim.h"

qrRobotSim::qrRobotSim(ros::NodeHandle &nhIn, std::string robotName, LocomotionMode mode):
    qrRobot(robotName + "_sim", mode), nh(nhIn)
{
    // set up ros subscribers and publishers of joint angles, IMU and force feed back
    imuSub = nh.subscribe("/trunk_imu", 1, &qrRobotSim::ImuCallback, this);
    jointStateSub[0] = nh.subscribe(robotName + "_gazebo/FR_hip_controller/state", 1, &qrRobotSim::FRhipCallback, this);
    jointStateSub[1] = nh.subscribe(robotName + "_gazebo/FR_thigh_controller/state", 1, &qrRobotSim::FRthighCallback, this);
    jointStateSub[2] = nh.subscribe(robotName + "_gazebo/FR_calf_controller/state", 1, &qrRobotSim::FRcalfCallback, this);
    jointStateSub[3] = nh.subscribe(robotName + "_gazebo/FL_hip_controller/state", 1, &qrRobotSim::FLhipCallback, this);
    jointStateSub[4] = nh.subscribe(robotName + "_gazebo/FL_thigh_controller/state", 1, &qrRobotSim::FLthighCallback, this);
    jointStateSub[5] = nh.subscribe(robotName + "_gazebo/FL_calf_controller/state", 1, &qrRobotSim::FLcalfCallback, this);
    jointStateSub[6] = nh.subscribe(robotName + "_gazebo/RR_hip_controller/state", 1, &qrRobotSim::RRhipCallback, this);
    jointStateSub[7] = nh.subscribe(robotName + "_gazebo/RR_thigh_controller/state", 1, &qrRobotSim::RRthighCallback, this);
    jointStateSub[8] = nh.subscribe(robotName + "_gazebo/RR_calf_controller/state", 1, &qrRobotSim::RRcalfCallback, this);
    jointStateSub[9] = nh.subscribe(robotName + "_gazebo/RL_hip_controller/state", 1, &qrRobotSim::RLhipCallback, this);
    jointStateSub[10] = nh.subscribe(robotName + "_gazebo/RL_thigh_controller/state", 1, &qrRobotSim::RLthighCallback, this);
    jointStateSub[11] = nh.subscribe(robotName + "_gazebo/RL_calf_controller/state", 1, &qrRobotSim::RLcalfCallback, this);
    footForceSub[0] = nh.subscribe("/visual/FR_foot_contact/the_force", 1, &qrRobotSim::FRfootCallback, this);
    footForceSub[1] = nh.subscribe("/visual/FL_foot_contact/the_force", 1, &qrRobotSim::FLfootCallback, this);
    footForceSub[2] = nh.subscribe("/visual/RR_foot_contact/the_force", 1, &qrRobotSim::RRfootCallback, this);
    footForceSub[3] = nh.subscribe("/visual/RL_foot_contact/the_force", 1, &qrRobotSim::RLfootCallback, this);

    jointCmdPub[0] = nh.advertise<unitree_legged_msgs::MotorCmd>(robotName + "_gazebo/FR_hip_controller/command", 1);
    jointCmdPub[1] = nh.advertise<unitree_legged_msgs::MotorCmd>(robotName + "_gazebo/FR_thigh_controller/command", 1);
    jointCmdPub[2] = nh.advertise<unitree_legged_msgs::MotorCmd>(robotName + "_gazebo/FR_calf_controller/command", 1);
    jointCmdPub[3] = nh.advertise<unitree_legged_msgs::MotorCmd>(robotName + "_gazebo/FL_hip_controller/command", 1);
    jointCmdPub[4] = nh.advertise<unitree_legged_msgs::MotorCmd>(robotName + "_gazebo/FL_thigh_controller/command", 1);
    jointCmdPub[5] = nh.advertise<unitree_legged_msgs::MotorCmd>(robotName + "_gazebo/FL_calf_controller/command", 1);
    jointCmdPub[6] = nh.advertise<unitree_legged_msgs::MotorCmd>(robotName + "_gazebo/RR_hip_controller/command", 1);
    jointCmdPub[7] = nh.advertise<unitree_legged_msgs::MotorCmd>(robotName + "_gazebo/RR_thigh_controller/command", 1);
    jointCmdPub[8] = nh.advertise<unitree_legged_msgs::MotorCmd>(robotName + "_gazebo/RR_calf_controller/command", 1);
    jointCmdPub[9] = nh.advertise<unitree_legged_msgs::MotorCmd>(robotName + "_gazebo/RL_hip_controller/command", 1);
    jointCmdPub[10] = nh.advertise<unitree_legged_msgs::MotorCmd>(robotName + "_gazebo/RL_thigh_controller/command", 1);
    jointCmdPub[11] = nh.advertise<unitree_legged_msgs::MotorCmd>(robotName + "_gazebo/RL_calf_controller/command", 1);

    usleep(300000); // must wait 300ms, to get first state

    timeStep = 0.001;
    this->ResetTimer();
    lastResetTime = GetTimeSinceReset();

    std::cout << "-------qrRobotSim init Complete-------" << std::endl;
}

void qrRobotSim::ImuCallback(const sensor_msgs::Imu &msg)
{
    // set quaternion information
    state.imu.quaternion[0] = msg.orientation.w;
    state.imu.quaternion[1] = msg.orientation.x;
    state.imu.quaternion[2] = msg.orientation.y;
    state.imu.quaternion[3] = msg.orientation.z;

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
    state.imu.gyroscope[0] = msg.angular_velocity.x;
    state.imu.gyroscope[1] = msg.angular_velocity.y;
    state.imu.gyroscope[2] = msg.angular_velocity.z;

    // set linear acceleration
    state.imu.accelerometer[0] = msg.linear_acceleration.x;
    state.imu.accelerometer[1] = msg.linear_acceleration.y;
    state.imu.accelerometer[2] = msg.linear_acceleration.z;
}

void qrRobotSim::FRhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    state.motorState[0].q = msg.q;
    state.motorState[0].dq = msg.dq;
}

void qrRobotSim::FRthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    state.motorState[1].q = msg.q;
    state.motorState[1].dq = msg.dq;
}

void qrRobotSim::FRcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    state.motorState[2].q = msg.q;
    state.motorState[2].dq = msg.dq;
}

void qrRobotSim::FLhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    state.motorState[3].q = msg.q;
    state.motorState[3].dq = msg.dq;
}

void qrRobotSim::FLthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    state.motorState[4].q = msg.q;
    state.motorState[4].dq = msg.dq;
}

void qrRobotSim::FLcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    state.motorState[5].q = msg.q;
    state.motorState[5].dq = msg.dq;
}

void qrRobotSim::RRhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    state.motorState[6].q = msg.q;
    state.motorState[6].dq = msg.dq;
}

void qrRobotSim::RRthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    state.motorState[7].q = msg.q;
    state.motorState[7].dq = msg.dq;
}

void qrRobotSim::RRcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    state.motorState[8].q = msg.q;
    state.motorState[8].dq = msg.dq;
}

void qrRobotSim::RLhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    state.motorState[9].q = msg.q;
    state.motorState[9].dq = msg.dq;
}

void qrRobotSim::RLthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    state.motorState[10].q = msg.q;
    state.motorState[10].dq = msg.dq;
}

void qrRobotSim::RLcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    state.motorState[11].q = msg.q;
    state.motorState[11].dq = msg.dq;
}

void qrRobotSim::FRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    state.footForce[0] = msg.wrench.force.z;
}

void qrRobotSim::FLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    state.footForce[1] = msg.wrench.force.z;
}

void qrRobotSim::RRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    state.footForce[2] = msg.wrench.force.z;
}

void qrRobotSim::RLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    state.footForce[3] = msg.wrench.force.z;
}

void qrRobotSim::SendCommand(const std::array<float, 60> motorcmd)
{
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        lowCmd.motorCmd[motor_id].mode = 0x0A;
        lowCmd.motorCmd[motor_id].q = motorcmd[motor_id * 5];
        lowCmd.motorCmd[motor_id].Kp = motorcmd[motor_id * 5 + 1];
        lowCmd.motorCmd[motor_id].dq = motorcmd[motor_id * 5 + 2];
        lowCmd.motorCmd[motor_id].Kd = motorcmd[motor_id * 5 + 3];
        lowCmd.motorCmd[motor_id].tau = motorcmd[motor_id * 5 + 4];
    }
    for (int m = 0; m < 12; m++) {
        jointCmdPub[m].publish(lowCmd.motorCmd[m]);
    }
}

void qrRobotSim::ReceiveObservation()
{
    state.Update();
}

void qrRobotSim::ApplyAction(const Eigen::MatrixXf &motorCommands,
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

void qrRobotSim::Step(const Eigen::MatrixXf &action,
                    MotorMode motorControlMode)
{
    ReceiveObservation();
    ApplyAction(action, motorControlMode);
}

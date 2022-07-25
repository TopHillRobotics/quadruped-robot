/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Swing Leg Controller
* Author: Xie Ming Cheng & Zhao Yao
* Create: 2021-12-16
* Notes: xx
* Modify: add head comment and add some function comments and delete some test functions. @ xie_mingcheng 2021.11.22
*/

#include "sim/a1_sim.h"
namespace Quadruped {
    static bool firstObservation = true;

    A1Sim::A1Sim(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn, std::string configFilePath):
        Robot(configFilePath), nh(nhIn), privateNh(privateNhIn)
    {
        float standUpAbAngle, standUpHipAngle, standUpKneeAngle;
        standUpAbAngle = 0.f;
        standUpHipAngle = std::acos(config->bodyHeight / 2.f / config->upperLegLength);
        standUpKneeAngle = -2.f * standUpHipAngle;
        Eigen::Matrix<float, 3, 1> defaultStandUpAngle(standUpAbAngle, standUpHipAngle, standUpKneeAngle);
        standUpMotorAngles << defaultStandUpAngle, defaultStandUpAngle, defaultStandUpAngle, defaultStandUpAngle;

        float sitDownAbAngle, sitDownHipAngle, sitDownKneeAngle;
        sitDownAbAngle = -0.167136f;
        sitDownHipAngle = 0.934969f;
        sitDownKneeAngle = -2.54468f;
        Eigen::Matrix<float, 3, 1> defaultSitDownAngle(sitDownAbAngle, sitDownHipAngle, sitDownKneeAngle);
        sitDownMotorAngles << defaultSitDownAngle, defaultSitDownAngle, defaultSitDownAngle, defaultSitDownAngle;

        // controlParams["mode"] = robotConfig["controller_params"]["mode"].as<int>(); //types.h: enum

        imuSub = nh.subscribe("/trunk_imu", 1, &A1Sim::ImuCallback, this);
        jointStateSub[0] = nh.subscribe("a1_gazebo/FR_hip_controller/state", 1, &A1Sim::FRhipCallback, this);
        jointStateSub[1] = nh.subscribe("a1_gazebo/FR_thigh_controller/state", 1, &A1Sim::FRthighCallback, this);
        jointStateSub[2] = nh.subscribe("a1_gazebo/FR_calf_controller/state", 1, &A1Sim::FRcalfCallback, this);
        jointStateSub[3] = nh.subscribe("a1_gazebo/FL_hip_controller/state", 1, &A1Sim::FLhipCallback, this);
        jointStateSub[4] = nh.subscribe("a1_gazebo/FL_thigh_controller/state", 1, &A1Sim::FLthighCallback, this);
        jointStateSub[5] = nh.subscribe("a1_gazebo/FL_calf_controller/state", 1, &A1Sim::FLcalfCallback, this);
        jointStateSub[6] = nh.subscribe("a1_gazebo/RR_hip_controller/state", 1, &A1Sim::RRhipCallback, this);
        jointStateSub[7] = nh.subscribe("a1_gazebo/RR_thigh_controller/state", 1, &A1Sim::RRthighCallback, this);
        jointStateSub[8] = nh.subscribe("a1_gazebo/RR_calf_controller/state", 1, &A1Sim::RRcalfCallback, this);
        jointStateSub[9] = nh.subscribe("a1_gazebo/RL_hip_controller/state", 1, &A1Sim::RLhipCallback, this);
        jointStateSub[10] = nh.subscribe("a1_gazebo/RL_thigh_controller/state", 1, &A1Sim::RLthighCallback, this);
        jointStateSub[11] = nh.subscribe("a1_gazebo/RL_calf_controller/state", 1, &A1Sim::RLcalfCallback, this);
        footForceSub[0] = nh.subscribe("/visual/FR_foot_contact/the_force", 1, &A1Sim::FRfootCallback, this);
        footForceSub[1] = nh.subscribe("/visual/FL_foot_contact/the_force", 1, &A1Sim::FLfootCallback, this);
        footForceSub[2] = nh.subscribe("/visual/RR_foot_contact/the_force", 1, &A1Sim::RRfootCallback, this);
        footForceSub[3] = nh.subscribe("/visual/RL_foot_contact/the_force", 1, &A1Sim::RLfootCallback, this);

        jointCmdPub[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/FR_hip_controller/command", 1);
        jointCmdPub[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/FR_thigh_controller/command", 1);
        jointCmdPub[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/FR_calf_controller/command", 1);
        jointCmdPub[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/FL_hip_controller/command", 1);
        jointCmdPub[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/FL_thigh_controller/command", 1);
        jointCmdPub[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/FL_calf_controller/command", 1);
        jointCmdPub[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/RR_hip_controller/command", 1);
        jointCmdPub[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/RR_thigh_controller/command", 1);
        jointCmdPub[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/RR_calf_controller/command", 1);
        jointCmdPub[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/RL_hip_controller/command", 1);
        jointCmdPub[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/RL_thigh_controller/command", 1);
        jointCmdPub[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("a1_gazebo/RL_calf_controller/command", 1);

        usleep(300000); // must wait 300ms, to get first state

        timeStep = 0.001;
        this->ResetTimer();
        lastResetTime = GetTimeSinceReset();
        initComplete = true;

        std::cout << "-------A1Sim init Complete-------" << std::endl;
    }

    void A1Sim::ImuCallback(const sensor_msgs::Imu &msg)
    {
        state.imu.quaternion[0] = msg.orientation.w;
        state.imu.quaternion[1] = msg.orientation.x;
        state.imu.quaternion[2] = msg.orientation.y;
        state.imu.quaternion[3] = msg.orientation.z;

        Eigen::Matrix<float, 4, 1> quaternion = {state.imu.quaternion[0],
                                                 state.imu.quaternion[1],
                                                 state.imu.quaternion[2],
                                                 state.imu.quaternion[3]};
        Eigen::Matrix<float, 3, 1> rpy = robotics::math::quatToRPY(quaternion);

        state.imu.rpy[0] = rpy[0];
        state.imu.rpy[1] = rpy[1];
        state.imu.rpy[2] = rpy[2];

        state.imu.gyroscope[0] = msg.angular_velocity.x;
        state.imu.gyroscope[1] = msg.angular_velocity.y;
        state.imu.gyroscope[2] = msg.angular_velocity.z;

        state.imu.accelerometer[0] = msg.linear_acceleration.x;
        state.imu.accelerometer[1] = msg.linear_acceleration.y;
        state.imu.accelerometer[2] = msg.linear_acceleration.z;
    }

    void A1Sim::FRhipCallback(const unitree_legged_msgs::MotorState &msg)
    {
        state.motorState[0].q = msg.q;
        state.motorState[0].dq = msg.dq;
    }

    void A1Sim::FRthighCallback(const unitree_legged_msgs::MotorState &msg)
    {
        state.motorState[1].q = msg.q;
        state.motorState[1].dq = msg.dq;
    }

    void A1Sim::FRcalfCallback(const unitree_legged_msgs::MotorState &msg)
    {
        state.motorState[2].q = msg.q;
        state.motorState[2].dq = msg.dq;
    }

    void A1Sim::FLhipCallback(const unitree_legged_msgs::MotorState &msg)
    {
        state.motorState[3].q = msg.q;
        state.motorState[3].dq = msg.dq;
    }

    void A1Sim::FLthighCallback(const unitree_legged_msgs::MotorState &msg)
    {
        state.motorState[4].q = msg.q;
        state.motorState[4].dq = msg.dq;
    }

    void A1Sim::FLcalfCallback(const unitree_legged_msgs::MotorState &msg)
    {
        state.motorState[5].q = msg.q;
        state.motorState[5].dq = msg.dq;
    }

    void A1Sim::RRhipCallback(const unitree_legged_msgs::MotorState &msg)
    {
        state.motorState[6].q = msg.q;
        state.motorState[6].dq = msg.dq;
    }

    void A1Sim::RRthighCallback(const unitree_legged_msgs::MotorState &msg)
    {
        state.motorState[7].q = msg.q;
        state.motorState[7].dq = msg.dq;
    }

    void A1Sim::RRcalfCallback(const unitree_legged_msgs::MotorState &msg)
    {
        state.motorState[8].q = msg.q;
        state.motorState[8].dq = msg.dq;
    }

    void A1Sim::RLhipCallback(const unitree_legged_msgs::MotorState &msg)
    {
        state.motorState[9].q = msg.q;
        state.motorState[9].dq = msg.dq;
    }

    void A1Sim::RLthighCallback(const unitree_legged_msgs::MotorState &msg)
    {
        state.motorState[10].q = msg.q;
        state.motorState[10].dq = msg.dq;
    }

    void A1Sim::RLcalfCallback(const unitree_legged_msgs::MotorState &msg)
    {
        state.motorState[11].q = msg.q;
        state.motorState[11].dq = msg.dq;
    }

    void A1Sim::FRfootCallback(const geometry_msgs::WrenchStamped &msg)
    {
        state.footForce[0] = msg.wrench.force.z;
    }

    void A1Sim::FLfootCallback(const geometry_msgs::WrenchStamped &msg)
    {
        state.footForce[1] = msg.wrench.force.z;
    }

    void A1Sim::RRfootCallback(const geometry_msgs::WrenchStamped &msg)
    {
        state.footForce[2] = msg.wrench.force.z;
    }

    void A1Sim::RLfootCallback(const geometry_msgs::WrenchStamped &msg)
    {
        state.footForce[3] = msg.wrench.force.z;
    }

    void A1Sim::SendCommand(const std::array<float, 60> motorcmd)
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

    void A1Sim::ReceiveObservation()
    {
        state.Update();
    }

    void A1Sim::ApplyAction(const Eigen::MatrixXf &motorCommands,
                            MotorMode motorControlMode)
    {
        // std::cout << "apply:\n" << motorCommands << std::endl;
        std::array<float, 60> motorCommandsArray = {0};
        if (motorControlMode == POSITION_MODE) {
            Eigen::Matrix<float, 1, 12> motorCommandsShaped = motorCommands.transpose();
            for (int motorId = 0; motorId < RobotConfig::numMotors; motorId++) {
                motorCommandsArray[motorId * 5] = motorCommandsShaped[motorId];
                motorCommandsArray[motorId * 5 + 1] = config->motorKps[motorId];
                motorCommandsArray[motorId * 5 + 2] = 0;
                motorCommandsArray[motorId * 5 + 3] = config->motorKds[motorId];
                motorCommandsArray[motorId * 5 + 4] = 0;
            }
        } else if (motorControlMode == TORQUE_MODE) {
            Eigen::Matrix<float, 1, 12> motorCommandsShaped = motorCommands.transpose();
            for (int motorId = 0; motorId < RobotConfig::numMotors; motorId++) {
                motorCommandsArray[motorId * 5] = 0;
                motorCommandsArray[motorId * 5 + 1] = 0;
                motorCommandsArray[motorId * 5 + 2] = 0;
                motorCommandsArray[motorId * 5 + 3] = 0;
                motorCommandsArray[motorId * 5 + 4] = motorCommandsShaped[motorId];
            }
        } else if (motorControlMode == HYBRID_MODE) {
            Eigen::Matrix<float, 5, 12> motorCommandsShaped = motorCommands;
            for (int motorId = 0; motorId < RobotConfig::numMotors; motorId++) {
                motorCommandsArray[motorId * 5] = motorCommandsShaped(POSITION, motorId);
                motorCommandsArray[motorId * 5 + 1] = motorCommandsShaped(KP, motorId);
                motorCommandsArray[motorId * 5 + 2] =
                    motorCommandsShaped(VELOCITY, motorId);
                motorCommandsArray[motorId * 5 + 3] = motorCommandsShaped(KD, motorId);
                motorCommandsArray[motorId * 5 + 4] =
                    motorCommandsShaped(TORQUE, motorId);
            }
        } 

        for(int index=0; index< motorCommandsArray.size(); index++){
            if(isnan(motorCommandsArray[index])) {
                    motorCommandsArray[index] = 0.f;
            }
        }
        SendCommand(motorCommandsArray);
    }

    void A1Sim::ApplyAction(const std::vector<MotorCommand> &motorCommands,
                            MotorMode motorControlMode)
    {
        // std::cout << "apply:\n" << motorCommands << std::endl;
        std::array<float, 60> motorCommandsArray = {0};
        for (int motorId = 0; motorId < RobotConfig::numMotors; motorId++) {
            motorCommandsArray[motorId * 5] = motorCommands[motorId].p;
            motorCommandsArray[motorId * 5 + 1] = motorCommands[motorId].Kp;
            motorCommandsArray[motorId * 5 + 2] = motorCommands[motorId].d;
            motorCommandsArray[motorId * 5 + 3] = motorCommands[motorId].Kd;
            motorCommandsArray[motorId * 5 + 4] = motorCommands[motorId].tua;
        }
        robotInterface.SendCommand(motorCommandsArray);
    }

    void A1Sim::Step(const Eigen::MatrixXf &action,
                     MotorMode motorControlMode)
    {
        ReceiveObservation();
        ApplyAction(action, motorControlMode);
    }
} // namespace Quadruped

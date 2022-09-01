/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Swing Leg Controller
* Author: Xie Ming Cheng & Zhao Yao
* Create: 2021-12-16
* Notes: xx
* Modify: add head comment and add some function comments and delete some test functions. @ xie_mingcheng 2021.11.22
*/

#ifndef ROBOTS_A1_SIM_H
#define ROBOTS_A1_SIM_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/WrenchStamped.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include "qr_robot.h"

/**
 * @brief a1 robot class in simulation.
 */
class qrRobotA1Sim : public qrRobot {
public:
    /**
     * @brief constructor for Unitree A1 robot
     * @param nhIn: ros node
     * @param configFilePath: file path of the robot
     */
    qrRobotA1Sim(ros::NodeHandle &nhIn, std::string configFilePath);

    /**
     * @brief send command to gazebo controller
     * @param motorcmd: commands of 12 motors, each has q, dq, kp, kd, tau
     */
    void SendCommand(const std::array<float, 60> motorcmd);

    /**
     * @brief imu call back from gazebo
     * @param msg: ros message
     */
    void ImuCallback(const sensor_msgs::Imu &msg);

    /**
     * @brief front right hip motor q call back
     * @param msg: ros message
     */
    void FRhipCallback(const unitree_legged_msgs::MotorState &msg);

    /**
     * @brief front right thigh motor q call back
     * @param msg: ros message
     */
    void FRthighCallback(const unitree_legged_msgs::MotorState &msg);

    /**
     * @brief front right calf motor q call back
     * @param msg: ros message
     */
    void FRcalfCallback(const unitree_legged_msgs::MotorState &msg);

    /**
     * @brief front left hip motor q call back
     * @param msg: ros message
     */
    void FLhipCallback(const unitree_legged_msgs::MotorState &msg);

    /**
     * @brief front left thigh motor q call back
     * @param msg: ros message
     */
    void FLthighCallback(const unitree_legged_msgs::MotorState &msg);

    /**
     * @brief front left calf motor q call back
     * @param msg: ros message
     */
    void FLcalfCallback(const unitree_legged_msgs::MotorState &msg);

    /**
     * @brief rear right hip motor q call back
     * @param msg: ros message
     */
    void RRhipCallback(const unitree_legged_msgs::MotorState &msg);

    /**
     * @brief rear right thigh motor q call back
     * @param msg: ros message
     */
    void RRthighCallback(const unitree_legged_msgs::MotorState &msg);

    /**
     * @brief rear right calf motor q call back
     * @param msg: ros message
     */
    void RRcalfCallback(const unitree_legged_msgs::MotorState &msg);

    /**
     * @brief rear left hip motor q call back
     * @param msg: ros message
     */
    void RLhipCallback(const unitree_legged_msgs::MotorState &msg);

    /**
     * @brief rear left thigh motor q call back
     * @param msg: ros message
     */
    void RLthighCallback(const unitree_legged_msgs::MotorState &msg);

    /**
     * @brief rear left calf motor q call back
     * @param msg: ros message
     */
    void RLcalfCallback(const unitree_legged_msgs::MotorState &msg);

    /**
     * @brief front right foot force call back
     * @param msg: ros message
     */
    void FRfootCallback(const geometry_msgs::WrenchStamped &msg);

    /**
     * @brief front left foot force call back
     * @param msg: ros message
     */
    void FLfootCallback(const geometry_msgs::WrenchStamped &msg);

    /**
     * @brief rear right foot force call back
     * @param msg: ros message
     */
    void RRfootCallback(const geometry_msgs::WrenchStamped &msg);

    /**
     * @brief rear left foot force call back
     * @param msg: ros message
     */
    void RLfootCallback(const geometry_msgs::WrenchStamped &msg);

    /**
     * @brief override the method in qrRobot
     */
    void ReceiveObservation() override;

    /**
     * @brief override the method in qrRobot
     */
    void ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode) override;

    /**
     * @brief override the method in qrRobot
     */
    void Step(const Eigen::MatrixXf &action, MotorMode motorControlMode) override;

    /**
     * @brief ros node
     */
    ros::NodeHandle & nh;

    /**
     * @brief motor command defined by Unitree
     */
    unitree_legged_msgs::LowCmd lowCmd;

    /**
     * @brief ROS joint commander publisher
     */
    ros::Publisher jointCmdPub[12];

    /**
     * @brief ROS joint state subscriber
     */
    ros::Subscriber jointStateSub[12];

    /**
     * @brief ROS foot force subscriber
     */
    ros::Subscriber footForceSub[4];

    /**
     * @brief ROS IMU state subscriber
     */
    ros::Subscriber imuSub;
};

#endif //ROBOTS_A1_SIM_H

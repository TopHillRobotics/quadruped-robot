// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:tophill.robotics@gmail.com

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

#ifndef QR_ROBOT_A1SIM_H
#define QR_ROBOT_A1SIM_H

#include <ros/ros.h>
#include <iostream>
#include <array>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>

#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"

#include "qr_robot.h"
#include "qr_robot_state.h"

/**
 * @brief The qrRobotA1Sim class is used in gazebo simulation
 */
class qrRobotA1Sim: public qrRobot
{

public:

    /**
      * @brief qrRobotA1Sim
      * @param nhIn
      * @param configFilePath
      */
    qrRobotA1Sim(ros::NodeHandle &nhIn, std::string configFilePath);

    ~qrRobotA1Sim() override;

    void Observation() override;

    void SendCmd() override;

    void ImuCallback(const sensor_msgs::Imu &msg);

    void FRhipCallback(const unitree_legged_msgs::MotorState &msg);

    void FRthighCallback(const unitree_legged_msgs::MotorState &msg);

    void FRcalfCallback(const unitree_legged_msgs::MotorState &msg);

    void FLhipCallback(const unitree_legged_msgs::MotorState &msg);

    void FLthighCallback(const unitree_legged_msgs::MotorState &msg);

    void FLcalfCallback(const unitree_legged_msgs::MotorState &msg);

    void RRhipCallback(const unitree_legged_msgs::MotorState &msg);

    void RRthighCallback(const unitree_legged_msgs::MotorState &msg);

    void RRcalfCallback(const unitree_legged_msgs::MotorState &msg);

    void RLhipCallback(const unitree_legged_msgs::MotorState &msg);

    void RLthighCallback(const unitree_legged_msgs::MotorState &msg);

    void RLcalfCallback(const unitree_legged_msgs::MotorState &msg);

    void FRfootCallback(const geometry_msgs::WrenchStamped &msg);

    void FLfootCallback(const geometry_msgs::WrenchStamped &msg);

    void RRfootCallback(const geometry_msgs::WrenchStamped &msg);

    void RLfootCallback(const geometry_msgs::WrenchStamped &msg);

private:

    /**
     * @brief node handler of ROS
     */
    ros::NodeHandle &nh;

    /**
     * @brief state from ROS firstly save date to this buffer
     */
    qrRobotState robotStateBuffer;

    /**
     * @brief commands of unitree A1
     */
    unitree_legged_msgs::LowCmd lowCmd;

    /**
     * @brief ros publisher of joint commands
     */
    ros::Publisher jointCmdPub[12];

    /**
     * @brief ros subscriber of joint state
     */
    ros::Subscriber jointStateSub[12];

    /**
     * @brief ros subscriber of foot force
     */
    ros::Subscriber footForceSub[4];

    /**
     * @brief ros subscriber of IMU
     */
    ros::Subscriber imuSub;
};

#endif // QR_ROBOT_A1SIM_H

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

#ifndef QR_ROBOT_LITE2_SIM_H
#define QR_ROBOT_LITE2_SIM_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/WrenchStamped.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include "robots/qr_robot.h"


namespace Quadruped {

class qrRobotLite2Sim: public qrRobot {

public:

    /**
     * @brief Constructor of class RobotLite2Sim
     * @param nhIn: ROS node handle.
     * @param privateNhIn: private ROS node handle.
     * @param configFilePath: config file path.
     */
    qrRobotLite2Sim(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn, std::string configFilePath);

    ~qrRobotLite2Sim() = default;

    /**
     * @see qrRobot::ReceiveObservation
     */
    void ReceiveObservation() override;

    /**
     * @see qrRobot::ApplyAction
     */
    void ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode) override;

    /**
     * @see qrRobot::ApplyAction
     */
    void ApplyAction(const std::vector<qrMotorCommand> &motorCommands, MotorMode motorControlMode);

    /**
     * @see qrRobot::Step
     */
    void Step(const Eigen::MatrixXf &action, MotorMode motorControlMode) override;

    /**
     * @see qrRobot::BuildDynamicModel
     */
    virtual bool BuildDynamicModel();

    void SendCommand(const std::array<float, 60> motorcmd);

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

    /**
     * @brief ROS node handle.
     */
    ros::NodeHandle & nh;

    /**
     * @brief ROS private node handle.
     */
    ros::NodeHandle & privateNh;

    /**
     * @brief Unitree low command that stores motor commands.
     */
    unitree_legged_msgs::LowCmd lowCmd;

    /**
     * @brief Unitree low state that stores IMU and joint states.
     */
    unitree_legged_msgs::LowState lowState;

    /**
     * @brief 12 joint command publishers.
     */
    ros::Publisher jointCmdPub[12];

    /**
     * @brief 12 joint state subscribers.
     */
    ros::Subscriber jointStateSub[12];

    /**
     * @brief 4 force sensor subscribers.
     */
    ros::Subscriber footForceSub[4];

    /**
     * @brief Gazebo IMU subscribers.
     */
    ros::Subscriber imuSub;

};

} // Namespace Quadruped

#endif // QR_ROBOT_LITE2_SIM_H

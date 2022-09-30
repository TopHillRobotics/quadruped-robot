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

#ifndef QR_ROBOT_SIM_H
#define QR_ROBOT_SIM_H

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
class qrRobotSim : public qrRobot {
public:
    /**
     * @brief constructor for Unitree A1 robot
     * @param nhIn: ros node
     * @param robotName: name of robot to create
     */
    qrRobotSim(ros::NodeHandle &nhIn, std::string robotName, LocomotionMode mode = LocomotionMode::VELOCITY_LOCOMOTION);

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

#endif //QR_ROBOT_SIM_H

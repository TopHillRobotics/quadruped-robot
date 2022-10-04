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

#ifndef QR_ROBOT_REAL__H
#define QR_ROBOT_REAL__H

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
class qrRobotReal : public qrRobot {
public:
    /**
     * @brief constructor for Unitree A1 robot
     * @param robotName: name of robot to create
     */
    qrRobotReal( std::string robotName, LocomotionMode mode = LocomotionMode::VELOCITY_LOCOMOTION);

    /**
     * @brief send command to gazebo controller
     * @param motorcmd: commands of 12 motors, each has q, dq, kp, kd, tau
     */
    void SendCommand(const std::array<float, 60> motorcmd);

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
     * @brief update robot state
     */
    void UpdateRobotState();

    /**
     * @brief motor command defined by Unitree
     */
    unitree_legged_msgs::LowCmd lowCmd;

    /**
     * @brief robot state interface for real robot
     */
    RobotInterface robotInterface;
};

#endif //QR_ROBOT_REAL__H

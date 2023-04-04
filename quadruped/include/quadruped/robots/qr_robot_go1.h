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

#ifndef QR_ROBOT_GO1_H
#define QR_ROBOT_GO1_H

#include "robots/qr_robot.h"


namespace Quadruped {

class qrRobotGO1: public qrRobot {

public:

    /**
     * @brief Constructor method of class qrRobotGO1.
     * @param config_file_path: the path to config file.
     */
    qrRobotGO1(std::string configFilePath);

    virtual ~qrRobotGO1() = default;

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
    void ApplyAction(const std::vector<qrMotorCommand> &motorCommands,
                     MotorMode motorControlMode);

    /**
     * @see qrRobot::Step
     */
    void Step(const Eigen::MatrixXf &action, MotorMode motorControlMode) override;

    /**
     * @see qrRobot::Step
     */
    void Step(const std::vector<qrMotorCommand> &motorCommands, MotorMode motorControlMode) override;

    virtual LowState &GetLowState();

    RobotInterface robotInterface;

    std::vector<IMU> imuDatas;

};

} // namespace Quadruped

#endif  // QR_ROBOT_GO1_H

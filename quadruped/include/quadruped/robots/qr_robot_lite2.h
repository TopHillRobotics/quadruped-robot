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

#ifndef QR_LITE2_ROBOT_H
#define QR_LITE2_ROBOT_H

#include "qr_robot.h"
#include "deeprobotics_legged_sdk/parse_cmd.h"
#include "deeprobotics_legged_sdk/send_to_robot.h"


namespace Quadruped {

class qrRobotLite2: public qrRobot {

public:

    /**
     * @brief Constructor method of class qrRobotLite2.
     * @param config_file_path: the path to config file.
     */
    qrRobotLite2(std::string configFilePath);

    ~qrRobotLite2() = default;

    /**
     * @see qrRobot::ReceiveObservation
     */
    void ReceiveObservation();

    /**
     * @see qrRobot::ApplyAction
     */
    void ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode);

    /**
     * @see qrRobot::ApplyAction
     */
    void ApplyAction(const std::vector<qrMotorCommand> &motorCommands, MotorMode motorControlMode);

    /**
     * @see qrRobot::Step
     */
    void Step(const Eigen::MatrixXf &action, MotorMode motorControlMode);
    
    /**
     * @see qrRobot::BuildDynamicModel
     */
    virtual bool BuildDynamicModel() override;

    /**
     * @brief State receiver for lite2.
     */
    ParseCMD lite2Receiver;

    /**
     * @brief Command sender for lite2.
     */
    SendToRobot lite2Sender;

    /**
     * @brief Lite2 robot state.
     */
    RobotState lowState_lite2;

};

} // namespace Quadruped

#endif // QR_LITE2_ROBOT_H

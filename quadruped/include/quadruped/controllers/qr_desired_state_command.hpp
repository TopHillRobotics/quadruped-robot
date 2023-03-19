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

#ifndef QR_DESIRED_STATE_COMMAND_H
#define QR_DESIRED_STATE_COMMAND_H

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "config/qr_enum_types.h"
#include "config/qr_config.h"
#include "utils/qr_cpptypes.h"
#include "robots/qr_robot.h"


namespace Quadruped {

class qrDesiredStateCommand {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Contructor of class qrDesiredStateCommand.
     * @param nhIn: ROS node handle. This is used to subscribe the joy node.
     * @param robotIn: robot pointer. This is mainly used to get the robot height and then set the desired height.
     */
    qrDesiredStateCommand(ros::NodeHandle &nhIn, qrRobot* robotIn);

    ~qrDesiredStateCommand() = default;

    /**
     * @brief Update the desired state according to the joy input
     */
    void Update();

    /**
     * @brief print command from joy
     */
    void PrintRawInfo();

    /**
     * @brief print desired state generated from joy command
     */
    void PrintStateCommandInfo();

    /**
     * @brief the callback function called when the joy command received
     * @param joy_msg: joy command message from ROS joy_node
     */
    void JoyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg);

    /**
     * @brief Getter method of member joyCtrlState.
     */
    inline RC_MODE getJoyCtrlState() const {
      return joyCtrlState;
    }

    /**
     * @brief Getter method of member joyCtrlStateChangeRequest.
     */
    inline bool getJoyCtrlStateChangeRequest() const {
      return joyCtrlStateChangeRequest;
    }

    /**
     * @brief Setter method of member joyCtrlStateChangeRequest.
     * @param request: new value for joyCtrlStateChangeRequest.
     */
    inline void setJoyCtrlStateChangeRequest(bool request) {
      joyCtrlStateChangeRequest = request;
    }

    /**
     * @brief Desired linear velocity expressed in body frame.
     */
    Vec3<float> vDesInBodyFrame;

    /**
     * @brief Desired angular velocity expressed in body frame.
     */
    Vec3<float> wDesInBodyFrame;

    /**
     * @brief Stores desired position, roll pitch yaw, linear velocity and roll pitch yaw rate.
     */
    Vec12<float> stateDes;

    /**
     * @brief Stores current position, roll pitch yaw, linear velocity and roll pitch yaw rate.
     */
    Vec12<float> stateCur;

    /**
     * @brief Stores last desired position, roll pitch yaw, linear velocity and roll pitch yaw rate.
     */
    Vec12<float> preStateDes;

    /**
     * @brief Stores the linear and angular acceleration, and 12 joint accelerations.
     */
    Vec18<float> ddqDes;

    /**
     * @brief Desired joint angles.
     * This memeber is now only used in walk locomotion.
     * May be abandoned in the future
     */
    Eigen::Matrix<float, 3, 4> legJointq;

    /**
     * @brief Desired joint angle velocities.
     * This member is not used now.
     * May be abandoned in the future
     */
    Eigen::Matrix<float, 3, 4> legJointdq;

    /**
     * @brief Desired footstep target position, expressed in world frame.
     */
    Eigen::Matrix<float, 3, 4> footTargetPositionsInWorldFrame;

private:

    /**
     * @brief Timestep for one control loop.
     * @todo sync this value to the timestep in Robot class
     * @see Quadruped::Robot
     */
    float dt = 0.002f;

    /**
     * @brief ROS nodehandle to subscribe "/joy".
     * @todo will use driver to get the state of the joy instead of ROS topic
     */
    ros::NodeHandle &nh;

    /**
     * @brief ROS subscriber for the ROS joy_node.
     */
    ros::Subscriber gamepadCommandSub;

    /**
     * @brief Topic name of the ROS joy_node.
     */
    std::string topicName = "/joy";

    /**
     * @brief Current joy state.
     */
    RC_MODE joyCtrlState;

    /**
     * @brief Last joy state.
     */
    RC_MODE prevJoyCtrlState;

    /**
     * @brief A variable indicates whether the quadruped is moving now.
     */
    int movementMode;

    /**
     * @brief A variavle indicating whether the quadruped need to change state.
     * If the variable is TRUE, it will be used to update the FSM of the quadruped.
     */
    bool joyCtrlStateChangeRequest;

    /**
     * @brief A variable indicating whether to use ROS message control or joy control.
     * @attention currently, we only use joy control in the quadruped library.
     * If you want to use other control method, please implement by yourself.
     */
    bool joyCtrlOnRequest;

    /**
     * @brief Contrary to joyCtrlOnRequest.
     */
    bool rosCmdRequest;

    /**
     * @brief A variable indicating whether the joy control stops.
     * When this boolean is true, the joy commands will set to zero,quadruped will stop locomotion.
     */
    bool joyCmdExit;

    /**
     * @brief A variable indicating whether the body is up or down.
     * If bodyUp equals -1, the quadruped is sitting down.
     * If bodyUp equals 0, the quadruped stops at bodyHeight.
     * If bodyUp equals 1, the quadruped is standing up.
     */
    int bodyUp;

    /**
     * @brief a variable indicating whether quadruped needs to change the gait.
     * Currently this variable has no actual use.
     * @todo will remove this member in the future.
     */
    bool gaitSwitch = false;

    /**
     * @brief desired body height.
     * Now it is a constant.
     */
    float joycmdBodyHeight;

    /**
     * @brief Linear velocity on Z Axis from joy control.
     */
    float joyCmdVz;

    /**
     * @brief Linear velocity on X Axis from joy control.
     */
    float joyCmdVx;

    /**
     * @brief Linear velocity on Y Axis from joy control.
     */
    float joyCmdVy;

    /**
     * @brief Yaw velocity from joy control.
     */
    float joyCmdYawRate;

    /**
     * @brief Roll velocity from joy control.
     */
    float joyCmdRollRate;

    /**
     * @brief Pitch velocity from joy control
     */
    float joyCmdPitchRate;

    /**
     * @brief This factor is used as a linear factor between current velocity and joy velocity.
     */
    const float filterFactor = 0.02f;

    /**
     * @brief The filtered linear velocity from the joy command.
     */
    Vec3<float> filteredVel;

    /**
     * @brief The filtered angular velocity from the joy command.
     * @attention This is actually the change rate of roll pitch yaw.
     */
    Vec3<float> filteredOmega;

    /**
     * @brief Choose how often to print info, every N iterations.
     */
    int printNum = 5;

    /**
     * @brief Track the number of iterations since last info print.
     */
    int printIter = 0;

    /**
     * @brief Max roll value for the desired state.
     */
    const float MAX_ROLL = 0.4f;

    /**
     * @brief Min roll value for the desired state.
     */
    const float MIN_ROLL = -0.4f;

    /**
     * @brief Max pitch value for the desired state.
     */
    const float MAX_PITCH = 0.5f;

    /**
     * @brief Min pitch value for the desired state.
     */
    const float MIN_PITCH = -0.5f;

    /**
     * @brief Max linear velocity on X Axis for the desired state.
     */
    const float MAX_VELX = 0.2f;

    /**
     * @brief Min linear velocity on X Axis for the desired state.
     */
    const float MIN_VELX = -0.15f;

    /**
     * @brief Max linear velocity on Y Axis for the desired state.
     */
    const float MAX_VELY = 0.1f;

    /**
     * @brief Min linear velocity on Y Axis for the desired state.
     */
    const float MIN_VELY = -0.1f;

    /**
     * @brief Max yaw velocity for the desired state.
     */
    const float MAX_YAWRATE = 0.2f;

    /**
     * @brief Min yaw velocity for the desired state.
     */
    const float MIN_YAWRATE = -0.2f;

    /**
     * @brief Max roll velocity for the desired state.
     */
    const float MAX_ROLLRATE = 0.2f;

    /**
     * @brief Min roll velocity for the desired state.
     */
    const float MIN_ROLLRATE = -0.2f;

    /**
     * @brief Max pitch velocity for the desired state.
     */
    const float MAX_PITCHRATE = 0.2f;

    /**
     * @brief Min pitch velocity for the desired state.
     */
    const float MIN_PITCHRATE = -0.2f;

    /**
     * @brief Max body height for the desired state.
     * This member is currently not used.
     */
    const float BODY_HEIGHT_MAX = 0.5f;

    /**
     * @brief Min body height for the desired state.
     * This member is currently not used
     */
    const float BODY_HEIGHT_MIN = 0.15f;

};

} // Namespace Quadruped

#endif // QR_DESIRED_STATE_COMMAND_H

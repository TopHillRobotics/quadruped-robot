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

#ifndef QR_ROBOT_H
#define QR_ROBOT_H

#include <iostream>
#include <string>
#include <cmath>
#include <unordered_map>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "config/qr_config.h"
#include "config/qr_enum_types.h"
#include "robots/qr_timer.h"
#include "robots/qr_motor.h"
#include "utils/qr_se3.h"
#include "utils/qr_tools.h"
#include "utils/qr_print.hpp"
#include "estimators/qr_moving_window_filter.hpp"
#include "controllers/qr_state_dataflow.h"
#include "dynamics/floating_base_model.hpp"

#include "unitree_legged_sdk/unitree_interface.h"


namespace Quadruped {

class qrRobot {

public:

    /**
     * @brief Constructor of class qrRobot.
     */
    qrRobot();

    /**
     * @brief Constructor of class qrRobot.
     * @param robot_name: type of the robot.
     * @param config_file_path: path to robot config file.
     */
    qrRobot(std::string robot_name, std::string config_file_path): robotName(robot_name), configFilePath(config_file_path), timer(useRosTime) {
    };

    virtual ~qrRobot() = default;

    /**
     * @brief Reset the robot state.
     */
    virtual void Reset();

    /**
     * @brief Update observation in each loop and get the robot states.
     * This method is user-defined according to different robots.
     */
    virtual void ReceiveObservation() = 0;

    /**
     * @brief Set and send commands to motor.
     * @param motor_commands: matrix of commands to execute by the motors.
     * @param motor_control_mode: control mode such as BRAKE.
     */
    virtual void ApplyAction(const Eigen::MatrixXf &motor_commands, MotorMode motor_control_mode) = 0;

    /**
     * @brief Set and send commands to motor.
     * @param motor_commands: vector of commands to execute by the motors.
     * @param motor_control_mode: control mode such as BRAKE.
     */
    virtual void ApplyAction(const std::vector<qrMotorCommand> &motor_commands, MotorMode motor_control_mode) {
    };

    /**
     * @brief Do Observation once and ApplyAction.
     * @param action: commands to execute.
     * @param motor_control_mode: control mode.
     */
    virtual void Step(const Eigen::MatrixXf &action, MotorMode motor_control_mode) = 0;

    /**
     * @brief Do Observation once and ApplyAction.
     * @param motor_commands: commands to execute.
     * @param motor_control_mode: control mode.
     */
    virtual void Step(const std::vector<qrMotorCommand> &motor_commands, MotorMode motor_control_mode) {
      std::cout << "a opppp" << std::endl;
    };

    /**
     * @brief Update some kinematics data such as velocitiy and some rotation matrices.
     */
    void UpdateDataFlow();

    /**
     * @brief Compute moment by dynamic formulation.
     */
    void ComputeMoment();

    /**
     * @brief convert foot position in hip frame to joint angles.
     * @param foot_position: position of foot in hip frame.
     * @param hip_sign: hip position.
     * @return joint angles
     */
    Vec3<float> FootPositionInHipFrameToJointAngle(Vec3<float> &foot_position, int hip_sign = 1);

    /**
     * @brief Get foot position in hip frame.
     * @param angles: angles of one leg.
     * @param hip_sign: hip position.
     * @return foot position in hip frame
     */
    Vec3<float> FootPositionInHipFrame(Vec3<float> &angles, int hip_sign = 1);

    /**
     * @brief Calculate Jacobian of a leg with leg ID and angles.
     * @param leg_angles: 3 motor angles on the leg.
     * @param legId: which leg to calculate.
     * @return the analytical jacobian.
     */
    Mat3<float> AnalyticalLegJacobian(Vec3<float> &leg_angles, int leg_id);

    /**
     * @brief Calculate foot position in base frame of robot.
     * @param foot_angles: joint angles
     * @return foot position in base frame
     */
    Mat34<float> FootPositionsInBaseFrame(Eigen::Matrix<float, 12, 1> foot_angles);

    /**
     * @brief Calculate foot velocity in base frame of robot.
     * @return foot velocities in base frame
     */
    Mat34<float> ComputeFootVelocitiesInBaseFrame();

    /**
     * @brief Convert foot position to joint angles.
     * @param leg_id: which leg to calculate
     * @param foot_local_position: the position of the foothold
     * @param joint_idx: index of foot to compute
     * @param joint_angles: angles to compute
     * @return position(x, y, z)
     */
    void ComputeMotorAnglesFromFootLocalPosition(int leg_id,
                                                 Vec3<float> foot_local_position,
                                                 Vec3<int> &joint_idx,
                                                 Vec3<float> &joint_angles);

    /**
     * @brief Compute motor velocity from foot local velocity.
     * @param leg_id: id of leg to compute.
     * @param leg_angles: angles of 3 motors of leg to compute.
     * @param foot_local_velocity: foot end velocity
     * @return motor velocity.
     */
    Vec3<float> ComputeMotorVelocityFromFootLocalVelocity(int leg_id,
                                                          Vec3<float> leg_angles,
                                                          Vec3<float> foot_local_velocity);

    /**
     * @brief caculate foot position in world frame of robot.
     * @param use_input: if use the given base position and base orientation.
     * @param base_position: the given base position of robot.
     * @param base_orientation: the given base orientation of robot.
     * @return foot position in world frame.
     */
    Mat34<float> GetFootPositionsInWorldFrame(bool use_input = false,
                                                            Vec3<float> base_position={0.f, 0.f, 0.f},
                                                            Quat<float> base_orientation={1.f, 0.f, 0.f, 0.f});

    /**
     * @brief get current jacobian of leg legId.
     * @param leg_id: which leg to calculate.
     * @return current Jacobian.
     */
    Eigen::Matrix<float, 3, 3> ComputeJacobian(int leg_id);

    /**
     * @brief Convert contact force of one leg to joint torque.
     * @param leg_id: which leg to convert.
     * @param contact_force: contact force of the leg.
     * @return joint torques, totally 3 joints.
     */
    std::map<int, float> MapContactForceToJointTorques(int leg_id, Vec3<float> contact_force);

    /**
     * @brief Convert vector to signed vectors according to different legs.
     * @param v: vector on leg.
     * @param leg_id: which leg to convert.
     * @return signed vector.
     */
    Vec3<float> WithLegSigns(const Vec3<float>& v, int leg_id);

    /**
     * @brief Reset the timer.
     */
    void ResetTimer() {
        timer.ResetStartTime();
    };

    /**
     * @brief Build the dynamic model of robot.
     * @return true if work has done.
     */
    virtual bool BuildDynamicModel() {
        return false;
    };

    /**
     * @brief Get foot position in base frame of robot.
     * @return foot position in base frame.
     */
    Mat34<float> GetFootPositionsInBaseFrame(){
        return stateDataFlow.footPositionsInBaseFrame;
    }

    /**
     * @brief Get locomotion mode of robot.
     */
    std::string GetControlMode() {
        return modeMap[controlParams["mode"]];
    };

    /**
     * @brief Getter method of member baseVelocityInBaseFrame.
     */
    inline Vec3<float> GetBaseVelocityInBaseFrame() {
        return baseVelocityInBaseFrame;
    };

    /**
     * @brief Getter method of member tick.
     */
    inline uint32_t GetTick() {
        return tick;
    };

    /**
     * @brief Getter method of member motorAngles.
     */
    inline Eigen::Matrix<float, 12, 1> GetMotorAngles() {
        return motorAngles;
    };

    /**
     * @brief Getter method of member motorVelocities.
     */
    inline Eigen::Matrix<float, 12, 1> GetMotorVelocities() const {
        return motorVelocities;
    };

    /**
     * @brief Getter method of member basePosition.
     */
    inline Vec3<float> GetBasePosition() const {
        return basePosition;
    };

    /**
     * @brief Getter method of member baseOrientation.
     */
    inline Eigen::Matrix<float, 4, 1> GetBaseOrientation() const {
        return baseOrientation;
    };

    /**
     * @brief Getter method of member defaultHipPosition.
     */
    inline Mat34<float> GetDefaultHipPosition() const {
        return defaultHipPosition;
    };

    /**
     * @brief Getter method of member baseRollPitchYaw.
     */
    inline Vec3<float> GetBaseRollPitchYaw() const {
        return baseRollPitchYaw;
    };

    /**
     * @brief Getter method of member baseRollPitchYawRate.
     */
    inline Vec3<float> GetBaseRollPitchYawRate() const {
        return baseRollPitchYawRate;
    };

    /**
     * @brief Getter method of member footForce.
     */
    inline Eigen::Matrix<float, 4, 1> GetFootForce() const {
        return footForce;
    };

    /**
     * @brief Getter method of member footContact.
     */
    inline Eigen::Matrix<bool, 4, 1> GetFootContact() const {
        return footContact;
    };

    /**
     * @brief Getter method of member motorKps.
     */
    inline Eigen::Matrix<float, 12, 1> GetMotorKps() const {
        return motorKps;
    };

    /**
     * @brief Getter method of member motorKds.
     */
    inline Eigen::Matrix<float, 12, 1> GetMotorKdp() const {
        return motorKds;
    };

    /**
     * @brief Getter method of member timeStep.
     */
    inline float GetTimeStep() {
        return timeStep;
    };

    /**
     * @brief Getter method of member timer.
     */
    inline qrTimerInterface &GetTimer() {
        return timer;
    };

    /**
     * @brief get time passed since robot reset
     * @return time after reset
     */
    float GetTimeSinceReset() {
        return timer.GetTimeSinceReset();
    };

    /**
     * @brief File path to robot config file。
     */
    std::string configFilePath;

    /**
     * @brief YAML node of the config file.
     */
    YAML::Node robotConfig;

    /**
     * @brief Type of the robot to create.
     */
    std::string robotName;

    /**
     * @brief Total mass of the robot.
     */
    float totalMass;

    /**
     * @brief Body mass of the robot.
     */
    float bodyMass;

    /**
     * @brief Total inertia matrix of robot.
     */
    Mat3<float> totalInertia;

    /**
     * @brief Body inertia matrix of robot
     */
    Mat3<float> bodyInertia;

    /**
     * @brief 12 link inertia matrices on four legs.
     */
    std::vector<Mat3<float>> linkInertias;

    /**
     * @brief Mass of 12 link on four legs.
     */
    std::vector<float> linkMasses;

    /**
     * @brief Length of 12 link on four legs.
     */
    std::vector<float> linkLength;

    /**
     * @brief Links' center of mass position relative to body geometry frame.
     */
    std::vector<std::vector<float>> linksComPos;

    /**
     * @brief The height of the main body.
     */
    float bodyHeight;

    /**
     * @brief Hip joint position relative to body frame.
     */
    Vec3<float> abadLocation;

    /**
     * @brief The first link length of the leg.
     */
    float hipLength;

    /**
     * @brief The second link length of the leg.
     */
    float upperLegLength;

    /**
     * @brief The third link length of the leg.
     */
    float lowerLegLength;

    /**
     * @brief Deviation of the foothold from the foot joint
     */
    float footHoldOffset = 0.1f;

    /**
     * @brief CoM offset to center of geometry.
     */
    Vec3<float> comOffset;

    /**
     * @brief Hip offset to center of geomotry.
     * hip offset to center of mass
     */
    Mat34<float> hipOffset;

    /**
     * @brief Default hip position.
     */
    Mat34<float> defaultHipPosition;

    /**
     * @brief Position stiffness (unit: N.m/rad).
     */
    Eigen::Matrix<float, 12, 1> motorKps;

    /**
     * @brief Velocity stiffness (unit: N.m/(rad/s) ).
     */
    Eigen::Matrix<float, 12, 1> motorKds;

    /**
     * @brief The sign of 12 joint angles.
     * Different robots may have different coordinate definition.
     */
    Eigen::Matrix<float, 12, 1> jointDirection = Eigen::Matrix<float, 12, 1>::Ones();

    /**
     * @brief The real joint angles offset to motor angles.
     * Different robots may have different joint definition.
     */
    Eigen::Matrix<float, 12, 1> jointOffset = Eigen::Matrix<float, 12, 1>::Zero();

    /**
     * @brief Default motor angle when robot stands.
     */
    Eigen::Matrix<float, 12, 1> standUpMotorAngles;

    /**
     * @brief Motor angles after robot sits down.
     */
    Eigen::Matrix<float, 12, 1> sitDownMotorAngles;

    /**
     * @brief Locomotion mode of the robot,
     * including vel, position, walk, advanced trot
     */
    std::map<std::string, int> controlParams;

    /**
     * @brief Whether to use ROS time tools.
     */
    bool useRosTime = true;

    /**
     * @brief Timer that store time since robot starts.
     */
    qrTimerInterface timer;

    /**
     * @brief Control frequence of the robot.
     */
    float timeStep;

    /**
     * @brief Stores last time that resets robot.
     */
    float lastResetTime;

    /**
     * @brief If the robot finished initialization completly.
     */
    bool initComplete = false;

    /**
     * @brief Whether robot is simulation.
     */
    bool isSim;

    /**
     * @brief Moving window filter that stores intermediate
     * variable and results of acceleration.
     */
    qrMovingWindowFilter<float,3> accFilter;

    /**
     * @brief Moving window filter that stores intermediate
     * variable and results of gyroscope.
     */
    qrMovingWindowFilter<float,3> gyroFilter;

    /**
     * @brief Moving window filter that stores intermediate
     * variable and results of row-pitch-yaw.
     */
    qrMovingWindowFilter<float,3> rpyFilter;

    /**
     * @brief Moving window filter that stores intermediate
     * variable and results of quaternion.
     */
    qrMovingWindowFilter<float,4> quatFilter;

    /**
     * @brief Moving window filter that stores intermediate
     * variable and results of motor velocity.
     */
    qrMovingWindowFilter<float,12> motorVFilter;

    /**
     * @brief Robot base position in world frame.
     */
    Vec3<float> basePosition = {0.f, 0.f, A1_BODY_HIGHT};

    /**
     * @brief Height of main body in world frame.
     */
    float absoluteHight = 0;

    /**
     * @brief Robot base orientation in world frame.
     */
    Eigen::Matrix<float, 4, 1> baseOrientation;

    /**
     * @brief Yaw calibrated robot rpy in world frame.
     */
    Vec3<float> baseRollPitchYaw;

    /**
     * @brief Robot rpy rate in base frame.
     */
    Vec3<float> baseRollPitchYawRate;

    /**
     * @brief Robot velocity in base frame.
     */
    Vec3<float> baseVelocityInBaseFrame;

    /**
     * @brief Robot acceleration in base frame.
     */
    Vec3<float> baseAccInBaseFrame;

    /**
     * @brief Current angle (unit: radian).
     */
    Eigen::Matrix<float, 12, 1> motorAngles;

    /**
     * @brief Current velocity (unit: radian/second).
     */
    Eigen::Matrix<float, 12, 1> motorVelocities;

    /**
     * @brief Current acceleration (unit: radian/second^2).
     */
    Eigen::Matrix<float, 12, 1> motorddq;

    /**
     * @brief Torque (unit: N.m).
     */
    Eigen::Matrix<float, 12, 1> motortorque;

    /**
     * @brief The force on 4 foot (unit: N).
     */
    Eigen::Matrix<float, 4, 1> footForce;

    /**
     * @brief Contact state of 4 feet.
     */
    Eigen::Matrix<bool, 4, 1> footContact;

    /**
     * @brief Stores some kinematics datas of the locomotion.
     */
    qrStateDataFlow stateDataFlow;

    /**
     * @brief Dynamics model for whole body control.
     */
    FloatingBaseModel<float> model;

    /**
     * @brief Low-level state including imu data encoder data.
     */
    LowState lowState;

    /**
     * @brief Real robot state used for compution time for each loop.
     */
    uint32_t tick = 0;

    /**
     * @brief Yaw offset when starting the robot.
     */
    float yawOffset = 0.f;

    /**
     * @brief Whether the robot has stopped.
     */
    bool stop = false;

    /**
     * @brief Robot base position in world frame.
     */
    Vec3<float> gazeboBasePosition = {0.f, 0.f, A1_BODY_HIGHT}; //robot base position in world frame

    /**
     * @brief Robot base orientation in world frame.
     */
    Eigen::Matrix<float, 4, 1> gazeboBaseOrientation = {1.f,0.f,0.f,0.f}; //robot base orientation in world frame

    /**
     * @brief Robot base velocity in base frame.
     */
    Vec3<float> gazeboBaseVInBaseFrame;

    /**
     * @brief Robot foot positons in world frame.
     */
    Mat34<float> gazeboFootPositionInWorldFrame;

    /**
     * @brief Stores the map of number to locomotion mode.
     */
    std::unordered_map<int, std::string> modeMap = {{0, "velocity"},
                                                    {1, "position"},
                                                    {2, "walk"},
                                                    {3, "advanced_trot"}};

    /**
     * @brief Current finite state machine mode.
     */
    int fsmMode = 4;

};

} // Namespace Quadruped

#endif // QR_ROBOT_H

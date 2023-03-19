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

#include "robots/qr_robot_aliengo_sim.h"


namespace Quadruped {

qrRobotAliengoSim::qrRobotAliengoSim(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn, std::string configFilePath):
    nh(nhIn),
    privateNh(privateNhIn)
{
    baseOrientation << 1.f, 0.f, 0.f, 0.f;
    baseRollPitchYaw << 0.f, 0.f, 0.f;
    baseRollPitchYawRate << 0.f, 0.f, 0.f;
    motorVelocities = Eigen::Matrix<float, 12, 1>::Zero();
    footForce << 0.f, 0.f, 0.f, 0.f;
    footContact << 1, 1, 1, 1;
    
    this->configFilePath = configFilePath;
    robotConfig = YAML::LoadFile(configFilePath);
    robotName = robotConfig["name"].as<std::string>();
    isSim = robotConfig["is_sim"].as<bool>();
    bodyMass = robotConfig["robot_params"]["body_mass"].as<float>();
    std::vector<float> bodyInertiaList = robotConfig["robot_params"]["body_inertia"].as<std::vector<float >>();
    bodyInertia = Eigen::MatrixXf::Map(&bodyInertiaList[0], 3, 3);
    bodyHeight = robotConfig["robot_params"]["body_height"].as<float>();

    hipLength = robotConfig["robot_params"]["hip_l"].as<float>();
    upperLegLength = robotConfig["robot_params"]["upper_l"].as<float>();
    lowerLegLength = robotConfig["robot_params"]["lower_l"].as<float>();

    controlParams["mode"] = robotConfig["controller_params"]["mode"].as<int>(); // types.h: enum

    std::vector<float> comOffsetList = robotConfig["robot_params"][modeMap[controlParams["mode"]]]["com_offset"].as<std::vector<float >>();
    // std::vector<float> comOffsetList = robotConfig["robot_params"]["com_offset"].as<std::vector<float >>();
    comOffset = -Eigen::MatrixXf::Map(&comOffsetList[0], 3, 1);

    std::vector<std::vector<float >>
        hipOffsetList =
        robotConfig["robot_params"]["hip_offset"].as<std::vector<std::vector<float>>>();
    Eigen::Matrix<float, 3, 1> hipOffsetFR = Eigen::MatrixXf::Map(&hipOffsetList[0][0], 3, 1) + comOffset;
    Eigen::Matrix<float, 3, 1> hipOffsetFL = Eigen::MatrixXf::Map(&hipOffsetList[1][0], 3, 1) + comOffset;
    Eigen::Matrix<float, 3, 1> hipOffsetRL = Eigen::MatrixXf::Map(&hipOffsetList[2][0], 3, 1) + comOffset;
    Eigen::Matrix<float, 3, 1> hipOffsetRR = Eigen::MatrixXf::Map(&hipOffsetList[3][0], 3, 1) + comOffset;
    hipOffset << hipOffsetFR, hipOffsetFL, hipOffsetRL, hipOffsetRR;

    std::vector<std::vector<float>> defaultHipPositionList =
        robotConfig["robot_params"]["default_hip_positions"].as<std::vector<std::vector<float>>>();
    Eigen::Matrix<float, 3, 1> defaultHipPositionFR = Eigen::MatrixXf::Map(&defaultHipPositionList[0][0], 3, 1);
    Eigen::Matrix<float, 3, 1> defaultHipPositionFL = Eigen::MatrixXf::Map(&defaultHipPositionList[1][0], 3, 1);
    Eigen::Matrix<float, 3, 1> defaultHipPositionRL = Eigen::MatrixXf::Map(&defaultHipPositionList[2][0], 3, 1);
    Eigen::Matrix<float, 3, 1> defaultHipPositionRR = Eigen::MatrixXf::Map(&defaultHipPositionList[3][0], 3, 1);
    defaultHipPosition << defaultHipPositionFR, defaultHipPositionFL, defaultHipPositionRL, defaultHipPositionRR;

    float abadKp, abadKd, hipKp, hipKd, kneeKp, kneeKd;
    abadKp = robotConfig["motor_params"]["abad_p"].as<float>();
    abadKd = robotConfig["motor_params"]["abad_d"].as<float>();
    hipKp = robotConfig["motor_params"]["hip_p"].as<float>();
    hipKd = robotConfig["motor_params"]["hip_d"].as<float>();
    kneeKp = robotConfig["motor_params"]["knee_p"].as<float>();
    kneeKd = robotConfig["motor_params"]["knee_d"].as<float>();
    Eigen::Matrix<float, 3, 1> kps(abadKp, hipKp, kneeKp);
    Eigen::Matrix<float, 3, 1> kds(abadKd, hipKd, kneeKd);
    motorKps << kps, kps, kps, kps;
    motorKds << kds, kds, kds, kds;

    std::vector<float>
        jointDirectionList = robotConfig["motor_params"]["joint_directions"].as<std::vector<float >>();
    std::vector<float>
        jointOffsetList = robotConfig["motor_params"]["joint_offsets"].as<std::vector<float >>();
    jointDirection = Eigen::MatrixXf::Map(&jointDirectionList[0], 12, 1);
    jointOffset = Eigen::MatrixXf::Map(&jointOffsetList[0], 12, 1);

    float standUpAbAngle, standUpHipAngle, standUpKneeAngle;
    standUpAbAngle = 0.f;
    standUpHipAngle = std::acos(bodyHeight / 2.f / upperLegLength);
    standUpKneeAngle = -2.f * standUpHipAngle;
    Eigen::Matrix<float, 3, 1> defaultStandUpAngle(standUpAbAngle, standUpHipAngle, standUpKneeAngle);
    standUpMotorAngles << defaultStandUpAngle, defaultStandUpAngle, defaultStandUpAngle, defaultStandUpAngle;

    float sitDownAbAngle, sitDownHipAngle, sitDownKneeAngle;
    sitDownAbAngle = robotConfig["robot_params"]["default_sitdown_angle"]["ab"].as<float>();
    sitDownHipAngle = robotConfig["robot_params"]["default_sitdown_angle"]["hip"].as<float>();
    sitDownKneeAngle = robotConfig["robot_params"]["default_sitdown_angle"]["knee"].as<float>();
    Eigen::Matrix<float, 3, 1> defaultSitDownAngle(sitDownAbAngle, sitDownHipAngle, sitDownKneeAngle);
    sitDownMotorAngles << defaultSitDownAngle, defaultSitDownAngle, defaultSitDownAngle, defaultSitDownAngle;

    // controlParams["mode"] = robotConfig["controller_params"]["mode"].as<int>(); //types.h: enum

    imuSub = nh.subscribe("/trunk_imu", 1, &qrRobotAliengoSim::ImuCallback, this);
    jointStateSub[0] = nh.subscribe("aliengo_gazebo/FR_hip_controller/state", 1, &qrRobotAliengoSim::FRhipCallback, this);
    jointStateSub[1] = nh.subscribe("aliengo_gazebo/FR_thigh_controller/state", 1, &qrRobotAliengoSim::FRthighCallback, this);
    jointStateSub[2] = nh.subscribe("aliengo_gazebo/FR_calf_controller/state", 1, &qrRobotAliengoSim::FRcalfCallback, this);
    jointStateSub[3] = nh.subscribe("aliengo_gazebo/FL_hip_controller/state", 1, &qrRobotAliengoSim::FLhipCallback, this);
    jointStateSub[4] = nh.subscribe("aliengo_gazebo/FL_thigh_controller/state", 1, &qrRobotAliengoSim::FLthighCallback, this);
    jointStateSub[5] = nh.subscribe("aliengo_gazebo/FL_calf_controller/state", 1, &qrRobotAliengoSim::FLcalfCallback, this);
    jointStateSub[6] = nh.subscribe("aliengo_gazebo/RR_hip_controller/state", 1, &qrRobotAliengoSim::RRhipCallback, this);
    jointStateSub[7] = nh.subscribe("aliengo_gazebo/RR_thigh_controller/state", 1, &qrRobotAliengoSim::RRthighCallback, this);
    jointStateSub[8] = nh.subscribe("aliengo_gazebo/RR_calf_controller/state", 1, &qrRobotAliengoSim::RRcalfCallback, this);
    jointStateSub[9] = nh.subscribe("aliengo_gazebo/RL_hip_controller/state", 1, &qrRobotAliengoSim::RLhipCallback, this);
    jointStateSub[10] = nh.subscribe("aliengo_gazebo/RL_thigh_controller/state", 1, &qrRobotAliengoSim::RLthighCallback, this);
    jointStateSub[11] = nh.subscribe("aliengo_gazebo/RL_calf_controller/state", 1, &qrRobotAliengoSim::RLcalfCallback, this);
    footForceSub[0] = nh.subscribe("/visual/FR_foot_contact/the_force", 1, &qrRobotAliengoSim::FRfootCallback, this);
    footForceSub[1] = nh.subscribe("/visual/FL_foot_contact/the_force", 1, &qrRobotAliengoSim::FLfootCallback, this);
    footForceSub[2] = nh.subscribe("/visual/RR_foot_contact/the_force", 1, &qrRobotAliengoSim::RRfootCallback, this);
    footForceSub[3] = nh.subscribe("/visual/RL_foot_contact/the_force", 1, &qrRobotAliengoSim::RLfootCallback, this);

    jointCmdPub[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("aliengo_gazebo/FR_hip_controller/command", 1);
    jointCmdPub[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("aliengo_gazebo/FR_thigh_controller/command", 1);
    jointCmdPub[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("aliengo_gazebo/FR_calf_controller/command", 1);
    jointCmdPub[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("aliengo_gazebo/FL_hip_controller/command", 1);
    jointCmdPub[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("aliengo_gazebo/FL_thigh_controller/command", 1);
    jointCmdPub[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("aliengo_gazebo/FL_calf_controller/command", 1);
    jointCmdPub[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("aliengo_gazebo/RR_hip_controller/command", 1);
    jointCmdPub[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("aliengo_gazebo/RR_thigh_controller/command", 1);
    jointCmdPub[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("aliengo_gazebo/RR_calf_controller/command", 1);
    jointCmdPub[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("aliengo_gazebo/RL_hip_controller/command", 1);
    jointCmdPub[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("aliengo_gazebo/RL_thigh_controller/command", 1);
    jointCmdPub[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("aliengo_gazebo/RL_calf_controller/command", 1);

    // ros::spinOnce();
    usleep(300000); // must wait 300ms, to get first state

    yawOffset = lowState.imu.rpy[2];
    std::cout << "yawOffset: " << yawOffset << std::endl;

    timeStep = 0.001;
    this->ResetTimer();
    lastResetTime = GetTimeSinceReset();
    initComplete = true;

    std::cout << "-------AliengoSim init Complete-------" << std::endl;
}


void qrRobotAliengoSim::ImuCallback(const sensor_msgs::Imu &msg)
{
    lowState.imu.quaternion[0] = msg.orientation.w;
    lowState.imu.quaternion[1] = msg.orientation.x;
    lowState.imu.quaternion[2] = msg.orientation.y;
    lowState.imu.quaternion[3] = msg.orientation.z;

    Eigen::Matrix<float, 4, 1> quaternion = {lowState.imu.quaternion[0],
                                             lowState.imu.quaternion[1],
                                             lowState.imu.quaternion[2],
                                             lowState.imu.quaternion[3]};
    Eigen::Matrix<float, 3, 1> rpy = robotics::math::quatToRPY(quaternion);

    lowState.imu.rpy[0] = rpy[0];
    lowState.imu.rpy[1] = rpy[1];
    lowState.imu.rpy[2] = rpy[2];

    lowState.imu.gyroscope[0] = msg.angular_velocity.x;
    lowState.imu.gyroscope[1] = msg.angular_velocity.y;
    lowState.imu.gyroscope[2] = msg.angular_velocity.z;

    lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
    lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
    lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
}


void qrRobotAliengoSim::FRhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[0].mode = msg.mode;
    lowState.motorState[0].q = msg.q;
    lowState.motorState[0].dq = msg.dq;
}


void qrRobotAliengoSim::FRthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[1].mode = msg.mode;
    lowState.motorState[1].q = msg.q;
    lowState.motorState[1].dq = msg.dq;
    lowState.motorState[1].tauEst = msg.tauEst;
}


void qrRobotAliengoSim::FRcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[2].mode = msg.mode;
    lowState.motorState[2].q = msg.q;
    lowState.motorState[2].dq = msg.dq;
    lowState.motorState[2].tauEst = msg.tauEst;
}


void qrRobotAliengoSim::FLhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[3].mode = msg.mode;
    lowState.motorState[3].q = msg.q;
    lowState.motorState[3].dq = msg.dq;
    lowState.motorState[3].tauEst = msg.tauEst;
}


void qrRobotAliengoSim::FLthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[4].mode = msg.mode;
    lowState.motorState[4].q = msg.q;
    lowState.motorState[4].dq = msg.dq;
    lowState.motorState[4].tauEst = msg.tauEst;
}


void qrRobotAliengoSim::FLcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[5].mode = msg.mode;
    lowState.motorState[5].q = msg.q;
    lowState.motorState[5].dq = msg.dq;
    lowState.motorState[5].tauEst = msg.tauEst;
}


void qrRobotAliengoSim::RRhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[6].mode = msg.mode;
    lowState.motorState[6].q = msg.q;
    lowState.motorState[6].dq = msg.dq;
    lowState.motorState[6].tauEst = msg.tauEst;
}


void qrRobotAliengoSim::RRthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[7].mode = msg.mode;
    lowState.motorState[7].q = msg.q;
    lowState.motorState[7].dq = msg.dq;
    lowState.motorState[7].tauEst = msg.tauEst;
}


void qrRobotAliengoSim::RRcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[8].mode = msg.mode;
    lowState.motorState[8].q = msg.q;
    lowState.motorState[8].dq = msg.dq;
    lowState.motorState[8].tauEst = msg.tauEst;
}


void qrRobotAliengoSim::RLhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[9].mode = msg.mode;
    lowState.motorState[9].q = msg.q;
    lowState.motorState[9].dq = msg.dq;
    lowState.motorState[9].tauEst = msg.tauEst;
}


void qrRobotAliengoSim::RLthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[10].mode = msg.mode;
    lowState.motorState[10].q = msg.q;
    lowState.motorState[10].dq = msg.dq;
    lowState.motorState[10].tauEst = msg.tauEst;
}


void qrRobotAliengoSim::RLcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[11].mode = msg.mode;
    lowState.motorState[11].q = msg.q;
    lowState.motorState[11].dq = msg.dq;
    lowState.motorState[11].tauEst = msg.tauEst;
}


void qrRobotAliengoSim::FRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    lowState.eeForce[0].x = msg.wrench.force.x;
    lowState.eeForce[0].y = msg.wrench.force.y;
    lowState.eeForce[0].z = msg.wrench.force.z;
    lowState.footForce[0] = msg.wrench.force.z;
}


void qrRobotAliengoSim::FLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    lowState.eeForce[1].x = msg.wrench.force.x;
    lowState.eeForce[1].y = msg.wrench.force.y;
    lowState.eeForce[1].z = msg.wrench.force.z;
    lowState.footForce[1] = msg.wrench.force.z;
}


void qrRobotAliengoSim::RRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    lowState.eeForce[2].x = msg.wrench.force.x;
    lowState.eeForce[2].y = msg.wrench.force.y;
    lowState.eeForce[2].z = msg.wrench.force.z;
    lowState.footForce[2] = msg.wrench.force.z;
}


void qrRobotAliengoSim::RLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    lowState.eeForce[3].x = msg.wrench.force.x;
    lowState.eeForce[3].y = msg.wrench.force.y;
    lowState.eeForce[3].z = msg.wrench.force.z;
    lowState.footForce[3] = msg.wrench.force.z;
}


void qrRobotAliengoSim::SendCommand(const std::array<float, 60> motorcmd)
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
    // ros::spinOnce();
    // usleep(1000);
}


void qrRobotAliengoSim::ReceiveObservation()
{
    // ros::spinOnce();
    // usleep(1000);
    unitree_legged_msgs::LowState state = lowState; //callback

    // tick = state.tick;
    baseAccInBaseFrame << state.imu.accelerometer[0],
                          state.imu.accelerometer[1],
                          state.imu.accelerometer[2];
    // stateDataFlow.baseLinearAcceleration = accFilter.CalculateAverage(baseAccInBaseFrame);
    boost::array<float, 3> rpy = state.imu.rpy;
    boost::array<float, 3> gyro = state.imu.gyroscope;
    // std::array<float, 4> quaternion = state.imu.quaternion;
    // baseOrientation << quaternion[0], quaternion[1], quaternion[2], quaternion[3]; // w,x,y,z

    // calibrated
    float calibratedYaw = rpy[2] - yawOffset;
    if (calibratedYaw >= M_PI)
        calibratedYaw -= M_2PI;
    else if (calibratedYaw <= -M_PI)
        calibratedYaw += M_2PI;
    baseRollPitchYaw << rpy[0], rpy[1], calibratedYaw;
    baseOrientation = robotics::math::rpyToQuat(baseRollPitchYaw);

    baseRollPitchYawRate << gyro[0], gyro[1], gyro[2];
    for (int motorId = 0; motorId < NumMotor; motorId++) {
        motorAngles[motorId] = state.motorState[motorId].q;
        motorVelocities[motorId] = state.motorState[motorId].dq;
    }
    boost::array<int16_t, 4> force = state.footForce;
    footForce << force[0], force[1], force[2], force[3];
    for (int footId = 0; footId < NumLeg; footId++) {
        if (footForce[footId] > 5) {
            footContact[footId] = true;
        } else {
            footContact[footId] = false;
        }
    }
}


void qrRobotAliengoSim::ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode)
{
    std::array<float, 60> motorCommandsArray = {0};
    if (motorControlMode == POSITION_MODE) {
        Eigen::Matrix<float, 1, 12> motorCommandsShaped = motorCommands.transpose();
        for (int motorId = 0; motorId < NumMotor; motorId++) {
            motorCommandsArray[motorId * 5] = motorCommandsShaped[motorId];
            motorCommandsArray[motorId * 5 + 1] = motorKps[motorId];
            motorCommandsArray[motorId * 5 + 2] = 0;
            motorCommandsArray[motorId * 5 + 3] = motorKds[motorId];
            motorCommandsArray[motorId * 5 + 4] = 0;
        }
    } else if (motorControlMode == TORQUE_MODE) {
        Eigen::Matrix<float, 1, 12> motorCommandsShaped = motorCommands.transpose();
        for (int motorId = 0; motorId < NumMotor; motorId++) {
            motorCommandsArray[motorId * 5] = 0;
            motorCommandsArray[motorId * 5 + 1] = 0;
            motorCommandsArray[motorId * 5 + 2] = 0;
            motorCommandsArray[motorId * 5 + 3] = 0;
            motorCommandsArray[motorId * 5 + 4] = motorCommandsShaped[motorId];
        }
    } else if (motorControlMode == HYBRID_MODE) {
        Eigen::Matrix<float, 5, 12> motorCommandsShaped = motorCommands;
        for (int motorId = 0; motorId < NumMotor; motorId++) {
            motorCommandsArray[motorId * 5] = motorCommandsShaped(POSITION, motorId);
            motorCommandsArray[motorId * 5 + 1] = motorCommandsShaped(KP, motorId);
            motorCommandsArray[motorId * 5 + 2] = motorCommandsShaped(VELOCITY, motorId);
            motorCommandsArray[motorId * 5 + 3] = motorCommandsShaped(KD, motorId);
            motorCommandsArray[motorId * 5 + 4] = motorCommandsShaped(TORQUE, motorId);
        }
    }

    for (int index=0; index< motorCommandsArray.size(); index++) {
        if (isnan(motorCommandsArray[index])) {
            motorCommandsArray[index] = 0.f;
        }
    }
    SendCommand(motorCommandsArray);
}


void qrRobotAliengoSim::ApplyAction(const std::vector<qrMotorCommand> &motorCommands, MotorMode motorControlMode)
{
    std::array<float, 60> motorCommandsArray = {0};
    for (int motorId = 0; motorId < NumMotor; motorId++) {
        motorCommandsArray[motorId * 5] = motorCommands[motorId].p;
        motorCommandsArray[motorId * 5 + 1] = motorCommands[motorId].Kp;
        motorCommandsArray[motorId * 5 + 2] = motorCommands[motorId].d;
        motorCommandsArray[motorId * 5 + 3] = motorCommands[motorId].Kd;
        motorCommandsArray[motorId * 5 + 4] = motorCommands[motorId].tua;
    }
    // robotInterface.SendCommand(motorCommandsArray);
}


void qrRobotAliengoSim::Step(const Eigen::MatrixXf &action, MotorMode motorControlMode)
{
    ReceiveObservation();
    ApplyAction(action, motorControlMode);
}

} // namespace Quadruped

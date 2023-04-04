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

#include "robots/qr_robot_lite2.h"


using std::cout;
using std::endl;


namespace Quadruped {

qrRobotLite2::qrRobotLite2(std::string configFilePath)
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
    totalMass = robotConfig["robot_params"]["total_mass"].as<float>();
    bodyMass = robotConfig["robot_params"]["body_mass"].as<float>();
    std::vector<float> totalInertiaVec = robotConfig["robot_params"]["total_inertia"].as<std::vector<float >>();
    totalInertia = Eigen::MatrixXf::Map(&totalInertiaVec[0], 3, 3);
    std::vector<float> bodyInertiaVec = robotConfig["robot_params"]["body_inertia"].as<std::vector<float >>();
    bodyInertia = Eigen::MatrixXf::Map(&bodyInertiaVec[0], 3, 3);
    
    std::vector<std::vector<float>> inertias = robotConfig["robot_params"]["links_inertia"].as<std::vector<std::vector<float>>>();
    std::vector<float> masses = robotConfig["robot_params"]["links_mass"].as<std::vector<float>>();
    std::vector<std::vector<float>> linksComPos_ = robotConfig["robot_params"]["links_com_pos"].as<std::vector<std::vector<float>>>();

    for (int legId=0; legId<NumLeg;++legId) {
        Mat3<float> inertia = Mat3<float>::Zero();
        inertia = Eigen::MatrixXf::Map(&inertias[0][0], 3, 3);
        linkInertias.push_back(inertia); // hip link
        inertia = Eigen::MatrixXf::Map(&inertias[1][0], 3, 3);
        linkInertias.push_back(inertia); // thigh link
        inertia = Eigen::MatrixXf::Map(&inertias[2][0], 3, 3);
        linkInertias.push_back(inertia); // calf link

        linkMasses.push_back(masses[0]);
        linkMasses.push_back(masses[1]);
        linkMasses.push_back(masses[2]);

        linksComPos.push_back(linksComPos_[0]);
        linksComPos.push_back(linksComPos_[1]);
        linksComPos.push_back(linksComPos_[2]);
    }
    
    bodyHeight = robotConfig["robot_params"]["body_height"].as<float>();
    std::vector<float> abadLocation_ = robotConfig["robot_params"]["abad_location"].as<std::vector<float>>();
    abadLocation = Eigen::MatrixXf::Map(&abadLocation_[0], 3, 1);
    hipLength = robotConfig["robot_params"]["hip_l"].as<float>();
    upperLegLength = robotConfig["robot_params"]["upper_l"].as<float>();
    lowerLegLength = robotConfig["robot_params"]["lower_l"].as<float>();
    
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

    // float standUpAbAngle, standUpHipAngle, standUpKneeAngle;
    // standUpAbAngle = 0.f;
    // standUpHipAngle = std::acos(bodyHeight / 2.f / upperLegLength);
    // standUpKneeAngle = -2.f * standUpHipAngle;
    // Eigen::Matrix<float, 3, 1> defaultStandUpAngle(standUpAbAngle, standUpHipAngle, standUpKneeAngle);
    Eigen::Matrix<float, 3, 1> defaultStandUpAngle;
    float standUpAbAngle, standUpHipAngle, standUpKneeAngle;
    standUpAbAngle = robotConfig["robot_params"]["default_standup_angle"]["ab"].as<float>();
    standUpHipAngle = robotConfig["robot_params"]["default_standup_angle"]["hip"].as<float>();
    standUpKneeAngle = robotConfig["robot_params"]["default_standup_angle"]["knee"].as<float>();
    defaultStandUpAngle << standUpAbAngle, standUpHipAngle, standUpKneeAngle;
    standUpMotorAngles << defaultStandUpAngle, defaultStandUpAngle, defaultStandUpAngle, defaultStandUpAngle;

    float sitDownAbAngle, sitDownHipAngle, sitDownKneeAngle;
    sitDownAbAngle = robotConfig["robot_params"]["default_sitdown_angle"]["ab"].as<float>();
    sitDownHipAngle = robotConfig["robot_params"]["default_sitdown_angle"]["hip"].as<float>();
    sitDownKneeAngle = robotConfig["robot_params"]["default_sitdown_angle"]["knee"].as<float>();
    Eigen::Matrix<float, 3, 1> defaultSitDownAngle(sitDownAbAngle, sitDownHipAngle, sitDownKneeAngle);
    sitDownMotorAngles << defaultSitDownAngle, defaultSitDownAngle, defaultSitDownAngle, defaultSitDownAngle;

    controlParams["mode"] = robotConfig["controller_params"]["mode"].as<int>();
    Reset(); // reset com_offset

    // timeStep = 0.001;
    // timeStep = 1.0 / robotConfig["controller_params"]["freq"].as<int>();
    
    ResetTimer();
    lastResetTime = GetTimeSinceReset();
    initComplete = true;

    // lowState_lite2 = robotInterface.ReceiveObservation();
    // lite2Sender.startWork();
    lite2Sender.init();
    lite2Receiver.startWork();
    // lite2Sender.control_get(ABLE);
    lite2Sender.robot_state_init();
    long long count = 0;
    
    while (1) {
        // ((RobotLite2*)quadruped)->lite2Sender.control_get(ABLE);
        ReceiveObservation();
        count++;
        std::cout << "count = " << count << ",  " << baseRollPitchYaw.transpose() << std::endl;
        if (abs(baseRollPitchYaw[1]) > 1e-5) break;
        usleep(1000);
    }

    yawOffset = baseRollPitchYaw[2];
    std::cout << "yawOffset: " << yawOffset << std::endl;
    std::cout << "-------RobotLite2 init Complete-------" << std::endl;
    // sleep(2);
    // save robot data into npy format
    // trainData.resize(NumX*NumY, 0.f);
    // trainDataLabel.resize(NumX, 0);
    // std::cout << trainData.max_size();
}


void qrRobotLite2::ReceiveObservation()
{
    RobotState& state = lite2Receiver.get_recv();
    // lowState_lite2 = state;

    tick = state.tick;
    std::array<float, 3> rpy;
    // std::copy(std::begin(state.imu.rpy), std::end(state.imu.rpy), std::begin(rpy));
    rpy[0] = state.imu.angle_roll / 57.3;
    rpy[1] = state.imu.angle_pitch / 57.3;
    rpy[2] = state.imu.angle_yaw / 57.3;
    
    std::array<float, 3> gyro;
    // std::copy(std::begin(state.imu.gyroscope), std::end(state.imu.gyroscope), std::begin(gyro));
    gyro[0] = state.imu.angular_velocity_roll; // degree to radian
    gyro[1] = state.imu.angular_velocity_pitch;
    gyro[2] = state.imu.angular_velocity_yaw;
    
    baseAccInBaseFrame[0] = state.imu.acc_x;
    baseAccInBaseFrame[1] = state.imu.acc_y;
    baseAccInBaseFrame[2] = state.imu.acc_z;
    
    stateDataFlow.baseLinearAcceleration = accFilter.CalculateAverage(baseAccInBaseFrame);
    // stateDataFlow.baseLinearAcceleration << acc[0], acc[1], acc[2];
    float calibratedYaw = rpy[2] - yawOffset;
    while (calibratedYaw >= M_PI)
        calibratedYaw -= M_2PI;
    while (calibratedYaw <= -M_PI)
        calibratedYaw += M_2PI;
    while (rpy[0] >= M_PI)
        rpy[0] -= M_2PI;
    while (rpy[0] <= -M_PI)
        rpy[0] += M_2PI; 
    while (rpy[1] >= M_PI)
        rpy[1] -= M_2PI;
    while (rpy[1] <= -M_PI)
        rpy[1] += M_2PI;
    // float groundPitch = robotics::math::quatToRPY(stateDataFlow.groundOrientation)[1];
    
    baseRollPitchYaw << rpy[0], rpy[1], calibratedYaw;
    baseOrientation = robotics::math::rpyToQuat(baseRollPitchYaw);

    Vec3<float> gyroVec(gyro[0], gyro[1], gyro[2]);
    baseRollPitchYawRate = gyroFilter.CalculateAverage(gyroVec);
    // baseRollPitchYawRate << gyro[0], gyro[1], gyro[2];

    for (int motorId = 0; motorId < NumMotor; motorId++) {
        int motorId_ = ((motorId/3)%2 == 0? motorId+3: motorId-3);
        motorAngles[motorId] = state.motor_state.joint_data[motorId_].pos;
        motorVelocities[motorId] = state.motor_state.joint_data[motorId_].vel;
        // motorddq[motorId] = state.motor_state[motorId].ddq;
        motortorque[motorId] = state.motor_state.joint_data[motorId_].tor;
    }
    motorAngles = jointDirection.cwiseProduct(motorAngles + jointOffset);
    motorVelocities = jointDirection.cwiseProduct(motorVelocities);
    // motorddq = jointDirection.cwiseProduct(motorddq);
    motortorque = jointDirection.cwiseProduct(motortorque);

    motorVelocities = motorVFilter.CalculateAverage(motorVelocities);

    // std::array<int16_t, 4> force;
    // std::copy(std::begin(state.footForce), std::end(state.footForce), std::begin(force));
    // footForce << force[0], force[1], force[2], force[3];
    footForce << (float)state.fr_tor[2], (float)state.fl_tor[2], (float)state.hr_tor[2], (float)state.hl_tor[2];

    for (int footId = 0; footId < NumLeg; ++footId) {
        // 20 is contact threshold
        if (footForce[footId] >= 28) { // 25
            footContact[footId] = true;
        } else {
            footContact[footId] = false;
        }
    }

    UpdateDataFlow();
    // RecordDate(0);
}


void qrRobotLite2::ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode)
{
    // std::array<float, 60> motorCommandsArray = {0};
    RobotCmd motorCommandsArray;
    float t = GetTimeSinceReset();

    if (motorControlMode == POSITION_MODE) {
        Eigen::Matrix<float, 12, 1> motorCommandsShaped = motorCommands;
        motorCommandsShaped = jointDirection.cwiseProduct(motorCommandsShaped) - jointOffset;
        
        for (int motorId = 0; motorId < NumMotor; motorId++) {
            int motorId_ = (motorId/3)%2 == 0? motorId+3: motorId-3;
            /*
            if (motorId / 3 > 1) {
                // tau = 0.;
                motorCommandsShaped[motorId] = 0;
                motorKps[motorId] = 0;
                motorKds[motorId] = 0;
            }
            if (motorId % 3 != 1) {
                // tau = 0;
                motorCommandsShaped[motorId] = 0;
                motorKps[motorId] = 0;
                motorKds[motorId] = 0;
            } 
            */
            motorCommandsArray.joint_cmd[motorId_].pos = motorCommandsShaped[motorId];
            motorCommandsArray.joint_cmd[motorId_].kp = motorKps[motorId];
            motorCommandsArray.joint_cmd[motorId_].vel = 0;
            motorCommandsArray.joint_cmd[motorId_].kd = motorKds[motorId];
            motorCommandsArray.joint_cmd[motorId_].tor = 0;
        }
    } else if (motorControlMode == TORQUE_MODE) {
        Eigen::Matrix<float, 12, 1> motorCommandsShaped = motorCommands;
        motorCommandsShaped = jointDirection.cwiseProduct(motorCommandsShaped);
        
        for (int motorId = 0; motorId < NumMotor; motorId++) {
            int motorId_ = (motorId/3)%2 == 0? motorId+3: motorId-3;
            float tau = motorCommandsShaped[motorId];
            // if (motorId % 3 == 1) {
            //     tau = tau/5;
            // }
            /*
            if (motorId/3 < 2 && motorId % 3 == 1) {
                // tau = 0;
        
            if (abs(t - 10.0) < 1e-0) {
                    tau = 3;
            } else {
                tau = 0;
            }
            } else {
                tau = 0;
            }
            */
            //if (motorId % 3 != 2) {
            //    tau = 0;
            //}
            motorCommandsArray.joint_cmd[motorId_].pos = 0;
            motorCommandsArray.joint_cmd[motorId_].kp = 0;
            motorCommandsArray.joint_cmd[motorId_].vel = 0;
            motorCommandsArray.joint_cmd[motorId_].kd = 0;
            motorCommandsArray.joint_cmd[motorId_].tor = tau;
        }
    } else if (motorControlMode == HYBRID_MODE) {
        Eigen::Matrix<float, 5, 12> motorCommandsShaped = motorCommands;
        Eigen::Matrix<float, 12, 1> angles = motorCommandsShaped.row(POSITION).transpose();
        motorCommandsShaped.row(POSITION) = (jointDirection.cwiseProduct(angles) - jointOffset).transpose();
        Eigen::Matrix<float, 12, 1> vels = motorCommandsShaped.row(VELOCITY).transpose();
        motorCommandsShaped.row(VELOCITY) = jointDirection.cwiseProduct(vels).transpose();
        Eigen::Matrix<float, 12, 1> tuas = motorCommandsShaped.row(TORQUE).transpose();
        motorCommandsShaped.row(TORQUE) = jointDirection.cwiseProduct(tuas).transpose();
        
        for (int motorId = 0; motorId < NumMotor; motorId++) {
            int motorId_ = (motorId/3)%2 == 0? motorId+3: motorId-3;
            motorCommandsArray.joint_cmd[motorId_].pos = motorCommandsShaped(POSITION, motorId);
            motorCommandsArray.joint_cmd[motorId_].kp = motorCommandsShaped(KP, motorId);
            motorCommandsArray.joint_cmd[motorId_].vel = motorCommandsShaped(VELOCITY, motorId);
            motorCommandsArray.joint_cmd[motorId_].kd = motorCommandsShaped(KD, motorId);
            motorCommandsArray.joint_cmd[motorId_].tor = motorCommandsShaped(TORQUE, motorId);
        }
    }
    // robotInterface.SendCommand(motorCommandsArray);
    lite2Sender.set_send(motorCommandsArray); // todo
}


void qrRobotLite2::ApplyAction(const std::vector<qrMotorCommand> &motorCommands, MotorMode motorControlMode)
{
    std::array<float, 60> motorCommandsArray = {0};
    for (int motorId = 0; motorId < NumMotor; motorId++) {
        motorCommandsArray[motorId * 5] = motorCommands[motorId].p * jointDirection(motorId) - jointOffset(motorId);
        motorCommandsArray[motorId * 5 + 1] = motorCommands[motorId].Kp;
        motorCommandsArray[motorId * 5 + 2] = motorCommands[motorId].d * jointDirection(motorId);
        motorCommandsArray[motorId * 5 + 3] = motorCommands[motorId].Kd;
        motorCommandsArray[motorId * 5 + 4] = motorCommands[motorId].tua * jointDirection(motorId);
    }
    // robotInterface.SendCommand(motorCommandsArray);
}


void qrRobotLite2::Step(const Eigen::MatrixXf &action, MotorMode motorControlMode)
{
    // ReceiveObservation();
    ApplyAction(action, motorControlMode);
    ReceiveObservation();
}


bool qrRobotLite2::BuildDynamicModel()
{
    // we assume the cheetah's body (not including rotors) can be modeled as a
    // uniformly distributed box.
    std::vector<float> bodySize = robotConfig["robot_params"]["body_size"].as<std::vector<float>>(); // Length, Width, Height
    Vec3<float> bodyDims(bodySize[0], bodySize[1], bodySize[2]);
    
    // locations
    // Vec3<float> _abadRotorLocation = {0.14f, 0.047f, 0.f}; // a1
    Vec3<float> _abadRotorLocation = {0.935f, 0.062f, 0.f}; // lite3
    Vec3<float> _abadLocation = abadLocation;        
    Vec3<float> _hipLocation = Vec3<float>(0, hipLength, 0);
    // Vec3<float> _hipRotorLocation = Vec3<float>(0, 0.04, 0); // a1
    Vec3<float> _hipRotorLocation = Vec3<float>(0, 0.0, 0); // lite3
    Vec3<float> _kneeLocation = Vec3<float>(0, 0, -upperLegLength);
    // Vec3<float> _kneeRotorLocation = Vec3<float>(0, 0, 0); // a1
    Vec3<float> _kneeRotorLocation = Vec3<float>(0, -0.50, 0); // lite3

    float scale_ = 1.0; //1e-2(sim);
    // rotor inertia if the rotor is oriented so it spins around the z-axis
    Mat3<float> rotorRotationalInertiaZ;
    // rotorRotationalInertiaZ << 33, 0, 0, // a1
    //                            0, 33, 0,
    //                            0, 0, 63;
    rotorRotationalInertiaZ << 11, 0, 0, // lite3
                               0, 18, 0,
                               0, 0, 11;
    rotorRotationalInertiaZ.setIdentity();
    rotorRotationalInertiaZ = scale_*1e-6 * rotorRotationalInertiaZ;

    Mat3<float> RY = coordinateRotation<float>(CoordinateAxis::Y, M_PI / 2);
    Mat3<float> RX = coordinateRotation<float>(CoordinateAxis::X, M_PI / 2);
    Mat3<float> rotorRotationalInertiaX = RY * rotorRotationalInertiaZ * RY.transpose();
    Mat3<float> rotorRotationalInertiaY = RX * rotorRotationalInertiaZ * RX.transpose();

    // spatial inertias of leg links
    Mat3<float> abadRotationalInertia = linkInertias[0];
    Vec3<float> abadCOM(linksComPos[0][0],linksComPos[0][1],linksComPos[0][2]);
    std::cout << "abadCOM=" << abadCOM << std::endl;
    std::cout << "linkMasses[0]=" << linkMasses[0] << std::endl;
    std::cout << "abadRotationalInertia=" << abadRotationalInertia << std::endl;
    SpatialInertia<float> abadInertia(linkMasses[0], abadCOM, abadRotationalInertia);

    Mat3<float> hipRotationalInertia = linkInertias[1];
    // Vec3<float> hipCOM(-0.003237, -0.022327, -0.027326); // a1. left, for right filp y-axis value.
    Vec3<float> hipCOM(linksComPos[1][0],linksComPos[1][1],linksComPos[1][2]);
    SpatialInertia<float> hipInertia(linkMasses[1], hipCOM, hipRotationalInertia);
    std::cout << "linkMasses[1]=" << linkMasses[1] << std::endl;
    std::cout << "hipRotationalInertia=" << hipRotationalInertia << std::endl;
    
    Mat3<float> kneeRotationalInertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated = linkInertias[2];
    kneeRotationalInertia = kneeRotationalInertiaRotated;//RY * kneeRotationalInertiaRotated * RY.transpose();
    
    Vec3<float> kneeCOM(linksComPos[2][0],linksComPos[2][1],linksComPos[2][2]);
    SpatialInertia<float> kneeInertia(linkMasses[2], kneeCOM, kneeRotationalInertia);
    std::cout << "linkMasses[2]=" << linkMasses[2] << std::endl;
    std::cout << "kneeRotationalInertia=" << kneeRotationalInertia << std::endl;
    
    // rotors
    Vec3<float> rotorCOM(0, 0, 0);
    float rotorMass = 0.0708; // 1e-8, 0.055(a1), 0.0708(lite3)
    SpatialInertia<float> rotorInertiaX(rotorMass, rotorCOM, rotorRotationalInertiaX);
    SpatialInertia<float> rotorInertiaY(rotorMass, rotorCOM, rotorRotationalInertiaY);
    auto& abadRotorInertia = rotorInertiaX;
    float abadGearRatio = 12; //1(sim), 6(a1), 12(lite3)
    auto& hipRotorInertia = rotorInertiaY;
    float hipGearRatio = 12; // 6, 12
    auto& kneeRotorInertia = rotorInertiaY;
    float kneeGearRatio = 18; // 9.33, 18
    float kneeLinkY_offset = 0.004;
    
    // body
    Mat3<float> bodyRotationalInertia = bodyInertia;
    Vec3<float> bodyCOM = comOffset;
    std::cout << "totalMass = " << totalMass << ", bodyMass = " << bodyMass << std::endl;
    SpatialInertia<float> bodyInertia_(bodyMass, bodyCOM, bodyRotationalInertia);
    
    const int baseID = 5;
    int bodyID = baseID;
    float sideSign = -1;
    Mat3<float> I3 = Mat3<float>::Identity();
    model.addBase(bodyInertia_);
    model.addGroundContactBoxPoints(bodyID, bodyDims);

    for (int legID = 0; legID < NumLeg; legID++) {
        // Ab/Ad joint
        //  int addBody(const SpatialInertia<T>& inertia, const SpatialInertia<T>& rotorInertia, T gearRatio,
            //  int parent, JointType jointType, CoordinateAxis jointAxis,
            //  const Mat6<T>& Xtree, const Mat6<T>& Xrot);
        bodyID++;

        Vec3<float> offsetZ(0,0,0);
        // if (legID>1) offsetZ << 0,0,+0.02; //todo
        
        Mat6<float> xtreeAbad = createSXform(I3, WithLegSigns(_abadLocation + offsetZ, legID));
        Mat6<float> xtreeAbadRotor = createSXform(I3, WithLegSigns(_abadRotorLocation, legID));
        if (sideSign < 0) {
            model.addBody(abadInertia.flipAlongAxis(CoordinateAxis::Y),
                          abadRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                          abadGearRatio, baseID, JointType::Revolute,
                          CoordinateAxis::X, xtreeAbad, xtreeAbadRotor);
        } else {
            model.addBody(abadInertia, abadRotorInertia, abadGearRatio, baseID,
                          JointType::Revolute, CoordinateAxis::X, xtreeAbad,
                          xtreeAbadRotor);
        }
        // model.addGroundContactPoint(bodyID, withLegSigns(_hipLocation, legID));

        // Hip Joint
        bodyID++;
        Mat6<float> xtreeHip =
            createSXform(I3, //coordinateRotation(CoordinateAxis::Z, float(M_PI)),
                        WithLegSigns(_hipLocation, legID)); // 0, hipLength=0.085, 0
        Mat6<float> xtreeHipRotor =
            createSXform(coordinateRotation(CoordinateAxis::Z, float(M_PI)),
                        WithLegSigns(_hipRotorLocation, legID));
        if (sideSign < 0) {
            model.addBody(hipInertia.flipAlongAxis(CoordinateAxis::Y),
                          hipRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                          hipGearRatio, bodyID - 1, JointType::Revolute,
                          CoordinateAxis::Y, xtreeHip, xtreeHipRotor);
        } else {
            model.addBody(hipInertia, hipRotorInertia, hipGearRatio, bodyID - 1,
                          JointType::Revolute, CoordinateAxis::Y, xtreeHip,
                          xtreeHipRotor);
        }

        // add knee ground contact point
        model.addGroundContactPoint(bodyID, Vec3<float>(0, 0, -upperLegLength));

        // Knee Joint
        bodyID++;
        Mat6<float> xtreeKnee = createSXform(I3, _kneeLocation);
        Mat6<float> xtreeKneeRotor = createSXform(I3, _kneeRotorLocation);
        if (sideSign < 0) {
            model.addBody(kneeInertia, //.flipAlongAxis(CoordinateAxis::Y),
                          kneeRotorInertia,//.flipAlongAxis(CoordinateAxis::Y),
                          kneeGearRatio, bodyID - 1, JointType::Revolute,
                          CoordinateAxis::Y, xtreeKnee, xtreeKneeRotor);

            model.addGroundContactPoint(bodyID, Vec3<float>(0, kneeLinkY_offset, -lowerLegLength), true);
        } else {
            model.addBody(kneeInertia, kneeRotorInertia, kneeGearRatio, bodyID - 1,
                          JointType::Revolute, CoordinateAxis::Y, xtreeKnee,
                          xtreeKneeRotor);

            model.addGroundContactPoint(bodyID, Vec3<float>(0, -kneeLinkY_offset, -lowerLegLength), true);
        }

        // add foot
        //model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, -_kneeLinkLength), true);

        sideSign *= -1;
    }

    Vec3<float> g(0, 0, -9.81);
    model.setGravity(g);

    return true;
}

} // namespace Quadruped

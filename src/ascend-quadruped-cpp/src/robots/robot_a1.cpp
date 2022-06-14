/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: inherited from quadruped robot, name as A1.
* Author: Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhao Yao
*/


#include "robots/robot_a1.h"

namespace Quadruped {
    RobotA1::RobotA1(std::string configFilePath)
    {
        baseLinearAcceleration << 0.f, 0.f, 0.f;
        baseOrientation << 1.f, 0.f, 0.f, 0.f;
        baseRollPitchYaw << 0.f, 0.f, 0.f;
        baseRollPitchYawRate << 0.f, 0.f, 0.f;
        motorVelocities = Eigen::Matrix<float, 12, 1>::Zero();
        footForce << 0.f, 0.f, 0.f, 0.f;
        footContact << 1, 1, 1, 1;

        this->configFilePath = configFilePath;
        robotConfig = YAML::LoadFile(configFilePath);

        robotName = robotConfig["name"].as<std::string>();

        bodyMass = robotConfig["robot_params"]["body_mass"].as<float>();
        std::vector<float> bodyInertiaList = robotConfig["robot_params"]["body_inertia"].as<std::vector<float >>();
        bodyInertia = Eigen::MatrixXf::Map(&bodyInertiaList[0], 3, 3);
        bodyHeight = robotConfig["robot_params"]["body_height"].as<float>();

        hipLength = robotConfig["robot_params"]["hip_l"].as<float>();
        upperLegLength = robotConfig["robot_params"]["upper_l"].as<float>();
        lowerLegLength = robotConfig["robot_params"]["lower_l"].as<float>();
        
        controlParams["mode"] = robotConfig["controller_params"]["mode"].as<int>(); // types.h: enum

        std::vector<float> comOffsetList = robotConfig["robot_params"][modeMap[controlParams["mode"]]]["com_offset"].as<std::vector<float >>();
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

        timeStep = 0.001;
        this->ResetTimer();
        lastResetTime = GetTimeSinceReset();
        initComplete = true;

        lowState = robotInterface.ReceiveObservation();
        yawOffset = lowState.imu.rpy[2];

        std::cout << "yawOffset: " << yawOffset << std::endl;
        std::cout << "-------RobotA1 init Complete-------" << std::endl;
        // save robot data into npy format
        // trainData.resize(NumX*NumY, 0.f);
        // trainDataLabel.resize(NumX, 0);
        // std::cout << trainData.max_size();
    }

    void RobotA1::ReceiveObservation()
    {
        LowState state = robotInterface.ReceiveObservation();
        lowState = state;
        // std::array<float, 3> rpy = state.imu.rpy;
        // std::array<float, 3> gyro = state.imu.gyroscope;
        std::array<float, 3> rpy;
        std::copy(std::begin(state.imu.rpy), std::end(state.imu.rpy), std::begin(rpy));
        std::array<float, 3> gyro;
        std::copy(std::begin(state.imu.gyroscope), std::end(state.imu.gyroscope), std::begin(gyro));
        // std::array<float, 4> quaternion = state.imu.quaternion;
        // baseOrientation << quaternion[0], quaternion[1], quaternion[2], quaternion[3]; // w,x,y,z
        std::array<float, 3> acc;
        std::copy(std::begin(state.imu.accelerometer), std::end(state.imu.accelerometer), std::begin(acc));
        baseLinearAcceleration << acc[0], acc[1], acc[2];
        // calibrated
        float calibratedYaw = rpy[2] - yawOffset;
        if (calibratedYaw >= M_PI)
            calibratedYaw -= 2 * M_PI;
        else if (calibratedYaw <= -M_PI)
            calibratedYaw += 2 * M_PI;
        baseRollPitchYaw << rpy[0], rpy[1], calibratedYaw;
        baseOrientation = robotics::math::rpyToQuat(baseRollPitchYaw);

        baseRollPitchYawRate << gyro[0], gyro[1], gyro[2];
        for (int motorId = 0; motorId < numMotors; motorId++) {
            motorAngles[motorId] = state.motorState[motorId].q;
            motorVelocities[motorId] = state.motorState[motorId].dq;
        }
        std::array<int16_t, 4> force;
        std::copy(std::begin(state.footForce), std::end(state.footForce), std::begin(force));
        footForce << force[0], force[1], force[2], force[3];
        // std::cout << "[reve] footForce = " << footForce.transpose() << std::endl;
        for (int footId = 0; footId < numLegs; footId++) {
            // 20 is contact threshold
            if (footForce[footId] >= 20) {
                footContact[footId] = true;
            } else {
                footContact[footId] = false;
            }
        }
        // RecordDate(0);
    }

    void RobotA1::SaveData(std::string fileName)
    {
        cnpy::npy_save(fileName+".npy",&trainData[0],{NumX,NumY},"x");
        cnpy::npy_save(fileName+"_label.npy",&trainDataLabel[0],{NumX},"y");
    }

    void  RobotA1::RecordData(int beginId)
    {   
        // srand(0);
        // create random data
        // float curTime = locomotionController->GetTime();
        float motorAs[12];
        float motorVs[12];
        Eigen::Map<Eigen::MatrixXf>(&motorAs[0], 12, 1) = motorAngles;
        Eigen::Map<Eigen::MatrixXf>(&motorVs[0], 12, 1) = motorVelocities; 
        trainData.insert(trainData.begin()+ NumY*beginId, std::begin(motorAs), std::end(motorAs));       
        trainData.insert(trainData.begin()+ NumY*beginId+12, std::begin(motorVs), std::end(motorVs));
       
        float a[3];
        Eigen::Map<Eigen::MatrixXf>(&a[0], 3, 1) = baseLinearAcceleration;
        trainData.insert(trainData.begin() + NumY*beginId+24, std::begin(a), std::end(a));     
       
        float w[3];
        Eigen::Map<Eigen::MatrixXf>(&w[0], 3, 1) = baseRollPitchYawRate;
        trainData.insert(trainData.begin() + NumY*beginId+27, std::begin(w), std::end(w));
        
        float pF[12];
        Eigen::Map<Eigen::MatrixXf>(&pF[0], 3, 4) = GetFootPositionsInBaseFrame();
        trainData.insert(trainData.begin() + NumY*beginId+30, std::begin(pF), std::end(pF));     
        
        float vF[3];      
        for (int legId=0; legId<NumLeg; legId++) {
            auto Ji = ComputeJacobian(legId);
            Eigen::Map<Eigen::MatrixXf>(&vF[0], 3, 1) = Ji * motorVelocities.segment(legId * 3, 3);
            trainData.insert(trainData.begin() + NumY*beginId+42 + 3*legId, std::begin(vF), std::end(vF));
        }
     
        uint8_t label = 0;
        for (int legId=3; legId>=0; legId--) {
            if (footContact[legId]) {
                uint8_t tmp = 1 << (3-legId);
                label += tmp;
            }
        }
        trainDataLabel[beginId] = label;
        
        //load it into a new array
        // cnpy::NpyArray arr = cnpy::npy_load("data1.npy");
        // float* loaded_data = arr.data<float>();
        // //make sure the loaded data matches the saved data
        // assert(arr.word_size == sizeof(float));
        // assert(arr.shape.size() == 2 && arr.shape[0] == Nx && arr.shape[1] == Ny);
        // for(int i = 0; i < Nx*Ny*Nz;i++) {
        //     assert(data[i] == loaded_data[i]);
        // }
        // return data;
    }


    void RobotA1::ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode)
    {
        // std::cout << "apply:\n" << motorCommands << std::endl;
        std::array<float, 60> motorCommandsArray = {0};
        if (motorControlMode == POSITION_MODE) {
            Eigen::Matrix<float, 1, 12> motorCommandsShaped = motorCommands.transpose();
            for (int motorId = 0; motorId < numMotors; motorId++) {
                motorCommandsArray[motorId * 5] = motorCommandsShaped[motorId];
                motorCommandsArray[motorId * 5 + 1] = motorKps[motorId];
                motorCommandsArray[motorId * 5 + 2] = 0;
                motorCommandsArray[motorId * 5 + 3] = motorKds[motorId];
                motorCommandsArray[motorId * 5 + 4] = 0;
            }
        } else if (motorControlMode == TORQUE_MODE) {
            Eigen::Matrix<float, 1, 12> motorCommandsShaped = motorCommands.transpose();
            for (int motorId = 0; motorId < numMotors; motorId++) {
                motorCommandsArray[motorId * 5] = 0;
                motorCommandsArray[motorId * 5 + 1] = 0;
                motorCommandsArray[motorId * 5 + 2] = 0;
                motorCommandsArray[motorId * 5 + 3] = 0;
                motorCommandsArray[motorId * 5 + 4] = motorCommandsShaped[motorId];
            }
        } else if (motorControlMode == HYBRID_MODE) {
            Eigen::Matrix<float, 5, 12> motorCommandsShaped = motorCommands;
            for (int motorId = 0; motorId < numMotors; motorId++) {
                motorCommandsArray[motorId * 5] = motorCommandsShaped(POSITION, motorId);
                motorCommandsArray[motorId * 5 + 1] = motorCommandsShaped(KP, motorId);
                motorCommandsArray[motorId * 5 + 2] = motorCommandsShaped(VELOCITY, motorId);
                motorCommandsArray[motorId * 5 + 3] = motorCommandsShaped(KD, motorId);
                motorCommandsArray[motorId * 5 + 4] = motorCommandsShaped(TORQUE, motorId);
            }
        } 

        // std::cout << "motorCommandsArray:\n" << std::endl;
        // for (int index = 0; index < motorCommandsArray.size(); index++) {
        //     std::cout << motorCommandsArray[index] << " ";
        // }
        // std::cout << std::endl;
        robotInterface.SendCommand(motorCommandsArray);
    }

    void RobotA1::ApplyAction(const std::vector<MotorCommand> &motorCommands, MotorMode motorControlMode)
    {
        std::array<float, 60> motorCommandsArray = {0};
        for (int motorId = 0; motorId < numMotors; motorId++) {
            motorCommandsArray[motorId * 5] = motorCommands[motorId].p;
            motorCommandsArray[motorId * 5 + 1] = motorCommands[motorId].Kp;
            motorCommandsArray[motorId * 5 + 2] = motorCommands[motorId].d;
            motorCommandsArray[motorId * 5 + 3] = motorCommands[motorId].Kd;
            motorCommandsArray[motorId * 5 + 4] = motorCommands[motorId].tua;
        }
        robotInterface.SendCommand(motorCommandsArray);
    }

    void RobotA1::Step(const Eigen::MatrixXf &action, MotorMode motorControlMode)
    {
        ReceiveObservation();
        ApplyAction(action, motorControlMode);
    }
} // namespace Quadruped
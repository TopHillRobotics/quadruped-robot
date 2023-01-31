/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: inherited from quadruped robot, name as A1.
* Author: Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhao Yao
*/

#include "robots/robot_a1.h"
using std::cout;
using std::endl;

namespace Quadruped {
    RobotA1::RobotA1(std::string configFilePath)
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
    
        // for (int i=0; i < 9; ++i) {
        //     std::cout << inertias[0][i] << std::endl;
        // }
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

        controlParams["mode"] = robotConfig["controller_params"]["mode"].as<int>();
        Reset(); // reset com_offset

        // timeStep = 0.001;
        // timeStep = 1.0 / robotConfig["controller_params"]["freq"].as<int>();
        
        ResetTimer();
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

        std::array<float, 3> rpy;
        std::copy(std::begin(state.imu.rpy), std::end(state.imu.rpy), std::begin(rpy));
        std::array<float, 3> gyro;
        std::copy(std::begin(state.imu.gyroscope), std::end(state.imu.gyroscope), std::begin(gyro));
        std::array<float, 3> acc;
        std::copy(std::begin(state.imu.accelerometer), std::end(state.imu.accelerometer), std::begin(acc));
        
        stateDataFlow.baseLinearAcceleration << acc[0], acc[1], acc[2];
        stateDataFlow.baseLinearAcceleration = accFilter.CalculateAverage(stateDataFlow.baseLinearAcceleration);
        // stateDataFlow.baseLinearAcceleration << acc[0], acc[1], acc[2];
        // std::cout << "[REVE]: RPY = " << rpy[0] << ", "<< rpy[1] << ", " << rpy[2] << std::endl;
        // std::cout << "[REVE]: Acc = " << acc[0] << ", "<< acc[1] << ", " << acc[2] << std::endl;
        float calibratedYaw = rpy[2] - yawOffset;
        if (calibratedYaw >= M_PI)
            calibratedYaw -= M_2PI;
        else if (calibratedYaw <= -M_PI)
            calibratedYaw += M_2PI;
        float groundPitch = robotics::math::quatToRPY(stateDataFlow.groundOrientation)[1];
        
        baseRollPitchYaw << rpy[0], rpy[1], calibratedYaw;
        baseOrientation = robotics::math::rpyToQuat(baseRollPitchYaw);

        Vec3<float> gyroVec(gyro[0], gyro[1], gyro[2]);
        baseRollPitchYawRate = gyroFilter.CalculateAverage(gyroVec);
        // baseRollPitchYawRate << gyro[0], gyro[1], gyro[2];

        for (int motorId = 0; motorId < NumMotor; motorId++) {
            motorAngles[motorId] = state.motorState[motorId].q;
            motorVelocities[motorId] = state.motorState[motorId].dq;
            motorddq[motorId] = state.motorState[motorId].ddq;
            motortorque[motorId] = state.motorState[motorId].tauEst;
        }
        // stateDataFlow.visualizer.datax.push_back(basePosition[0]);
        
        // stateDataFlow.visualizer.datay1.push_back(basePosition[1]);
        // stateDataFlow.visualizer.datay2.push_back(basePosition[2]);
        // stateDataFlow.visualizer.datay3.push_back(baseVelocityInBaseFrame[0]);
        // stateDataFlow.visualizer.datay4.push_back(baseVelocityInBaseFrame[1]);
        // stateDataFlow.visualizer.datay5.push_back(baseRollPitchYawRate[2]);
    
        motorVelocities = motorVFilter.CalculateAverage(motorVelocities);

        std::array<int16_t, 4> force;
        std::copy(std::begin(state.footForce), std::end(state.footForce), std::begin(force));
        footForce << force[0], force[1], force[2], force[3];
        // std::cout << "[reve] footForce = " << footForce.transpose() << std::endl;
        // std::cout << "[reve] motorAngles = " << motorAngles.transpose() << std::endl;
        // std::cout << "[reve] motorVelocities = " << motorVelocities.transpose() << std::endl;
        for (int footId = 0; footId < NumLeg; footId++) {
            // 20 is contact threshold
            if (footForce[footId] >= 15) { // 20
                footContact[footId] = true;
            } else {
                footContact[footId] = false;
            }
        }
        
 
        UpdateDataFlow();
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
        Eigen::Map<Eigen::MatrixXf>(&a[0], 3, 1) = stateDataFlow.baseLinearAcceleration;
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
        for (int motorId = 0; motorId < NumMotor; motorId++) {
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


    bool RobotA1::BuildDynamicModel()
    {
        // we assume the cheetah's body (not including rotors) can be modeled as a
        // uniformly distributed box.
        std::vector<float> bodySize = robotConfig["robot_params"]["body_size"].as<std::vector<float>>(); // Length, Width, Height
        Vec3<float> bodyDims(bodySize[0], bodySize[1], bodySize[2]);
        
        // locations
        Vec3<float> _abadRotorLocation = {0.14f, 0.047f, 0.f}; // a1
        Vec3<float> _abadLocation = abadLocation;        
        Vec3<float> _hipLocation = Vec3<float>(0, hipLength, 0);
        Vec3<float> _hipRotorLocation = Vec3<float>(0, 0.04, 0);
        Vec3<float> _kneeLocation = Vec3<float>(0, 0, -upperLegLength);
        Vec3<float> _kneeRotorLocation = Vec3<float>(0, 0, 0);

        float scale_ = 1e-2;
        // rotor inertia if the rotor is oriented so it spins around the z-axis
        Mat3<float> rotorRotationalInertiaZ;
        rotorRotationalInertiaZ << 33, 0, 0,
                                   0, 33, 0,
                                   0, 0, 63;
        rotorRotationalInertiaZ.setIdentity();
        rotorRotationalInertiaZ = scale_*1e-6 * rotorRotationalInertiaZ;

        Mat3<float> RY = coordinateRotation<float>(CoordinateAxis::Y, M_PI / 2);
        Mat3<float> RX = coordinateRotation<float>(CoordinateAxis::X, M_PI / 2);
        Mat3<float> rotorRotationalInertiaX = RY * rotorRotationalInertiaZ * RY.transpose();
        Mat3<float> rotorRotationalInertiaY = RX * rotorRotationalInertiaZ * RX.transpose();

        // spatial inertias
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
        
        // std::cout << "hipInertia -----" <<std::endl;
        // std::cout << hipInertia.getInertiaTensor() << std::endl;
        // std::cout << hipInertia.flipAlongAxis(CoordinateAxis::Y).getInertiaTensor() << std::endl;
        // std::cout << "----- hipInertia " <<std::endl;
        
        Mat3<float> kneeRotationalInertia, kneeRotationalInertiaRotated;
        kneeRotationalInertiaRotated = linkInertias[2];
        kneeRotationalInertia = kneeRotationalInertiaRotated;//RY * kneeRotationalInertiaRotated * RY.transpose();
        
        Vec3<float> kneeCOM(linksComPos[2][0],linksComPos[2][1],linksComPos[2][2]);
        SpatialInertia<float> kneeInertia(linkMasses[2], kneeCOM, kneeRotationalInertia);
        std::cout << "linkMasses[2]=" << linkMasses[2] << std::endl;
        std::cout << "kneeRotationalInertia=" << kneeRotationalInertia << std::endl;
        

        Vec3<float> rotorCOM(0, 0, 0);
        float rotorMass = 1e-8; //0.055
        SpatialInertia<float> rotorInertiaX(rotorMass, rotorCOM, rotorRotationalInertiaX);
        SpatialInertia<float> rotorInertiaY(rotorMass, rotorCOM, rotorRotationalInertiaY);

        Mat3<float> bodyRotationalInertia;
        bodyRotationalInertia = bodyInertia;
        // Vec3<float> bodyCOM(0.01, 0, 0.00);
        Vec3<float> bodyCOM = comOffset;
        // (0.02, 0.0, 0.04);
        // (0.03, 0.0, 0.04);
        
        std::cout << "totalMass = " << totalMass << ", bodyMass = " << bodyMass << std::endl;
        SpatialInertia<float> bodyInertia_(bodyMass, bodyCOM, bodyRotationalInertia);
        
        model.addBase(bodyInertia_);
        // add contact for the cheetah's body
        model.addGroundContactBoxPoints(5, bodyDims);
        
        const int baseID = 5;
        int bodyID = baseID;
        float sideSign = -1;

        Mat3<float> I3 = Mat3<float>::Identity();

        auto& abadRotorInertia = rotorInertiaX;
        float abadGearRatio = 1; // 6
        auto& hipRotorInertia = rotorInertiaY;
        float hipGearRatio = 1; // 6
        auto& kneeRotorInertia = rotorInertiaY;
        float kneeGearRatio = 1; // 9.33
        float kneeLinkY_offset = 0.004;

        // loop over 4 legs
        for (int legID = 0; legID < 4; legID++) {
            // Ab/Ad joint
            //  int addBody(const SpatialInertia<T>& inertia, const SpatialInertia<T>& rotorInertia, T gearRatio,
                //  int parent, JointType jointType, CoordinateAxis jointAxis,
                //  const Mat6<T>& Xtree, const Mat6<T>& Xrot);
            bodyID++;

            Vec3<float> offsetZ(0,0,0);
            // if (legID>1) offsetZ << 0,0,+0.02; //todo
            
            Mat6<float> xtreeAbad = createSXform(I3, withLegSigns(_abadLocation+offsetZ, legID));
            Mat6<float> xtreeAbadRotor = createSXform(I3, withLegSigns(_abadRotorLocation, legID));
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
                            withLegSigns(_hipLocation, legID)); // 0, hipLength=0.085, 0
            Mat6<float> xtreeHipRotor =
                createSXform(coordinateRotation(CoordinateAxis::Z, float(M_PI)),
                            withLegSigns(_hipRotorLocation, legID));
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

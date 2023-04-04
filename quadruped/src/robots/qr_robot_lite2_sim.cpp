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

#include "robots/qr_robot_lite2_sim.h"


namespace Quadruped {

qrRobotLite2Sim::qrRobotLite2Sim(ros::NodeHandle &nhIn, ros::NodeHandle &privateNhIn, std::string configFilePath):
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
    totalMass = robotConfig["robot_params"]["total_mass"].as<float>();
    bodyMass = robotConfig["robot_params"]["body_mass"].as<float>();

    std::vector<float> totalInertiaVec = robotConfig["robot_params"]["total_inertia"].as<std::vector<float >>();
    totalInertia = Eigen::MatrixXf::Map(&totalInertiaVec[0], 3, 3);
    std::vector<float> bodyInertiaVec = robotConfig["robot_params"]["body_inertia"].as<std::vector<float >>();
    bodyInertia = Eigen::MatrixXf::Map(&bodyInertiaVec[0], 3, 3);

    std::vector<std::vector<float>> inertias = robotConfig["robot_params"]["links_inertia"].as<std::vector<std::vector<float>>>();
    std::vector<float> masses = robotConfig["robot_params"]["links_mass"].as<std::vector<float>>();
    std::vector<std::vector<float>> linksComPos_ = robotConfig["robot_params"]["links_com_pos"].as<std::vector<std::vector<float>>>();

    for (int i=0; i < 9; ++i) {
        std::cout << inertias[0][i] << std::endl;
    }

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
    Eigen::Matrix<float, 3, 1> defaultStandUpAngle(0.f, 0.82f, -1.5f);

    standUpMotorAngles << defaultStandUpAngle, defaultStandUpAngle, defaultStandUpAngle, defaultStandUpAngle;

    float sitDownAbAngle, sitDownHipAngle, sitDownKneeAngle;
    sitDownAbAngle = robotConfig["robot_params"]["default_sitdown_angle"]["ab"].as<float>();
    sitDownHipAngle = robotConfig["robot_params"]["default_sitdown_angle"]["hip"].as<float>();
    sitDownKneeAngle = robotConfig["robot_params"]["default_sitdown_angle"]["knee"].as<float>();
    Eigen::Matrix<float, 3, 1> defaultSitDownAngle(sitDownAbAngle, sitDownHipAngle, sitDownKneeAngle);
    sitDownMotorAngles << defaultSitDownAngle, defaultSitDownAngle, defaultSitDownAngle, defaultSitDownAngle;

    controlParams["mode"] = robotConfig["controller_params"]["mode"].as<int>();
    Reset(); // reset com_offset

    imuSub = nh.subscribe("/trunk_imu", 1, &qrRobotLite2Sim::ImuCallback, this);
    jointStateSub[0] = nh.subscribe("lite2_gazebo/FR_hip_controller/state", 1, &qrRobotLite2Sim::FRhipCallback, this);
    jointStateSub[1] = nh.subscribe("lite2_gazebo/FR_thigh_controller/state", 1, &qrRobotLite2Sim::FRthighCallback, this);
    jointStateSub[2] = nh.subscribe("lite2_gazebo/FR_calf_controller/state", 1, &qrRobotLite2Sim::FRcalfCallback, this);
    jointStateSub[3] = nh.subscribe("lite2_gazebo/FL_hip_controller/state", 1, &qrRobotLite2Sim::FLhipCallback, this);
    jointStateSub[4] = nh.subscribe("lite2_gazebo/FL_thigh_controller/state", 1, &qrRobotLite2Sim::FLthighCallback, this);
    jointStateSub[5] = nh.subscribe("lite2_gazebo/FL_calf_controller/state", 1, &qrRobotLite2Sim::FLcalfCallback, this);
    jointStateSub[6] = nh.subscribe("lite2_gazebo/RR_hip_controller/state", 1, &qrRobotLite2Sim::RRhipCallback, this);
    jointStateSub[7] = nh.subscribe("lite2_gazebo/RR_thigh_controller/state", 1, &qrRobotLite2Sim::RRthighCallback, this);
    jointStateSub[8] = nh.subscribe("lite2_gazebo/RR_calf_controller/state", 1, &qrRobotLite2Sim::RRcalfCallback, this);
    jointStateSub[9] = nh.subscribe("lite2_gazebo/RL_hip_controller/state", 1, &qrRobotLite2Sim::RLhipCallback, this);
    jointStateSub[10] = nh.subscribe("lite2_gazebo/RL_thigh_controller/state", 1, &qrRobotLite2Sim::RLthighCallback, this);
    jointStateSub[11] = nh.subscribe("lite2_gazebo/RL_calf_controller/state", 1, &qrRobotLite2Sim::RLcalfCallback, this);
    footForceSub[0] = nh.subscribe("/visual/FR_foot_contact/the_force", 1, &qrRobotLite2Sim::FRfootCallback, this);
    footForceSub[1] = nh.subscribe("/visual/FL_foot_contact/the_force", 1, &qrRobotLite2Sim::FLfootCallback, this);
    footForceSub[2] = nh.subscribe("/visual/RR_foot_contact/the_force", 1, &qrRobotLite2Sim::RRfootCallback, this);
    footForceSub[3] = nh.subscribe("/visual/RL_foot_contact/the_force", 1, &qrRobotLite2Sim::RLfootCallback, this);

    jointCmdPub[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("lite2_gazebo/FR_hip_controller/command", 1);
    jointCmdPub[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("lite2_gazebo/FR_thigh_controller/command", 1);
    jointCmdPub[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("lite2_gazebo/FR_calf_controller/command", 1);
    jointCmdPub[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("lite2_gazebo/FL_hip_controller/command", 1);
    jointCmdPub[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("lite2_gazebo/FL_thigh_controller/command", 1);
    jointCmdPub[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("lite2_gazebo/FL_calf_controller/command", 1);
    jointCmdPub[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("lite2_gazebo/RR_hip_controller/command", 1);
    jointCmdPub[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("lite2_gazebo/RR_thigh_controller/command", 1);
    jointCmdPub[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("lite2_gazebo/RR_calf_controller/command", 1);
    jointCmdPub[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("lite2_gazebo/RL_hip_controller/command", 1);
    jointCmdPub[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("lite2_gazebo/RL_thigh_controller/command", 1);
    jointCmdPub[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("lite2_gazebo/RL_calf_controller/command", 1);

    // ros::spinOnce();
    usleep(300000); // must wait 300ms, to get first state

    yawOffset = 0;//lowState.imu.rpy[2]; // todo
    std::cout << "yawOffset: " << yawOffset << std::endl;

    // timeStep = 1.0 / robotConfig["controller_params"]["freq"].as<int>();
    this->ResetTimer();
    lastResetTime = GetTimeSinceReset();
    initComplete = true;
    std::cout << "-------Lite2Sim init Complete-------" << std::endl;
}


bool qrRobotLite2Sim::BuildDynamicModel()
{
    // we assume the cheetah's body (not including rotors) can be modeled as a
    // uniformly distributed box.
    std::vector<float> bodySize = robotConfig["robot_params"]["body_size"].as<std::vector<float>>(); // Length, Width, Height
    Vec3<float> bodyDims(bodySize[0], bodySize[1], bodySize[2]);

    // locations
    Vec3<float> _abadRotorLocation = {0.14f, 0.047f, 0.f};
    Vec3<float> _abadLocation = {0.1805f, 0.047f, 0.f};
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
    Mat3<float> abadRotationalInertia;
    abadRotationalInertia << 469.2, -9.4, -0.342,
                             -9.4, 807.5, -0.466,
                             -0.342, -0.466,  552.9;
    // abadRotationalInertia.setIdentity();
    abadRotationalInertia = abadRotationalInertia * 1e-6;
    // Vec3<float> abadCOM(0, 0.036, 0);  // mini-cheetah
    Vec3<float> abadCOM(-0.0033, 0, 0);
    SpatialInertia<float> abadInertia(0.696, abadCOM, abadRotationalInertia);

    Mat3<float> hipRotationalInertia;
    hipRotationalInertia << 5529, 4.825, 343.9,
                            4.825, 5139.3, 22.4,
                            343.9, 22.4, 1367.8;
    // hipRotationalInertia.setIdentity();
    hipRotationalInertia = hipRotationalInertia * 1e-6;
    // Vec3<float> hipCOM(0, 0.016, -0.02);
    Vec3<float> hipCOM(-0.003237, -0.022327, -0.027326); // left, for right filp y-axis value.
    SpatialInertia<float> hipInertia(1.013, hipCOM, hipRotationalInertia);
    std::cout << "hipInertia -----" <<std::endl;
    std::cout << hipInertia.getInertiaTensor() << std::endl;
    std::cout << hipInertia.flipAlongAxis(CoordinateAxis::Y).getInertiaTensor() << std::endl;
    std::cout << "----- hipInertia " <<std::endl;


    Mat3<float> kneeRotationalInertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated << 2998, 0,   -141.2,
                                    0,    3014, 0,
                                    -141.2, 0,   32.4;
    // kneeRotationalInertiaRotated.setIdentity();
    kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
    kneeRotationalInertia = kneeRotationalInertiaRotated;//RY * kneeRotationalInertiaRotated * RY.transpose();
    // Vec3<float> kneeCOM(0, 0, -0.061);
    Vec3<float> kneeCOM(0.006435, 0, -0.107);
    SpatialInertia<float> kneeInertia(0.166, kneeCOM, kneeRotationalInertia);

    Vec3<float> rotorCOM(0, 0, 0);
    float rotorMass = 1e-8; //0.055
    SpatialInertia<float> rotorInertiaX(rotorMass, rotorCOM, rotorRotationalInertiaX);
    SpatialInertia<float> rotorInertiaY(rotorMass, rotorCOM, rotorRotationalInertiaY);

    Mat3<float> bodyRotationalInertia;
    bodyRotationalInertia << 15853, 0, 0,
                             0, 37799, 0,
                             0, 0, 45654;
    bodyRotationalInertia = bodyRotationalInertia * 1e-6;
    Vec3<float> bodyCOM(0, 0, 0);
    // Vec3<float> bodyCOM(0, 0.004, -0.0005);
    SpatialInertia<float> bodyInertia(6, bodyCOM, bodyRotationalInertia);

    model.addBase(bodyInertia);
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
        //  int addBody(const SpatialInertia<T>& inertia, const SpatialInertia<T>&
        //  rotorInertia, T gearRatio,
        //              int parent, JointType jointType, CoordinateAxis jointAxis,
        //              const Mat6<T>& Xtree, const Mat6<T>& Xrot);
        bodyID++;
        Mat6<float> xtreeAbad = createSXform(I3, WithLegSigns(_abadLocation, legID));
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
                          kneeRotorInertia.flipAlongAxis(CoordinateAxis::Y),
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

    bool test_fb = false;
    if (test_fb) {
        FBModelState<float> fb;
        // for (size_t i(0); i < 3; ++i) {
        // _state.bodyVelocity[i] = omegaBody[i]; // in body frame
        // _state.bodyVelocity[i + 3] = vBody[i];

        //     for (size_t leg(0); leg < 4; ++leg) {
        //         _state.q[3 * leg + i] = q[3 * leg + i];
        //         _state.qd[3 * leg + i] = dq[3 * leg + i];
        //         _full_config[3 * leg + i + 6] = _state.q[3 * leg + i];
        //     }
        // }
        printf("339\n");
        fb.bodyOrientation << 1,0,0,0;//0.896127, 0.365452,0.246447,-0.0516205;
        // fb.bodyPosition << 0.00437649, 0.000217693, 0.285963;
        fb.bodyVelocity <<  3,3,3, 0.2, 0.1, 0.1;
        fb.bodyPosition.setZero();
        printf("343\n");
        fb.q.resize(12,1);
        fb.q.setZero();
        fb.q << 0.2, 0, -0.2,
                0.2, 0, -0.2, // 0, 0.7, 0,
                0, 0.3, 0.5, // 0, 0.8, 0
                0, 0.3, 0.5,
        fb.qd.resize(12, 1);
        fb.qd.setZero();
        // fb.qd << 0.2, 0, 0,
        //          0.1, 0, 0.,
        //          0, -0.3, 0.6,
        //          0, 0.3, 1;

        printf("346\n");
        model.setState(fb);
        printf("348\n");
        model.forwardKinematics();
        model.massMatrix();
        Eigen::MatrixXf A;
        Eigen::Matrix<float,18,1> dq;
        dq << fb.bodyVelocity, fb.qd;
        // model.generalizedGravityForce();
        // model.generalizedCoriolisForce();
        A = model.getMassMatrix();
        for(int i=0; i <18 ; ++i) {
            for (int j=0; j<18; ++j) {
                if (A(i,j)<1e-6) A(i,j) = 0;
            }
        }
        std::cout << "A = \n" << A << std::endl;
        float energy = 0.5*dq.dot(A*dq);
        std::cout << "energy = " << energy << std::endl;

        // model.getPosition(8);
        printf("351\n");
        throw std::domain_error("finished!!!!");
    }
    return true;
}


void qrRobotLite2Sim::ImuCallback(const sensor_msgs::Imu &msg)
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


void qrRobotLite2Sim::FRhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[0].mode = msg.mode;
    lowState.motorState[0].q = msg.q;
    lowState.motorState[0].dq = msg.dq;
    lowState.motorState[0].ddq = msg.ddq;
    lowState.motorState[0].tauEst = msg.tauEst;
}


void qrRobotLite2Sim::FRthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[1].mode = msg.mode;
    lowState.motorState[1].q = msg.q;
    lowState.motorState[1].dq = msg.dq;
    lowState.motorState[1].ddq = msg.ddq;
    lowState.motorState[1].tauEst = msg.tauEst;
}


void qrRobotLite2Sim::FRcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[2].mode = msg.mode;
    lowState.motorState[2].q = msg.q;
    lowState.motorState[2].dq = msg.dq;
    lowState.motorState[2].ddq = msg.ddq;
    lowState.motorState[2].tauEst = msg.tauEst;
}


void qrRobotLite2Sim::FLhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[3].mode = msg.mode;
    lowState.motorState[3].q = msg.q;
    lowState.motorState[3].dq = msg.dq;
    lowState.motorState[3].ddq = msg.ddq;
    lowState.motorState[3].tauEst = msg.tauEst;
}


void qrRobotLite2Sim::FLthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[4].mode = msg.mode;
    lowState.motorState[4].q = msg.q;
    lowState.motorState[4].dq = msg.dq;
    lowState.motorState[4].ddq = msg.ddq;
    lowState.motorState[4].tauEst = msg.tauEst;
}


void qrRobotLite2Sim::FLcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[5].mode = msg.mode;
    lowState.motorState[5].q = msg.q;
    lowState.motorState[5].dq = msg.dq;
    lowState.motorState[5].ddq = msg.ddq;
    lowState.motorState[5].tauEst = msg.tauEst;
}


void qrRobotLite2Sim::RRhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[6].mode = msg.mode;
    lowState.motorState[6].q = msg.q;
    lowState.motorState[6].dq = msg.dq;
    lowState.motorState[6].ddq = msg.ddq;
    lowState.motorState[6].tauEst = msg.tauEst;
}


void qrRobotLite2Sim::RRthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[7].mode = msg.mode;
    lowState.motorState[7].q = msg.q;
    lowState.motorState[7].dq = msg.dq;
    lowState.motorState[7].ddq = msg.ddq;
    lowState.motorState[7].tauEst = msg.tauEst;
}


void qrRobotLite2Sim::RRcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[8].mode = msg.mode;
    lowState.motorState[8].q = msg.q;
    lowState.motorState[8].dq = msg.dq;
    lowState.motorState[8].ddq = msg.ddq;
    lowState.motorState[8].tauEst = msg.tauEst;
}


void qrRobotLite2Sim::RLhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[9].mode = msg.mode;
    lowState.motorState[9].q = msg.q;
    lowState.motorState[9].dq = msg.dq;
    lowState.motorState[9].ddq = msg.ddq;
    lowState.motorState[9].tauEst = msg.tauEst;
}


void qrRobotLite2Sim::RLthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[10].mode = msg.mode;
    lowState.motorState[10].q = msg.q;
    lowState.motorState[10].dq = msg.dq;
    lowState.motorState[10].ddq = msg.ddq;
    lowState.motorState[10].tauEst = msg.tauEst;
}


void qrRobotLite2Sim::RLcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[11].mode = msg.mode;
    lowState.motorState[11].q = msg.q;
    lowState.motorState[11].dq = msg.dq;
    lowState.motorState[11].ddq = msg.ddq;
    lowState.motorState[11].tauEst = msg.tauEst;
}


void qrRobotLite2Sim::FRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    lowState.eeForce[0].x = msg.wrench.force.x;
    lowState.eeForce[0].y = msg.wrench.force.y;
    lowState.eeForce[0].z = msg.wrench.force.z;
    lowState.footForce[0] = msg.wrench.force.z;
}


void qrRobotLite2Sim::FLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    lowState.eeForce[1].x = msg.wrench.force.x;
    lowState.eeForce[1].y = msg.wrench.force.y;
    lowState.eeForce[1].z = msg.wrench.force.z;
    lowState.footForce[1] = msg.wrench.force.z;
}


void qrRobotLite2Sim::RRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    lowState.eeForce[2].x = msg.wrench.force.x;
    lowState.eeForce[2].y = msg.wrench.force.y;
    lowState.eeForce[2].z = msg.wrench.force.z;
    lowState.footForce[2] = msg.wrench.force.z;
}


void qrRobotLite2Sim::RLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    lowState.eeForce[3].x = msg.wrench.force.x;
    lowState.eeForce[3].y = msg.wrench.force.y;
    lowState.eeForce[3].z = msg.wrench.force.z;
    lowState.footForce[3] = msg.wrench.force.z;
}


void qrRobotLite2Sim::SendCommand(const std::array<float, 60> motorcmd)
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


void qrRobotLite2Sim::ReceiveObservation()
{
    // ros::spinOnce();
    // usleep(1000);
    unitree_legged_msgs::LowState state = lowState;
    
    // tick = state.tick;
    // std::array<float, 3> acc;
    // std::copy(std::begin(state.imu.accelerometer), std::end(state.imu.accelerometer), std::begin(acc));
    baseAccInBaseFrame << state.imu.accelerometer[0],
                          state.imu.accelerometer[1],
                          state.imu.accelerometer[2];
    stateDataFlow.baseLinearAcceleration = accFilter.CalculateAverage(baseAccInBaseFrame);
    
    std::array<float, 3> rpy;
    std::copy(std::begin(state.imu.rpy), std::end(state.imu.rpy), std::begin(rpy));
    // calibrated
    float calibratedYaw = rpy[2] - yawOffset;
    if (calibratedYaw >= M_PI) {
        calibratedYaw -= M_2PI;
    } else if (calibratedYaw <= -M_PI) {
        calibratedYaw += M_2PI;
    }
    // baseRollPitchYaw << rpy[0], rpy[1], calibratedYaw;
    if (abs(baseRollPitchYaw[2]-calibratedYaw) >= M_PI) {
        rpyFilter.Reset();
    }

    Vec3<float> rpyVec(rpy[0], rpy[1], calibratedYaw);
    baseRollPitchYaw = rpyFilter.CalculateAverage(rpyVec);
    if (baseRollPitchYaw[2] >= M_PI) {
        baseRollPitchYaw[2] -= M_2PI;
    } else if (baseRollPitchYaw[2] <= -M_PI) {
        baseRollPitchYaw[2] += M_2PI;
    }
    
    std::array<float, 3> gyro;
    std::copy(std::begin(state.imu.gyroscope), std::end(state.imu.gyroscope), std::begin(gyro));
    Vec3<float> gyroVec(gyro[0], gyro[1], gyro[2]);
    baseRollPitchYawRate = gyroFilter.CalculateAverage(gyroVec);

    // baseRollPitchYaw << rpyVec[0], rpyVec[1], calibratedYaw;
    baseOrientation = robotics::math::rpyToQuat(baseRollPitchYaw);
    // baseRollPitchYawRate << gyro[0], gyro[1], gyro[2];

    for (int motorId = 0; motorId < NumMotor; ++motorId) {
        motorAngles[motorId] = state.motorState[motorId].q;
        motorVelocities[motorId] = state.motorState[motorId].dq;
        motorddq[motorId] = state.motorState[motorId].ddq;
        motortorque[motorId] = state.motorState[motorId].tauEst;
    }
    motorAngles = jointDirection.cwiseProduct(motorAngles + jointOffset);
    motorVelocities = jointDirection.cwiseProduct(motorVelocities);
    motorddq = jointDirection.cwiseProduct(motorddq);
    motortorque = jointDirection.cwiseProduct(motortorque);

    boost::array<int16_t, 4> force = state.footForce;
    footForce << force[0], force[1], force[2], force[3];

    for (int footId = 0; footId < NumLeg; footId++) {
        if (footForce[footId] >= 5) {
            footContact[footId] = true;
        } else {
            footContact[footId] = false;
        }
    }

    UpdateDataFlow();
}


void qrRobotLite2Sim::ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode)
{
    std::array<float, 60> motorCommandsArray = {0};
    if (motorControlMode == POSITION_MODE) {
        Eigen::Matrix<float, 12, 1> motorCommandsShaped = motorCommands;
        motorCommandsShaped = jointDirection.cwiseProduct(motorCommandsShaped) - jointOffset;
        for (int motorId = 0; motorId < NumMotor; ++motorId) {
            motorCommandsArray[motorId * 5] = motorCommandsShaped[motorId];
            motorCommandsArray[motorId * 5 + 1] = motorKps[motorId];
            motorCommandsArray[motorId * 5 + 2] = 0;
            motorCommandsArray[motorId * 5 + 3] = motorKds[motorId];
            motorCommandsArray[motorId * 5 + 4] = 0;
        }
    } else if (motorControlMode == TORQUE_MODE) {
        Eigen::Matrix<float, 12, 1> motorCommandsShaped = motorCommands;
        motorCommandsShaped = jointDirection.cwiseProduct(motorCommandsShaped);
        for (int motorId = 0; motorId < NumMotor; ++motorId) {
            motorCommandsArray[motorId * 5] = 0;
            motorCommandsArray[motorId * 5 + 1] = 0;
            motorCommandsArray[motorId * 5 + 2] = 0;
            motorCommandsArray[motorId * 5 + 3] = 0;
            motorCommandsArray[motorId * 5 + 4] = motorCommandsShaped[motorId];
        }
    } else if (motorControlMode == HYBRID_MODE) {
        Eigen::Matrix<float, 5, 12> motorCommandsShaped = motorCommands;
        Eigen::Matrix<float, 12, 1> angles = motorCommandsShaped.row(POSITION).transpose();
        motorCommandsShaped.row(POSITION) = (jointDirection.cwiseProduct(angles) - jointOffset).transpose();
        Eigen::Matrix<float, 12, 1> vels = motorCommandsShaped.row(VELOCITY).transpose();
        motorCommandsShaped.row(VELOCITY) = jointDirection.cwiseProduct(vels).transpose();
        Eigen::Matrix<float, 12, 1> tuas = motorCommandsShaped.row(TORQUE).transpose();
        motorCommandsShaped.row(TORQUE) = jointDirection.cwiseProduct(tuas).transpose();
        
        for (int motorId = 0; motorId < NumMotor; ++motorId) {
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


void qrRobotLite2Sim::ApplyAction(const std::vector<qrMotorCommand> &motorCommands, MotorMode motorControlMode)
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


void qrRobotLite2Sim::Step(const Eigen::MatrixXf &action, MotorMode motorControlMode)
{
    ReceiveObservation();
    ApplyAction(action, motorControlMode);
}

}

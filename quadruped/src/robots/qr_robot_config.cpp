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

#include "robots/qr_robot_config.h"

std::unordered_map<int, std::string> modeMap = {{0, "velocity"}, {1, "position"}, {2, "walk"}, {3, "advanced_trot"}};

qrRobotConfig::qrRobotConfig(std::string path, LocomotionMode mode)
{
    Load(path, mode);
}

void qrRobotConfig::Load(std::string path, LocomotionMode mode)
{
    node = YAML::LoadFile(path);

    bodyMass = node["robot_params"]["body_mass"].as<float>();
    std::vector<float> bodyInertiaList = node["robot_params"]["body_inertia"].as<std::vector<float >>();
    bodyInertia = Eigen::MatrixXf::Map(&bodyInertiaList[0], 3, 3);
    bodyHeight = node["robot_params"]["body_height"].as<float>();

    hipLength = node["robot_params"]["hip_l"].as<float>();
    upperLegLength = node["robot_params"]["upper_l"].as<float>();
    lowerLegLength = node["robot_params"]["lower_l"].as<float>();


    float standUpAbAngle, standUpHipAngle, standUpKneeAngle;
    standUpAbAngle = node["robot_params"]["default_standup_angle"]["ab"].as<float>();
    standUpHipAngle = node["robot_params"]["default_standup_angle"]["hip"].as<float>();
    standUpKneeAngle = node["robot_params"]["default_standup_angle"]["knee"].as<float>();
    Eigen::Matrix<float, 3, 1> defaultStandUpAngle(standUpAbAngle, standUpHipAngle, standUpKneeAngle);
    standUpMotorAngles << defaultStandUpAngle, defaultStandUpAngle, defaultStandUpAngle, defaultStandUpAngle;
        
    float sitDownAbAngle, sitDownHipAngle, sitDownKneeAngle;
    sitDownAbAngle = node["robot_params"]["default_sitdown_angle"]["ab"].as<float>();
    sitDownHipAngle = node["robot_params"]["default_sitdown_angle"]["hip"].as<float>();
    sitDownKneeAngle = node["robot_params"]["default_sitdown_angle"]["knee"].as<float>();
    Eigen::Matrix<float, 3, 1> defaultSitDownAngle(sitDownAbAngle, sitDownHipAngle, sitDownKneeAngle);
    sitDownMotorAngles << defaultSitDownAngle, defaultSitDownAngle, defaultSitDownAngle, defaultSitDownAngle;

    isSim = node["is_sim"].as<bool>();

    LoadComOffset(mode);
    LoadHipOffset();
    LoadHipPosition();
    LoadKps();
    LoadKds();
}

void qrRobotConfig::LoadKps()
{
  float abadKp, hipKp, kneeKp;
  abadKp = node["motor_params"]["abad_p"].as<float>();
    hipKp = node["motor_params"]["hip_p"].as<float>();
    kneeKp = node["motor_params"]["knee_p"].as<float>();
    Eigen::Matrix<float, 3, 1> kps(abadKp, hipKp, kneeKp);
    motorKps << kps, kps, kps, kps;
}

void qrRobotConfig::LoadKds()
{
    float abadKd, hipKd, kneeKd;
    abadKd = node["motor_params"]["abad_d"].as<float>();
    hipKd = node["motor_params"]["hip_d"].as<float>();
    kneeKd = node["motor_params"]["knee_d"].as<float>();
    Eigen::Matrix<float, 3, 1> kds(abadKd, hipKd, kneeKd);
    motorKds << kds, kds, kds, kds;
}

void qrRobotConfig::LoadComOffset(LocomotionMode mode)
{
    std::vector<float> comOffsetList = node["robot_params"][modeMap[mode]]["com_offset"].as<std::vector<float >>();
    comOffset = -Eigen::MatrixXf::Map(&comOffsetList[0], 3, 1);
}

void qrRobotConfig::LoadHipOffset()
{
    std::vector<std::vector<float >> hipOffsetList = node["robot_params"]["hip_offset"].as<std::vector<std::vector<float>>>();
    Eigen::Matrix<float, 3, 1> hipOffsetFR = Eigen::MatrixXf::Map(&hipOffsetList[0][0], 3, 1) + comOffset;
    Eigen::Matrix<float, 3, 1> hipOffsetFL = Eigen::MatrixXf::Map(&hipOffsetList[1][0], 3, 1) + comOffset;
    Eigen::Matrix<float, 3, 1> hipOffsetRL = Eigen::MatrixXf::Map(&hipOffsetList[2][0], 3, 1) + comOffset;
    Eigen::Matrix<float, 3, 1> hipOffsetRR = Eigen::MatrixXf::Map(&hipOffsetList[3][0], 3, 1) + comOffset;
    hipOffset << hipOffsetFR, hipOffsetFL, hipOffsetRL, hipOffsetRR;
}

void qrRobotConfig::LoadHipPosition()
{
    std::vector<std::vector<float>> defaultHipPositionList = node["robot_params"]["default_hip_positions"].as<std::vector<std::vector<float>>>();
    Eigen::Matrix<float, 3, 1> defaultHipPositionFR = Eigen::MatrixXf::Map(&defaultHipPositionList[0][0], 3, 1);
    Eigen::Matrix<float, 3, 1> defaultHipPositionFL = Eigen::MatrixXf::Map(&defaultHipPositionList[1][0], 3, 1);
    Eigen::Matrix<float, 3, 1> defaultHipPositionRL = Eigen::MatrixXf::Map(&defaultHipPositionList[2][0], 3, 1);
    Eigen::Matrix<float, 3, 1> defaultHipPositionRR = Eigen::MatrixXf::Map(&defaultHipPositionList[3][0], 3, 1);
    defaultHipPosition << defaultHipPositionFR, defaultHipPositionFL, defaultHipPositionRL, defaultHipPositionRR;
}

Eigen::Matrix<float, 3, 1> qrRobotConfig::FootPositionInHipFrameToJointAngle(Eigen::Matrix<float, 3, 1> &footPosition, int hipSign)
{
    float signedHipLength = hipLength * hipSign;
    Eigen::Matrix<float, 3, 1> xyz(footPosition[0], footPosition[1], footPosition[2]);
    Eigen::Matrix<float, 3, 1> legLength(signedHipLength, upperLegLength, lowerLegLength);

    float thetaAB, thetaHip, thetaKnee;
    thetaKnee = -acos((xyz.squaredNorm() - legLength.squaredNorm()) / (2 * lowerLegLength * upperLegLength));
    float l = sqrt(upperLegLength * upperLegLength + lowerLegLength * lowerLegLength +
        2 * upperLegLength * lowerLegLength * cos(thetaKnee));
    thetaHip = asin(-xyz.x() / l) - thetaKnee / 2;
    float c1 = signedHipLength * xyz.y() - l * cos(thetaHip + thetaKnee / 2) * xyz.z();
    float s1 = l * cos(thetaHip + thetaKnee / 2) * xyz.y() + signedHipLength * xyz.z();
    thetaAB = atan2(s1, c1);

    return Eigen::Matrix<float, 3, 1>(thetaAB, thetaHip, thetaKnee);
}

Eigen::Matrix<float, 3, 1> qrRobotConfig::FootPositionInHipFrame(Eigen::Matrix<float, 3, 1> &angles, int hipSign)
{
    float thetaAB = angles[0], thetaHip = angles[1], thetaKnee = angles[2];
    float signedHipLength = hipLength * hipSign;
    float legDistance = sqrt(upperLegLength * upperLegLength + lowerLegLength * lowerLegLength +
        2 * upperLegLength * lowerLegLength * cos(thetaKnee));
    float effSwing = thetaHip + thetaKnee / 2;
    float offXHip, offZHip, offYHip, offX, offY, offZ;
    offXHip = -legDistance * sin(effSwing);
    offZHip = -legDistance * cos(effSwing);
    offYHip = signedHipLength;

    offX = offXHip;
    offY = cos(thetaAB) * offYHip - sin(thetaAB) * offZHip;
    offZ = sin(thetaAB) * offYHip + cos(thetaAB) * offZHip;

    return Eigen::Matrix<float, 3, 1>(offX, offY, offZ);
}

Eigen::Matrix<float, 3, 3> qrRobotConfig::AnalyticalLegJacobian(Eigen::Matrix<float, 3, 1> &legAngles, int legId)
{
    float signedHipLength = hipLength * pow(-1, legId + 1);
    Eigen::Matrix<float, 3, 1> t = legAngles;

    float lEff = sqrt(upperLegLength * upperLegLength + lowerLegLength * lowerLegLength +
        2 * upperLegLength * lowerLegLength * cos(t[2]));
    float tEff = t[1] + t[2] / 2;

    Eigen::Matrix3f J = Eigen::Matrix3f::Zero();
    J(0, 0) = 0;
    J(0, 1) = -lEff * cos(tEff);
    J(0, 2) = lowerLegLength * upperLegLength * sin(t[2]) * sin(tEff) / lEff - lEff * cos(
        tEff) / 2;
    J(1, 0) = -signedHipLength * sin(t[0]) + lEff * cos(t(0)) * cos(tEff);
    J(1, 1) = -lEff * sin(t(0)) * sin(tEff);
    J(1, 2) = -lowerLegLength * upperLegLength * sin(t(0)) * sin(t(2)) * cos(
        tEff) / lEff - lEff * sin(t(0)) * sin(tEff) / 2;
    J(2, 0) = signedHipLength * cos(t(0)) + lEff * sin(t(0)) * cos(tEff);
    J(2, 1) = lEff * sin(tEff) * cos(t(0));
    J(2, 2) = lowerLegLength * upperLegLength * sin(t(2)) * cos(t(0)) * cos(
        tEff) / lEff + lEff * sin(tEff) * cos(t(0)) / 2;

    return J;
}

Eigen::Matrix<float, 3, 4> qrRobotConfig::FootPositionsInBaseFrame(Eigen::Matrix<float, 12, 1> footAngles)
{
    Eigen::Map<Eigen::MatrixXf> reshapedFootAngles(footAngles.data(), 3, 4);

    Eigen::MatrixXf footPositions = Eigen::Matrix<float, 3, 4>::Zero();
    for (int legId = 0; legId < qrRobotConfig::numLegs; legId++) {
        Eigen::Matrix<float, 3, 1> singleFootAngles;
        singleFootAngles << reshapedFootAngles.col(legId);
        footPositions.col(legId) = FootPositionInHipFrame(singleFootAngles, pow((-1), legId + 1));
    }
    return footPositions + hipOffset;
}

void qrRobotConfig::ComputeMotorAnglesFromFootLocalPosition(int legId,
                                                    Eigen::Matrix<float, 3, 1> footLocalPosition,
                                                    Eigen::Matrix<int, 3, 1> &jointIdx,
                                                    Eigen::Matrix<float, 3, 1> &jointAngles)
{
    int motorPreLeg = qrRobotConfig::dofPerLeg;
    jointIdx << motorPreLeg * legId, motorPreLeg * legId + 1, motorPreLeg * legId + 2;

    Eigen::Matrix<float, 3, 1> singleHipOffset;
    singleHipOffset << hipOffset.col(legId);
    Eigen::Matrix<float, 3, 1> singleFootLocalPosition = footLocalPosition - singleHipOffset;
    jointAngles = FootPositionInHipFrameToJointAngle(singleFootLocalPosition, pow(-1, (legId + 1)));

}

Eigen::Matrix<float, 3, 1> qrRobotConfig::ComputeMotorVelocityFromFootLocalVelocity(int legId,
                                                Eigen::Matrix<float, 3, 1> legAngles,
                                                Eigen::Matrix<float, 3, 1> footLocalVelocity)
{
    Eigen::Matrix<float, 3, 1> dq = AnalyticalLegJacobian(legAngles,legId).inverse()*footLocalVelocity;
    return dq;
    // return {0,0,0};
}

Vec3<float> qrRobotConfig::withLegSigns(const Vec3<float>& v, int legID)
{
    switch (legID) {
        case 0:
            return Vec3<float>(v[0], -v[1], v[2]);
        case 1:
            return Vec3<float>(v[0], v[1], v[2]);
        case 2:
            return Vec3<float>(-v[0], -v[1], v[2]);
        case 3:
            return Vec3<float>(-v[0], v[1], v[2]);
        default:
            throw std::runtime_error("Invalid leg id!");
    }
}

bool qrRobotConfig::BuildDynamicModel()
{
    // we assume the cheetah's body (not including rotors) can be modeled as a
    // uniformly distributed box.
    std::vector<float> bodySize = node["robot_params"]["body_size"].as<std::vector<float>>(); // Length, Width, Height
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
        Mat6<float> xtreeAbad = createSXform(I3, withLegSigns(_abadLocation, legID));
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
        //             0.1, 0, 0.,
        //             0, -0.3, 0.6,
        //             0, 0.3, 1;
                    
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
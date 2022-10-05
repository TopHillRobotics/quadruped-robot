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

// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: Xinyu Zhang   email: tophill.robotics@gmail.com

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

#include "robot/qr_robot_config.h"
#include "robot/qr_robot.h"

std::unordered_map<int, std::string> qrRobotConfig::modeMap =
  {{0, "velocity"}, {1, "position"}, {2, "walk"}};


qrRobotConfig::qrRobotConfig()
{
  bodyMass    = -1.0;
  bodyHeight  = -1.0;
  hipLength   = upperLength = lowerLength = -1.0;
  hipOffset   = Eigen::Matrix<float, 3, 4>::Zero();
  bodyInertia = Eigen::Matrix<float, 3, 3>::Zero();
  motorKps    = Eigen::Matrix<float, 12, 1>::Zero();
  motorKds    = Eigen::Matrix<float, 12, 1>::Zero();
}

qrRobotConfig::qrRobotConfig(std::string path)
{
  Load(path);
}

void qrRobotConfig::Load(std::string path)
{
  YAML::Node node = YAML::LoadFile(path);
  bodyMass    = node["body_mass"].as<float>();
  bodyHeight  = node["body_height"].as<float>();
  hipLength   = node["hip_length"].as<float>();
  upperLength = node["upper_length"].as<float>();
  lowerLength = node["lower_length"].as<float>();
  controlMode = node["controlMode"].as<int>();
  isSim = node["is simulation"].as<bool>();
  LoadKps(node);
  LoadKds(node);
  LoadComOffset(node);
  LoadHipOffset(node);
  LoadHipPosition(node);
  std::vector<float> bodyInertiaList = node["body_inertia"].as<std::vector<float >>();
  bodyInertia = Eigen::MatrixXf::Map(&bodyInertiaList[0], 3, 3);
}

void qrRobotConfig::LoadKps(YAML::Node &n)
{
  float abadKp = n["abad_kp"].as<float>();
  float hipKp  = n["hip_kp"].as<float>();
  float kneeKp = n["knee_kp"].as<float>();
  Eigen::Matrix<float, 3, 1> kp(abadKp, hipKp, kneeKp);
  motorKps << kp, kp, kp, kp;
}

void qrRobotConfig::LoadKds(YAML::Node &n)
{
  float abadKd = n["abad_kp"].as<float>();
  float hipKd  = n["hip_kp"].as<float>();
  float kneeKd = n["knee_kp"].as<float>();
  Eigen::Matrix<float, 3, 1> kd(abadKd, hipKd, kneeKd);
  motorKds << kd, kd, kd, kd;
}

void qrRobotConfig::LoadComOffset(YAML::Node &n)
{
  std::vector<float> comOffsetList = n["robot_params"][modeMap[controlMode]]["com_offset"].as<std::vector<float >>();
  comOffset = Eigen::MatrixXf::Map(&comOffsetList[0], 3, 1);
}

void qrRobotConfig::LoadHipOffset(YAML::Node &n)
{
  std::vector<std::vector<float >>
              hipOffsetList = n["hip_offset"].as<std::vector<std::vector<float>>>();
  Eigen::Matrix<float, 3, 1> hipOffsetFR = Eigen::MatrixXf::Map(&hipOffsetList[0][0], 3, 1) + comOffset;
  Eigen::Matrix<float, 3, 1> hipOffsetFL = Eigen::MatrixXf::Map(&hipOffsetList[1][0], 3, 1) + comOffset;
  Eigen::Matrix<float, 3, 1> hipOffsetRL = Eigen::MatrixXf::Map(&hipOffsetList[2][0], 3, 1) + comOffset;
  Eigen::Matrix<float, 3, 1> hipOffsetRR = Eigen::MatrixXf::Map(&hipOffsetList[3][0], 3, 1) + comOffset;
  hipOffset << hipOffsetFR, hipOffsetFL, hipOffsetRL, hipOffsetRR;
}

void qrRobotConfig::LoadHipPosition(YAML::Node &n)
{
  std::vector<std::vector<float>> defaultHipPositionList = n["default_hip_positions"].as<std::vector<std::vector<float>>>();
  Eigen::Matrix<float, 3, 1> defaultHipPositionFR = Eigen::MatrixXf::Map(&defaultHipPositionList[0][0], 3, 1);
  Eigen::Matrix<float, 3, 1> defaultHipPositionFL = Eigen::MatrixXf::Map(&defaultHipPositionList[1][0], 3, 1);
  Eigen::Matrix<float, 3, 1> defaultHipPositionRL = Eigen::MatrixXf::Map(&defaultHipPositionList[2][0], 3, 1);
  Eigen::Matrix<float, 3, 1> defaultHipPositionRR = Eigen::MatrixXf::Map(&defaultHipPositionList[3][0], 3, 1);
  defaultHipPosition << defaultHipPositionFR, defaultHipPositionFL, defaultHipPositionRL, defaultHipPositionRR;
}

Eigen::Matrix<float, 3, 3> qrRobotConfig::AnalyticalLegJacobian(Eigen::Matrix<float, 3, 1> &q, int legId)
{
  float signedHipLength = hipLength * float(pow(-1, legId + 1));
  Eigen::Matrix<float, 3, 1> t = q;

  float lEff = sqrtf(upperLength * upperLength + lowerLength * lowerLength +
      2 * upperLength * lowerLength * cosf(t[2]));
  float tEff = t[1] + t[2] / 2;

  Eigen::Matrix3f J = Eigen::Matrix3f::Zero();
  J(0, 0) = 0;
  J(0, 1) = -lEff * cosf(tEff);
  J(0, 2) = lowerLength * upperLength * sinf(t[2]) * sinf(tEff) / lEff - lEff * cosf(
      tEff) / 2;
  J(1, 0) = -signedHipLength * sinf(t[0]) + lEff * cosf(t(0)) * cosf(tEff);
  J(1, 1) = -lEff * sinf(t(0)) * sinf(tEff);
  J(1, 2) = -lowerLength * upperLength * sinf(t(0)) * sinf(t(2)) * cosf(
      tEff) / lEff - lEff * sinf(t(0)) * sinf(tEff) / 2;
  J(2, 0) = signedHipLength * cosf(t(0)) + lEff * sinf(t(0)) * cosf(tEff);
  J(2, 1) = lEff * sinf(tEff) * cosf(t(0));
  J(2, 2) = lowerLength * upperLength * sinf(t(2)) * cosf(t(0)) * cosf(
      tEff) / lEff + lEff * sinf(tEff) * cosf(t(0)) / 2;
  return J;
}

Eigen::Matrix<float, 3, 1> qrRobotConfig::FootPositionInHipFrame2JointAngle(
    Eigen::Matrix<float, 3, 1> &footPosition, int hipSign)
{
  float signedHipLength = hipLength * hipSign;
  Eigen::Matrix<float, 3, 1> xyz(footPosition[0], footPosition[1], footPosition[2]);
  Eigen::Matrix<float, 3, 1> legLength(signedHipLength, upperLength, lowerLength);

  float thetaAB, thetaHip, thetaKnee;
  thetaKnee = -acosf((xyz.squaredNorm() - legLength.squaredNorm()) / (2 * lowerLength * upperLength));
  float l = sqrtf(upperLength * upperLength + lowerLength * lowerLength +
      2 * upperLength * lowerLength * cosf(thetaKnee));
  thetaHip = asinf(-xyz.x() / l) - thetaKnee / 2;
  float c1 = signedHipLength * xyz.y() - l * cosf(thetaHip + thetaKnee / 2) * xyz.z();
  float s1 = l * cosf(thetaHip + thetaKnee / 2) * xyz.y() + signedHipLength * xyz.z();
  thetaAB = atan2f(s1, c1);

  return Eigen::Matrix<float, 3, 1>(thetaAB, thetaHip, thetaKnee);
}

Eigen::Matrix<float, 3, 1> qrRobotConfig::FootPosition2JointAngles(Eigen::Matrix<float, 3, 1> footPosition, int legId)
{
  Eigen::Matrix<float, 3, 1> legHipOffset = hipOffset.col(legId);
  Eigen::Matrix<float, 3, 1> footPositionWithOffset = footPosition - legHipOffset;
  return FootPositionInHipFrame2JointAngle(footPositionWithOffset, int(powf(-1, legId + 1)));
}

Eigen::Matrix<float, 3, 1> qrRobotConfig::FootVelocity2JointVelocity(
    Eigen::Matrix<float, 3, 1> q, Eigen::Matrix<float, 3, 1> dq, int legId)
{
  return AnalyticalLegJacobian(q, legId).inverse() * dq;
}

Eigen::Matrix<float, 3, 1> qrRobotConfig::JointVelocity2FootVelocity(
    Eigen::Matrix<float, 3, 1> q, Eigen::Matrix<float, 3, 1> dq, int legId)
{
  return AnalyticalLegJacobian(q, legId) * dq;
}

Eigen::Matrix<float, 3, 1> qrRobotConfig::JointAngles2FootPositionInHipFrame(Eigen::Matrix<float, 3, 1> q, int hipSign)
{
  float thetaAB = q[0], thetaHip = q[1], thetaKnee = q[2];
  float signedHipLength = hipLength * hipSign;
  float legDistance = sqrtf(upperLength * upperLength + lowerLength * lowerLength +
      2 * upperLength * lowerLength * cosf(thetaKnee));
  float effSwing = thetaHip + thetaKnee / 2;
  float offXHip, offZHip, offYHip, offX, offY, offZ;
  offXHip = -legDistance * sinf(effSwing);
  offZHip = -legDistance * cosf(effSwing);
  offYHip = signedHipLength;

  offX = offXHip;
  offY = cosf(thetaAB) * offYHip - sinf(thetaAB) * offZHip;
  offZ = sinf(thetaAB) * offYHip + cosf(thetaAB) * offZHip;

  return Eigen::Matrix<float, 3, 1>(offX, offY, offZ);
}

Eigen::Matrix<float, 3, 4> qrRobotConfig::JointAngles2FootPositionInBaseFrame(Eigen::Matrix<float, 12, 1> q)
{
  Eigen::Map<Eigen::MatrixXf> reshapedFootAngles(q.data(), 3, 4);

  Eigen::MatrixXf footPositions = Eigen::Matrix<float, 3, 4>::Zero();
  for (unsigned int legId = 0; legId < numLegs; legId++) {
      Eigen::Matrix<float, 3, 1> singleFootAngles;
      singleFootAngles << reshapedFootAngles.col(legId);
      footPositions.col(legId) = JointAngles2FootPositionInHipFrame(
            singleFootAngles, int(powf(-1, legId + 1)));
  }
  return footPositions + hipOffset;
}


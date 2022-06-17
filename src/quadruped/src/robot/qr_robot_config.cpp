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

qrRobotConfig::qrRobotConfig()
{
  bodyMass    = -1.0;
  bodyHeight  = -1.0;
  hipLength   = upperLength = lowerLength = -1.0;
  bodyInertia = Eigen::Matrix<float, 3, 3>::Zero();
  motorKps    = Eigen::Matrix<float, 12, 1>::Zero();
  motorKds    = Eigen::Matrix<float, 12, 1>::Zero();
}

qrRobotConfig::qrRobotConfig(std::string path)
{
  load(path);
}

void qrRobotConfig::load(std::string path)
{
  YAML::Node node = YAML::LoadFile(path);
  bodyMass    = node["body_mass"].as<float>();
  bodyHeight  = node["body_height"].as<float>();
  hipLength   = node["hip_length"].as<float>();
  upperLength = node["upper_length"].as<float>();
  lowerLength = node["lower_length"].as<float>();
  motorKps    = loadKps(node);
  motorKds    = loadKds(node);
  std::vector<float> bodyInertiaList = node["body_inertia"].as<std::vector<float >>();
  bodyInertia = Eigen::MatrixXf::Map(&bodyInertiaList[0], 3, 3);
}

Eigen::Matrix<float, 12, 1> qrRobotConfig::loadKps(YAML::Node &n)
{
  float abadKp = n["abad_kp"].as<float>();
  float hipKp = n ["hip_kp"].as<float>();
  float kneeKp = n["knee_kp"].as<float>();
  Eigen::Matrix<float, 3, 1> kp(abadKp, hipKp, kneeKp);
  Eigen::Matrix<float, 12, 1> kps;
  kps << kp, kp, kp, kp;
  return kps;
}

Eigen::Matrix<float, 12, 1> qrRobotConfig::loadKds(YAML::Node &n)
{
  float abadKd = n["abad_kp"].as<float>();
  float hipKd = n ["hip_kp"].as<float>();
  float kneeKd = n["knee_kp"].as<float>();
  Eigen::Matrix<float, 3, 1> kd(abadKd, hipKd, kneeKd);
  Eigen::Matrix<float, 12, 1> kds;
  kds << kd, kd, kd, kd;
  return kds;
}

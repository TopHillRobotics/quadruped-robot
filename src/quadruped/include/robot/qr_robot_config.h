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

#ifndef QR_ROBOT_CONFIG_H
#define QR_ROBOT_CONFIG_H

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>


/**
 * @brief a class that will load all robot configs from YAML file
 */
class qrRobotConfig
{
public:
  /**
   * @brief Construction of qrRobotConfig
   */
  qrRobotConfig();

  /**
   * @brief Construction of qrRobotConfig
   * @param path: the path to the YAML config file
   */
  qrRobotConfig(std::string path);
  /**
   * @brief Destruction of qrRobotConfig
   */
  ~qrRobotConfig();

  /**
   * @brief load parameter of the robot
   * @param path: the path to the YAML config file
   */
  void load(std::string path);

private:
  /**
   * @brief the weight of the main body
   */
  float bodyMass;

  /**
   * @brief the height of the main body
   */
  float bodyHeight;

  /**
   * @brief the length of three links, including hip, upper link and lowerlink
   */
  float hipLength, upperLength, lowerLength;

  /**
   * @brief the tensor of inertia of the body
   */
  Eigen::Matrix<float, 3, 3> bodyInertia;

  // Eigen::Matrix<float, 3, 1> comOffset; this is not used
  // Eigen::Matrix<float, 3, 4> hipOffset; this is not used

  //TODO: discuss
  // Eigen::Matrix<float, 3, 4> defaultHipPosition;

  /**
   * @brief motor position stiffness (unit: N.m/rad )
   */
  Eigen::Matrix<float, 12, 1> motorKps;


  /**
   * @brief velocity stiffness (unit: N.m/(rad/s) )
   */
  Eigen::Matrix<float, 12, 1> motorKds;

  // TODO: check these variables
  // Eigen::Matrix<float, 12, 1> jointDirection; this is not used
  // Eigen::Matrix<float, 12, 1> jointOffset; this is not used
  // Eigen::Matrix<float, 12, 1> standUpMotorAngles; should not used here
  // Eigen::Matrix<float, 12, 1> sitDownMotorAngles; should not used here


  /**
   * @brief load kps parameter from YAML file
   * @param node: node that load YAML file
   */
  Eigen::Matrix<float, 12, 1> loadKps(YAML::Node &node);


  /**
   * @brief load kps parameter from YAML file
   * @param path: file path
   */
  Eigen::Matrix<float, 12, 1> loadKds(YAML::Node &node);
};

#endif // QR_ROBOT_CONFIG_H

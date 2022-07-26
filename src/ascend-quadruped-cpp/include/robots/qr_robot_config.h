#ifndef QR_ROBOT_CONFIG_H
#define QR_ROBOT_CONFIG_H

#include <iostream>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "utils/cppTypes.h"
#include "utils/se3.h"

class RobotConfig
{
public:

    RobotConfig();

    RobotConfig(std::string path);

    void Load(std::string path);

    void LoadKps(YAML::Node &node);

    void LoadKds(YAML::Node &node);

    void LoadComOffset(YAML::Node &node);

    void LoadHipOffset(YAML::Node &node);

    void LoadHipPosition(YAML::Node &node);

    static const int numMotors = 12;
    static const int numLegs = 4;
    static const int dofPerLeg = 3;

    float footHoldOffset = 0.1f;// ???
    std::map<std::string, int> controlParams;

    std::string configFilePath;
    YAML::Node robotConfig;
    std::string robotName;
    float bodyMass;
    Eigen::Matrix<float, 3, 3> bodyInertia = Eigen::Matrix<float, 3, 3>::Zero();
    float bodyHeight;
    float hipLength;
    float upperLegLength;
    float lowerLegLength;

    Eigen::Matrix<float, 3, 1> comOffset;
    Eigen::Matrix<float, 3, 4> hipOffset;
    Eigen::Matrix<float, 3, 4> defaultHipPosition;

    Eigen::Matrix<float, 12, 1> motorKps;
    Eigen::Matrix<float, 12, 1> motorKds;

//    Eigen::Matrix<float, 12, 1> jointDirection = Eigen::Matrix<float, 12, 1>::Ones();
//    Eigen::Matrix<float, 12, 1> jointOffset = Eigen::Matrix<float, 12, 1>::Zero();

    bool isSim;

    Eigen::Matrix<float, 3, 1>
    FootPositionInHipFrameToJointAngle(Eigen::Matrix<float, 3, 1> &footPosition, int hipSign = 1);

    Eigen::Matrix<float, 3, 1>
    FootPositionInHipFrame(Eigen::Matrix<float, 3, 1> &angles, int hipSign = 1);

    Eigen::Matrix<float, 3, 3>
    AnalyticalLegJacobian(Eigen::Matrix<float, 3, 1> &legAngles, int legId);

    Eigen::Matrix<float, 3, 4> FootPositionsInBaseFrame(Eigen::Matrix<float, 12, 1> footAngles);

    void ComputeMotorAnglesFromFootLocalPosition(int legId,
                                                 Eigen::Matrix<float, 3, 1> footLocalPosition,
                                                 Eigen::Matrix<int, 3, 1> &jointIdx,
                                                 Eigen::Matrix<float, 3, 1> &jointAngles);

    Eigen::Matrix<float, 3, 1> ComputeMotorVelocityFromFootLocalVelocity(int legId,
                                                Eigen::Matrix<float, 3, 1> legAngles,
                                                Eigen::Matrix<float, 3, 1> footLocalVelocity);

};
#endif // QR_ROBOT_CONFIG_H

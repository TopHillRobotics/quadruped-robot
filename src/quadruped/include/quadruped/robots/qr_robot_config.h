#ifndef QR_ROBOT_CONFIG_H
#define QR_ROBOT_CONFIG_H

#include <iostream>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "utils/cppTypes.h"
#include "utils/se3.h"

class qrRobotConfig
{
public:

    /**
     * @brief Construction of qrRobotConfig
     * @param path: the path to the YAML config file
     */
    qrRobotConfig(std::string path);

    /**
     * @brief load parameter of the robot
     * @param path: the path to the YAML config file
     */
    void Load(std::string path);

    /**
     * @brief motor numbers of the robot
     */
    static const int numMotors = 12;

    /**
     * @brief number of legs
     */
    static const int numLegs = 4;

    /**
     * @brief DOF of each leg
     */
    static const int dofPerLeg = 3;

    float footHoldOffset = 0.1f;

    /**
     * @brief control parameters of the robot
     */
    std::map<std::string, int> controlParams;

    /**
     * @brief the weight of the main body
     */
    float bodyMass;

    Eigen::Matrix<float, 3, 3> bodyInertia = Eigen::Matrix<float, 3, 3>::Zero();

    /**
     * @brief the height of the main body
     */
    float bodyHeight;

    /**
     * @brief the length of three links, including hip, upper link and lowerlink
     */
    float hipLength, upperLegLength, lowerLegLength;

    Eigen::Matrix<float, 3, 1> comOffset;

    /**
     * @brief offset of hip
     */
    Eigen::Matrix<float, 3, 4> hipOffset;

    /**
     * @brief position of 4 hips
     */
    Eigen::Matrix<float, 3, 4> defaultHipPosition;

    /**
     * @brief motor position stiffness (unit: N.m/rad )
     */
    Eigen::Matrix<float, 12, 1> motorKps;

    /**
     * @brief velocity stiffness (unit: N.m/(rad/s) )
     */
    Eigen::Matrix<float, 12, 1> motorKds;

    /**
     * @brief whether robot is simulation
     */
    bool isSim;

    /**
     * @brief convert foot position in hip frame to joint angles
     * @param footPosition: position of foot in hip frame
     * @param hipSign: hip position
     * @return joint angles
     */
    Eigen::Matrix<float, 3, 1>
    FootPositionInHipFrameToJointAngle(Eigen::Matrix<float, 3, 1> &footPosition, int hipSign = 1);

    /**
     * @brief get foot position in hip frame
     * @param angles: angles of one leg
     * @param hipSign: hip position
     * @return foot position in hip frame
     */
    Eigen::Matrix<float, 3, 1>
    FootPositionInHipFrame(Eigen::Matrix<float, 3, 1> &angles, int hipSign = 1);

    /**
     * @brief calculate Jacobian of a leg with leg ID and angles
     * @param legAngles: angles
     * @param legId: which leg to calculate
     * @return
     */
    Eigen::Matrix<float, 3, 3>
    AnalyticalLegJacobian(Eigen::Matrix<float, 3, 1> &legAngles, int legId);

    /**
     * @brief calculate foot position in base frame of robot
     * @param footAngles: joint angles
     * @return foot position in base frame
     */
    Eigen::Matrix<float, 3, 4> FootPositionsInBaseFrame(Eigen::Matrix<float, 12, 1> footAngles);

    /**
     * @brief convert foot position to joint angles
     * @param legId: which leg to calculate
     * @param footLocalPosition: the position of the foothold
     * @param jointIdx: index of foot to compute
     * @param jointAngles: angles to compute
     * @return position(x, y, z)
     */
    void ComputeMotorAnglesFromFootLocalPosition(int legId,
                                                 Eigen::Matrix<float, 3, 1> footLocalPosition,
                                                 Eigen::Matrix<int, 3, 1> &jointIdx,
                                                 Eigen::Matrix<float, 3, 1> &jointAngles);

    /**
     * @brief ComputeMotorVelocityFromFootLocalVelocity
     * @param legId: id of leg to compute
     * @param legAngles: angles of 3 motors of leg to compute
     * @param footLocalVelocity: foot end velocity
     * @return motor velocity
     */
    Eigen::Matrix<float, 3, 1> ComputeMotorVelocityFromFootLocalVelocity(int legId,
                                                Eigen::Matrix<float, 3, 1> legAngles,
                                                Eigen::Matrix<float, 3, 1> footLocalVelocity);

private:

    /**
     * @brief auxiliary function for loading Kp
     * @param node: YAML node
     */
    void LoadKps(YAML::Node &node);

    /**
     * @brief auxiliary function for loading Kd
     * @param node: YAML node
     */
    void LoadKds(YAML::Node &node);

    /**
     * @brief auxiliary function for loading CoM offset
     * @param node: YAML node
     */
    void LoadComOffset(YAML::Node &node);

    /**
     * @brief auxiliary function for loading hip offset
     * @param node: YAML node
     */
    void LoadHipOffset(YAML::Node &node);

    /**
     * @brief auxiliary function for loading hip position
     * @param node: YAML node
     */
    void LoadHipPosition(YAML::Node &node);
};
#endif // QR_ROBOT_CONFIG_H

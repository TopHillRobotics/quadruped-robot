// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:tophill.robotics@gmail.com

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
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "common/qr_eigen_types.h"

/**
 * @brief a class that will load all robot configs from YAML file
 */
class qrRobotConfig
{
public:

    friend class qrRobot;
    friend class qrRobotState;
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
     * @brief load parameter of the robot
     * @param path: the path to the YAML config file
     */
    void Load(std::string path);

    /**
     * @brief calculate Jacobian of a leg with leg ID and angles
     * @param q: angles
     * @param legId: which leg to calculate
     * @return
     */
    Mat3<float> AnalyticalLegJacobian(Vec3<float> &q, int legId);

    /**
     * @brief calculate foot position in base frame of robot
     * @param q: joint angles
     * @return foot position in base frame
     */
    Mat3x4<float> JointAngles2FootPositionInBaseFrame(Vec12<float> q);

    /**
     * @brief convert foot position to joint angles
     * @param legId: which leg to calculate
     * @param position: the position of the foothold
     * @return position(x, y, z)
     */
    Vec3<float> FootPosition2JointAngles(Vec3<float> position, int legId);


    /**
     * @brief convert foot velocity2JointVelocity
     * @param q: joint angles
     * @param v: fothold velocity
     * @param legId: which leg to convert
     * @return joint velocity of the leg
     */
    Vec3<float> FootVelocity2JointVelocity(
        Vec3<float> q, Vec3<float> v, int legId);

    /**
     * @brief convert foot velocity2JointVelocity
     * @param q: joint angles
     * @param v: fothold velocity
     * @param legId: which leg to convert
     * @return joint velocity of the leg
     */
    Vec3<float> JointVelocity2FootVelocity(
        Vec3<float> q, Vec3<float> v, int legId);

    /**
     * @brief get hip positions
     * @return hip positions of 4 legs
     */
    inline Mat3x4<float> GetHipPositionsInBaseFrame(){
        return defaultHipPosition;
    }

    /**
     * @brief get Kp of the robot config
     * @return
     */
    inline Vec12<float> GetKps() const{
        return motorKps;
    }

    /**
     * @brief get Kd of the robot config
     * @return
     */
    inline Vec12<float> GetKds() const{
        return motorKds;
    }

    /**
     * @brief motor numbers of the robot
     */
    static const unsigned int numMotor  = 12;

    /**
     * @brief number of legs
     */
    static const unsigned int numLegs   = 4;

    /**
     * @brief DOF of each leg
     */
    static const unsigned int dofPerLeg = 3;

    // TODO: explain
    static const float footHoldOffset;

    /**
     * @brief control mode of the robot
     */
    int controlMode;

    /**
     * @brief whether the robot is simulation or real robot
     */
    bool isSim;


    inline Mat3<float> GetBodyInertia() const{
        return bodyInertia;
    }

    inline float GetBodyMass() const{
        return bodyMass;
    }

    inline void SetYawOffset(float yawOffset){
        this->yawOffset = yawOffset;
    }

    inline float GetBodyHeight() const{
        return bodyHeight;
    }

    inline float GetHipLength() const{
        return hipLength;
    }

    inline float GetUpperLength() const{
        return upperLength;
    }


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

    // check difference of these two;
    Mat3x4<float> defaultHipPosition;

    /**
     * @brief offset of hip
     */
    Mat3x4<float> hipOffset;

    /**
     * @brief offset of center of mass
     */
    Vec3<float> comOffset;

    /**
     * @brief the tensor of inertia of the body
     */
    Mat3<float> bodyInertia;

    /**
     * @brief motor position stiffness (unit: N.m/rad )
     */
    Vec12<float> motorKps;


    /**
     * @brief velocity stiffness (unit: N.m/(rad/s) )
     */
    Vec12<float> motorKds;

    // TODO: check these variables
    // Vec12<float> jointDirection; this is not used
    // Vec12<float> jointOffset; this is not used
    // Vec12<float> standUpMotorAngles; should not used here
    // Vec12<float> sitDownMotorAngles; should not used here

    float yawOffset = 0.f;

    /**
     * @brief load kps parameter from YAML file
     * @param node: node that load YAML file
     */
    void LoadKps(YAML::Node &node);

    /**
     * @brief load kps parameter from YAML file
     * @param path: file path
     */
    void LoadKds(YAML::Node &node);


    /**
     * @brief load CoM offset
     * @param node: YAML node of config file
     */
    void LoadComOffset(YAML::Node &node);

    /**
     * @brief load Hip offset
     * @param node: YAML node of config file
     */
    void LoadHipOffset(YAML::Node &node);

    /**
     * @brief load Hip Position
     * @param node: YAML node of config file
     */
    void LoadHipPosition(YAML::Node &node);

    /**
     * @brief convert foot position in hip frame to joint angles
     * @param footPosition: the position(x, y, z) of foot point
     * @param hipSign: FR & RR: 1, FL & RL: -1
     * @return joint angles
     */
    Vec3<float> FootPositionInHipFrame2JointAngle(Vec3<float> &footPosition, int hipSign);

    // TODO: discuss this. world frame?
    /**
     * @brief convert angles q to joint position in hip frame
     * @param q: joint angles
     * @param hipSign: FR & RR: 1, FL & RL: -1
     * @return
     */
    Vec3<float> JointAngles2FootPositionInHipFrame(Vec3<float> q, int hipSign);

    /**
     * @brief mapping of different control modes
     */
    static std::unordered_map<int, std::string> modeMap;

};

#endif // QR_ROBOT_CONFIG_H

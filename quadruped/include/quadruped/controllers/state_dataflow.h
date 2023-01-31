/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: record the state flow for planner and controller.
* Author: Zhu Yijie
* Create: 2022-03-27
* Notes: xx
* Modify: init the file. @ Zhu Yijie
* Modify: add UserParameters class @ Zhu Yijie 2022.04.01
*/

#ifndef ASCEND_QUADRUPED_CPP_ROBOT_STATE_DATAFLOW_H
#define ASCEND_QUADRUPED_CPP_ROBOT_STATE_DATAFLOW_H

#include "../config/config.h"
#include "utils/geometry.h"
#include "utils/se3.h"
#include "utils/visualization.h"
#include <yaml-cpp/yaml.h>

struct UserParameters {
    UserParameters(std::string filePath);
    float stairsTime;
    float stairsVel;
    unsigned int controlFrequency = 500;// hz

    // ground estimator
    unsigned int filterWindowSize = 50;

    // velocity estimator
    float accelerometerVariance = 0.01f;// 0.1f
    float sensorVariance = 0.01f;
    float initialVariance = 0.01f;
    int movingWindowFilterSize = 50;// 120 ms

    // swing controller
    float desiredHeight = A1_BODY_HIGHT;
    Vec3<float> desiredSpeed = {0.f, 0.f, 0.f};
    float desiredTwistingSpeed = 0.f;
    float footClearance = 0.01f;
    Vec4<float> frictionCoeffs = {0.45, 0.45, 0.45, 0.45};// FR, FL, RR, RL
    std::map<std::string, std::vector<float>> swingKp;

    // stance controller
    bool computeForceInWorldFrame = true;
    // mit stance leg controller
    bool useWBC = true;
};

class WbcCtrlData {
public:
    Vec3<float> pBody_des;
    Vec3<float> vBody_des;
    Vec3<float> aBody_des;
    Vec3<float> pBody_RPY_des;
    Vec3<float> vBody_Ori_des;

    Vec3<float> pFoot_des[4];
    Vec3<float> vFoot_des[4];
    Vec3<float> aFoot_des[4];
    Vec3<float> Fr_des[4];

    Vec4<bool> contact_state;

    bool allowAfterMPC = true;
};

namespace Quadruped {
    struct StateDataFlow {
        Eigen::Matrix<float, 3, 4> footPositionsInBaseFrame;
        Eigen::Matrix<float, 3, 4> footVelocitiesInBaseFrame;
        Vec3<float> baseVInWorldFrame;                // in World frame
        Vec3<float> baseWInWorldFrame;                //BASE Angular velocity in World frame
        Vec3<float> baseLinearAcceleration;           // in base/IMU frame
        std::vector<Mat3<float>> footJvs;             // Jacobian
        Eigen::Matrix<float, 3, 4> estimatedFootForce;// in base frame
        Vec3<float> estimatedMoment;                  // in base frame
        float heightInControlFrame = 0.27;
        Vec3<float> zmp;

        Mat3<float> baseRMat;
        Mat3<float> groundRMat;
        Vec4<float> groundOrientation;
        Mat3<float> baseRInControlFrame;

        Visualization2D visualizer;

        WbcCtrlData wbcData;

        StateDataFlow()
        {
            footPositionsInBaseFrame.setZero();
            footVelocitiesInBaseFrame.setZero();
            baseVInWorldFrame.setZero();
            baseWInWorldFrame.setZero();
            baseLinearAcceleration.setZero();
            footJvs = std::vector<Mat3<float>>(4, Mat3<float>::Identity());
            estimatedFootForce.setZero();
            estimatedMoment.setZero();
            zmp.setZero();
            baseRMat.setIdentity();
            groundRMat.setIdentity();
            groundOrientation << 1.f, 0.f, 0.f, 0.f;
            baseRInControlFrame.setIdentity();
        }
    };
}// namespace Quadruped
#endif//ASCEND_QUADRUPED_CPP_ROBOT_STATE_DATAFLOW_H

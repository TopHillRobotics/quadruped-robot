/* 
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.        
* Description: compute the force for stance controller. 
* Author: Zang Yaohua
* Create: 2021-10-25
* Notes: now this cotroller is based on floating-base dynamics, with lineard equations and MPC.
* Modify: init the file. @ Zang Yaohua
*/

#ifndef ASCEND_QUADRUPED_CPP_QP_TORQUE_OPTIMIZER_MPC_H
#define ASCEND_QUADRUPED_CPP_QP_TORQUE_OPTIMIZER_MPC_H

#include <tuple>
#include <Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "robots/robot.h"
#include "state_estimator/ground_estimator.h"

namespace Quadruped {

    const int predHorizon = 1;
    static float predTimeStep = 0.02; //0.115; //0.0575; // 0.115;
    const int ctrlHorizon = 1;
    std::tuple<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> ComputeMassMatrixMPC(
            float robotMass,
            Eigen::Matrix<float, 3, 3> robotInertia,
            Eigen::Matrix<float, 4, 3> footPositions,
            Vec3<float> rpy);

    std::tuple<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> ComputeConstraintMatrixMPC(
            float mpcBodyMass,
            Eigen::Matrix<int, 4, 1> contacts,
            float frictionCoef,
            float fMinRatio,
            float fMaxRatio);

    std::tuple<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<float, Eigen::Dynamic, 1>> ComputeObjectiveMatrixMPC(
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A,
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> B,
            Eigen::Matrix<float, 13, 1> X,
            Eigen::Matrix<float, 13, 1> desiredX,
            Eigen::Matrix<float, 13, 1> XWeight,
            float regWeight);

    Eigen::Matrix<float, 3*predHorizon, 4> ComputeContactForceMPC(Robot *robot,
                                                   Eigen::Matrix<float, 13, 1> X,
                                                   Eigen::Matrix<float, 13, 1> desiredX,
                                                   Eigen::Matrix<int, 4, 1> contacts,
                                                   Eigen::Matrix<float, 13, 1> XWeight,
                                                   float regWeight=1e-6,
                                                   float frictionCoef=0.6f,
                                                   float fMinRatio=0.1f,
                                                   float fMaxRatio=10.f);
}

#endif //ASCEND_QUADRUPED_CPP_QP_TORQUE_OPTIMIZER_MPC_H
/* 
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.        
* Description: compute the force for stance controller. 
* Author: Zang Yaohua
* Create: 2021-12-13
* Notes: xx
* Modify: init the file. @ Zang Yaohua
* Modify: correct func ComputeObjectiveMatrixMPC @ Zhu Yijie 2022.03.01
*/

#include "controllers/mpc/qp_torque_optimizer_mpc.h"
#include "QuadProg++.hh"
#include "Array.hh"

namespace Quadruped {

    std::tuple<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> ComputeMassMatrixMPC(
            float robotMass,
            Eigen::Matrix<float, 3, 3> robotInertia,
            Eigen::Matrix<float, 4, 3> footPositions,
            Vec3<float> rpy)
    {
        const int sizeAqp = 13 * predHorizon;
        const int colSizeBqp = 12 * predHorizon;
        const int rowSizeAdtList = 13 * predHorizon + 13;
        Eigen::Matrix<float, 3, 3> invMass;
        Eigen::Matrix<float, 3, 3> invInertiaWorld;
        Eigen::Matrix<float, 25, 25> AB; AB.setZero();
        Eigen::Matrix<float, 13, 13> A; A.setZero();
        Eigen::Matrix<float, 13, 12> B; B.setZero();
        Eigen::Matrix<float, 13, 13> Adt; Adt.setZero();
        Eigen::Matrix<float, 13, 12> Bdt; Bdt.setZero();
        Eigen::Matrix<float, 25, 25> ABexp; ABexp.setZero();
        Eigen::Matrix<float, rowSizeAdtList,13> AdtList;
        Eigen::Matrix<float, sizeAqp, 13> Aqp; Aqp.setZero();
        Eigen::Matrix<float, sizeAqp, colSizeBqp> Bqp; Bqp.setZero();
        Eigen::Matrix<float, 1, 3> x;
        Eigen::Matrix<float, 3, 3> footPositionSkew;
        
        Mat3<float> R = robotics::math::rpyToRotMat(rpy).transpose();
        invMass= Eigen::Matrix<float, 3, 3>::Identity(3, 3) / robotMass;
        invInertiaWorld = R * robotInertia.inverse() * R.transpose();

        A.block<3,3>(0, 6) = R.transpose(); // angular_velocity_to_rpy_rate
        A.block<3,3>(3,9) = Eigen::Matrix<float, 3, 3>::Identity(3, 3);
        A(11, 12) = 1.;
        for (int legId = 0; legId < 4; ++legId) {
            B.block<3, 3>(9, legId * 3) = invMass;
            x = footPositions.row(legId);
            footPositionSkew << 0., -x[2], x[1],
                                x[2], 0., -x[0],
                                -x[1], x[0], 0.;
            B.block<3, 3>(6, legId * 3) = invInertiaWorld * footPositionSkew;
        }

        AB.block<13,13>(0,0) = A * predTimeStep;
        AB.block<13,12>(0, 13) = B * predTimeStep;
        ABexp = AB.exp();
        Adt = ABexp.block<13,13>(0,0);
        Bdt = ABexp.block<13,12>(0,13);

        AdtList.block<13,13>(0,0) = Eigen::Matrix<float, 13, 13>::Identity(13, 13);
        for (int step = 1; step < predHorizon + 1; ++step) {
            AdtList.block<13,13>(step*13, 0) = Adt * AdtList.block<13,13>((step-1)*13, 0);
        }

        for (int step = 0; step < predHorizon; ++step) {
            Aqp.block<13,13>(step*13, 0) = AdtList.block<13,13>((step+1)*13, 0);
            for (int i=0; i<(step+1); ++i) {
                Bqp.block<13,12>(step*13, i*12) = AdtList.block<13,13>((step-i)*13, 0) * Bdt;
            }
        }

        std::tuple<Eigen::Matrix<float, sizeAqp, 13>, Eigen::Matrix<float, sizeAqp, colSizeBqp>> ABqp(Aqp, Bqp);
        return ABqp;
    }

    std::tuple<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> ComputeConstraintMatrixMPC(
            float mpcBodyMass,
            Eigen::Matrix<bool, 4, 1> contacts,
            float frictionCoef,
            float fMinRatio,
            float fMaxRatio)
    {
        const int rowSizeAA = predHorizon*4*6;
        const int colSizeAA = predHorizon*4*3;
        float fMin = fMinRatio * mpcBodyMass * 9.8;
        float fMax = fMaxRatio * mpcBodyMass * 9.8;
        Eigen::Matrix<float, 6, 3> Ablock; Ablock.setZero();
        Eigen::Matrix<float, 24, 1> lblock; lblock.setZero();
        Eigen::Matrix<float, rowSizeAA, colSizeAA> AA; AA.setZero();
        Eigen::Matrix<float, rowSizeAA, 1> lb; lb.setZero();

        Ablock << 1.,  0., frictionCoef,
                  -1., 0., frictionCoef,
                  0.,  1., frictionCoef,
                  0., -1., frictionCoef,
                  0., 0.,  1.,
                  0., 0., -1.;
        // AA
        for (int n=0; n<(ctrlHorizon*4); ++n) {
            AA.block<6, 3>(n*6, n*3) = Ablock;
        }

        for (int legId = 0; legId < 4; legId++) {
            if (contacts(legId, 0)) {
                lblock(legId*6 + 4, 0) = fMin;
                lblock(legId*6 + 5, 0) = -fMax;
            } else {
                lblock(legId*6 + 4, 0) = -1e-4;
                lblock(legId*6 + 5, 0) = -1e-4;
            }
        }
        // lb
        for (int step=0; step<ctrlHorizon; ++step) {
            lb.block<24, 1>(step*24, 0) = lblock;
        }

        std::tuple<Eigen::Matrix<float, colSizeAA, rowSizeAA>, Eigen::Matrix<float, rowSizeAA, 1>> Alb(AA.transpose(), lb);
        return Alb;
    }

    std::tuple<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<float, Eigen::Dynamic, 1>> ComputeObjectiveMatrixMPC(
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A,
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> B,
            Eigen::Matrix<float, 13, 1> Xk,
            Eigen::Matrix<float, 13, 1> desiredX,
            Eigen::Matrix<float, 13, 1> XWeight,
            float regWeight)
    {
        const int desiredXsRowSize = 13*predHorizon;
        Eigen::Matrix<float, desiredXsRowSize, 1> desiredXs;
        // Obtain reference trajectory
        for (int i=0; i<predHorizon; ++i) {
            Mat3<float> expUpper = robotics::math::vectorToSkewMat(Vec3<float>(desiredX.segment(6,3)))* (predTimeStep * (i+1));
            Mat3<float> dR = expUpper.exp();
            Vec3<float> drpy = robotics::math::rotationMatrixToRPY(dR.transpose());
            Mat3<float> R0 = robotics::math::rpyToRotMat(Vec3<float>(Xk.head(3))).transpose();
            Vec3<float> RPYdes = robotics::math::rotationMatrixToRPY((dR*R0).transpose());
            // (roll, pitch, yaw)
            if (Xk[2] > M_PI_2 && RPYdes(2,0) < 0) {
                    RPYdes(2,0) += M_2PI;
            } else if (Xk[2] < -M_PI_2 && RPYdes(2,0) > 0) {
                    RPYdes(2,0) -= M_2PI;
            }
            desiredXs(i*13+0, 0) = 0;//RPYdes(0, 0); // desiredX(0,0); 
            desiredXs(i*13+1, 0) = 0;//RPYdes(1, 0); // desiredX(1,0);   
            desiredXs(i*13+2, 0) = RPYdes(2, 0); // desiredX(2,0);  
            // (x, y, z)
            desiredXs(i*13+3, 0) = predTimeStep * (i+1) * desiredX(9, 0);  // desiredX(3,0);
            desiredXs(i*13+4, 0) = predTimeStep * (i+1) * desiredX(10, 0); // desiredX(4,0);
            desiredXs(i*13+5, 0) = desiredX(5, 0); // Xk[5] + predTimeStep * (i+1) * desiredX(10,0);
            // (droll, dpitch, dyaw)
            desiredXs(i*13+6, 0) = desiredX(6, 0);
            desiredXs(i*13+7, 0) = desiredX(7, 0);
            desiredXs(i*13+8, 0) = desiredX(8, 0);
            // (dx, dy, dz)
            desiredXs(i*13+9, 0) = desiredX(9, 0);
            desiredXs(i*13+10, 0) = desiredX(10, 0);
            desiredXs(i*13+11, 0) = desiredX(11, 0);
            // (-g)
            desiredXs(i*13+12, 0) = desiredX(12, 0);
        }
        // formulate quad term and linear term
        Eigen::Matrix<float, desiredXsRowSize, desiredXsRowSize> Q; Q.setZero();
        Eigen::Matrix<float, 13, 13> Qblock; Qblock.setZero();
        for (int i = 0; i < 13; ++i) {
            Qblock(i, i) = XWeight(i, 0);
        }
        for (int i = 0; i < predHorizon; ++i) {
            Q.block<13, 13>(i*13, i*13) = Qblock;
        }
        const int Rsize = predHorizon * 12;
        Eigen::Matrix<float, Rsize, Rsize> R; R.setIdentity();
        R *= regWeight;
        Eigen::Matrix<float, Rsize, Rsize> quadTerm; // quadTerm.setZero();
        Eigen::Matrix<float, Rsize, 1> linearTerm; // linearTerm.setZero();
        quadTerm = 2 * (B.transpose() * Q * B) + R;
        linearTerm =  2 * B.transpose() * Q.transpose() * (A * Xk - desiredXs);
        std::tuple<Eigen::Matrix<float, Rsize, Rsize>, Eigen::Matrix<float, Rsize, 1>> quadLinear(quadTerm, linearTerm);
        return quadLinear;
    }

    Eigen::Matrix<float, 3*predHorizon, 4> ComputeContactForceMPC(Robot *quadruped,
                                                   Eigen::Matrix<float, 13, 1> X,
                                                   Eigen::Matrix<float, 13, 1> desiredX,
                                                   Eigen::Matrix<bool, 4, 1> contacts,
                                                   Eigen::Matrix<float, 13, 1> XWeight,
                                                   float regWeight,
                                                   float frictionCoef,
                                                   float fMinRatio,
                                                   float fMaxRatio)
    {
        // update predTimeStep
        // predTimeStep = std::max(0.5 * (desiredX(9,0) - X(9,0)), 0.05);
        // predTimeStep = 0.05;

        // get the Aqp and Bqp
        const int AqpRowSize = predHorizon * 13;
        const int AqpColSize = 13;
        const int BqpSize = predHorizon * 12;
        Eigen::Matrix<float, 3, 4> footInBaseFrame = quadruped->GetFootPositionsInBaseFrame();
        Quat<float> quat = quadruped->GetBaseOrientation();
        Vec3<float> rpy =  quadruped->GetBaseRollPitchYaw();
        Eigen::Matrix<float, 3, 4> footInWorldFrame = robotics::math::invertRigidTransform({0.f, 0.f, 0.f}, quat, footInBaseFrame);
        std::tuple<Eigen::Matrix<float, AqpRowSize, AqpColSize>, Eigen::Matrix<float, AqpRowSize, BqpSize>> ABqp;
        ABqp = Quadruped::ComputeMassMatrixMPC(quadruped->bodyMass,
                                            quadruped->bodyInertia,
                                            footInWorldFrame.transpose(),
                                            rpy);
        Eigen::Matrix<float, AqpRowSize, AqpColSize> Aqp;
        Eigen::Matrix<float, AqpRowSize, BqpSize> Bqp;
        Aqp = std::get<0>(ABqp);
        Bqp = std::get<1>(ABqp);
        // get the G and a
        std::tuple<Eigen::Matrix<float, BqpSize, BqpSize>, Eigen::Matrix<float, BqpSize, 1>> Ga;
        Eigen::Matrix<float, BqpSize, BqpSize> G;
        Eigen::Matrix<float, BqpSize, 1> a;
        Ga = ComputeObjectiveMatrixMPC(Aqp, Bqp, X, desiredX, XWeight, regWeight);
        G = std::get<0>(Ga);
        // G = G + temp * 1e-6;
        a = std::get<1>(Ga);
        Eigen::Matrix<float, BqpSize, BqpSize> temp; temp.setIdentity();
        
        // get the Ci and b
        const int rowSizeCi = predHorizon*4*6;
        const int colSizebi = predHorizon*4*3;
        std::tuple<Eigen::Matrix<float, colSizebi, rowSizeCi>, Eigen::Matrix<float, rowSizeCi, 1>> CI;
        Eigen::Matrix<float, colSizebi, rowSizeCi> Ci;
        Eigen::Matrix<float, rowSizeCi, 1> bi;
        CI = ComputeConstraintMatrixMPC(quadruped->bodyMass, contacts, frictionCoef, fMinRatio, fMaxRatio);
        Ci = std::get<0>(CI);
        bi = std::get<1>(CI);

        quadprogpp::Matrix<double> GG(BqpSize, BqpSize);

        for (int i = 0; i < BqpSize; i++) {
            for (int j = 0; j < BqpSize; j++) {
                GG[i][j] = double(G(i, j));
            }
        }
        quadprogpp::Vector<double> aa(BqpSize);
        for (int i = 0; i < BqpSize; i++) {
            aa[i] = double(a(i, 0));
        }
        quadprogpp::Matrix<double> CICI(colSizebi, rowSizeCi);
        for (int i = 0; i < colSizebi; i++) {
            for (int j = 0; j < rowSizeCi; j++) {
                CICI[i][j] = double(Ci(i, j));
            }
        }
        quadprogpp::Vector<double> bb(rowSizeCi);
        for (int i = 0; i < rowSizeCi; i++) {
            bb[i] = double(-bi(i, 0));
        }
        quadprogpp::Matrix<double> CECE(BqpSize, 0);
        quadprogpp::Vector<double> ee(0);
        quadprogpp::Vector<double> xMPC(BqpSize);
        quadprogpp::solve_quadprog(GG, aa, CECE, ee, CICI, bb, xMPC);
        // take the first 12*ctrlHorizon elements of xMPC and reshape it as a matrix with shape (4,3*ctrlHorizon)
        Eigen::Matrix<float, 4, 3*predHorizon> force;
        for (int k=0; k<predHorizon; k++){
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 3; ++j) {
                    force(i, k*3 + j) = -float(xMPC[12*k + 3 * i + j]);
                }
            }
        }
        return force.transpose();
    }
}
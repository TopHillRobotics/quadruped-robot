/* 
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.        
* Description: compute the force for stance controller. 
* Author: Zang Yaohua
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zang Yaohua
*/

#include "controllers/balance_controller/qp_torque_optimizer.h"
#include "QuadProg++.hh"
#include "Array.hh"
// #include "OsqpEigen/OsqpEigen.h"

namespace Quadruped {
    /** @brief
     * @param robotMass : float, ture mass of robot.
     * @param robotInertia : Eigen::Matrix<float, 3, 3>, should expressed in control frame.
     * @param footPositions : Eigen::Matrix<float, 4, 3>, should expressed in control frame.
     * @return massMat : Eigen::Matrix<float, 6, 12>, in control frame.
     */
    Eigen::Matrix<float, 6, 12> ComputeMassMatrix(float robotMass,
                                                  Eigen::Matrix<float, 3, 3> robotInertia,
                                                  Eigen::Matrix<float, 4, 3> footPositions)
    {
        Eigen::Matrix<float, 3, 3> I = Eigen::Matrix<float, 3, 3>::Identity(3, 3);
        Eigen::Matrix<float, 3, 3> invMass;
        Eigen::Matrix<float, 3, 3> invInertia; // in control frame
        Eigen::Matrix<float, 6, 12> massMat = Eigen::Matrix<float, 6, 12>::Zero();
        Eigen::Matrix<float, 1, 3> x;
        Eigen::Matrix<float, 3, 3> footPositionSkew;

        invMass = I / robotMass;
        invInertia = robotInertia.inverse();

        for (int legId = 0; legId < 4; ++legId) {
            massMat.block<3, 3>(0, legId * 3) = invMass;
            x = footPositions.row(legId);
            footPositionSkew << 0., -x[2], x[1],
                                x[2], 0., -x[0],
                                -x[1], x[0], 0.;
            massMat.block<3, 3>(3, legId * 3) = invInertia * footPositionSkew;
        }
        return massMat;
    }

    std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> ComputeConstraintMatrix(
        float mpcBodyMass,
        Eigen::Matrix<bool, 4, 1> contacts,
        float frictionCoef,
        float fMinRatio,
        float fMaxRatio,
        Vec3<float> surfaceNormal)
    {
        float fMin;
        float fMax;
        fMin = fMinRatio * mpcBodyMass * 9.8;
        fMax = fMaxRatio * mpcBodyMass * 9.8;
        Eigen::Matrix<float, 24, 12> A = Eigen::Matrix<float, 24, 12>::Zero();
        Eigen::Matrix<float, 24, 1> lb = Eigen::Matrix<float, 24, 1>::Zero();

        // constrains on normal component of friction
        /*
        for (int legId = 0; legId < 4; legId++) {
            A(legId * 2, legId * 3 + 2) = 1;
            A(legId * 2 + 1, legId * 3 + 2) = -1;
            if (contacts(legId)) {
                lb(legId * 2, 0) = fMin;
                lb(legId * 2 + 1, 0) = -fMax;
            } else {
                lb(legId * 2, 0) = 1e-7;
                lb(legId * 2 + 1, 0) = 1e-7;
            }
        }
        // Friction cone constraints
        int rowId;
        int colId;
        for (int legId = 0; legId < 4; ++legId) {
            rowId = 8 + legId * 4;
            colId = legId * 3;
            lb.block<4, 1>(rowId, 0) << 0., 0., 0., 0.;
            A.block<1, 3>(rowId,     colId) <<  1,  0, frictionCoef;
            A.block<1, 3>(rowId + 1, colId) << -1,  0, frictionCoef;
            A.block<1, 3>(rowId + 2, colId) <<  0,  1, frictionCoef;
            A.block<1, 3>(rowId + 3, colId) <<  0, -1, frictionCoef;
        }
        */
        //  Friction cone are not parallel with z-axis of control frame
        for (int legId = 0; legId < 4; legId++) {
            A.block<1,3>(legId*2, legId*3) = surfaceNormal;
            A.block<1,3>(legId*2+1, legId*3) = -surfaceNormal;
            if (contacts[legId] > 0) {
                lb(legId * 2, 0) = fMin;
                lb(legId * 2 + 1, 0) = -fMax;
            } else {
                lb(legId * 2, 0) = 1e-7;
                lb(legId * 2 + 1, 0) = 1e-7;
            }
        }
        int rowId;
        int colId;
        
        Vec3<float> tangent2 = {0.f, 1.f, 0.f};
        Vec3<float> tangent1 = tangent2.cross(surfaceNormal);       
        for (int legId = 0; legId < 4; ++legId) {
            rowId = 8 + legId * 4;
            colId = legId * 3;
            lb.block<4, 1>(rowId, 0) << 0., 0., 0., 0.;
            A.block<1, 3>(rowId, colId) << (frictionCoef*surfaceNormal + tangent1).transpose();
            A.block<1, 3>(rowId + 1, colId) << (frictionCoef*surfaceNormal - tangent1).transpose();
            A.block<1, 3>(rowId + 2, colId) << (frictionCoef*surfaceNormal + tangent2).transpose();
            A.block<1, 3>(rowId + 3, colId) << (frictionCoef*surfaceNormal - tangent2).transpose();
        }
        
        std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> Alb(A.transpose(), lb);
        return Alb;
    }

    std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> ComputeObjectiveMatrix(
        Eigen::Matrix<float, 6, 12> massMatrix,
        Eigen::Matrix<float, 6, 1> desiredAcc,
        Eigen::Matrix<float, 6, 1> accWeight,
        float regWeight,
        Eigen::Matrix<float, 6, 1> g)
    {
        Eigen::Matrix<float, 6, 6> Q = Eigen::Matrix<float, 6, 6>::Zero();
        for (int i = 0; i < 6; ++i) {
            Q(i, i) = accWeight(i, 0);
        }
        Eigen::Matrix<float, 12, 12> R = Eigen::Matrix<float, 12, 12>::Ones();
        R = R * regWeight;
        Eigen::Matrix<float, 12, 12> quadTerm;
        Eigen::Matrix<float, 12, 1> linearTerm;
        quadTerm = massMatrix.transpose() * Q * massMatrix + R;
        linearTerm = (g + desiredAcc).transpose() * Q * massMatrix;
        std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> quadLinear(quadTerm, linearTerm);
        return quadLinear;
    }

    Eigen::Matrix<float,12,12> ComputeWeightMatrix(Robot *robot, const Eigen::Matrix<bool, 4, 1>& contacts)
    {
        Eigen::Matrix<float,12,12> W = 1e-4 * Eigen::Matrix<float,12,12>::Identity(); // 1e-4
        // regularize the joint torque
        // for(int i=0; i<4; ++i) {
        //     if(contacts[i]) {
        //         const Eigen::Matrix<float, 3, 3>& jv = robot->stateDataFlow.footJvs[i];
        //         //auto Jv = robot->ComputeJacobian(i);
        //         W.block<3,3>(3*i,3*i) = Jv * W.block<3,3>(3*i,3*i) * Jv.transpose();
        //     }
        // }
        return W;
    }

    Eigen::Matrix<float, 3, 4> ComputeContactForce(Robot *robot,
                                                   GroundSurfaceEstimator* groundEstimator,
                                                   Eigen::Matrix<float, 6, 1> desiredAcc,
                                                   Eigen::Matrix<bool, 4, 1> contacts,
                                                   Eigen::Matrix<float, 6, 1> accWeight,
                                                   float regWeight,
                                                   float frictionCoef,
                                                   float fMinRatio,
                                                   float fMaxRatio)
    {
        
        Quat<float> quat = robot->GetBaseOrientation();
        Vec3<float> controlFrameRPY = groundEstimator->GetControlFrameRPY();
        Mat3<float> rotMatControlFrame = groundEstimator->GetAlignedDirections();
        Vec3<float> surfaceNormal = {0.f, 0.f, 1.f};
        // std::cout<< "surfaceNormal = "<< surfaceNormal.transpose()  << std::endl;
        Eigen::Matrix<float, 6, 1> g = Eigen::Matrix<float, 6, 1>::Zero();
        g(2, 0) = 9.8;
        TerrainType& goundType = groundEstimator->terrain.terrainType;
        Mat3<float> Rcb;
        if (goundType == TerrainType::PLANE || goundType==TerrainType::PLUM_PILES) {
            Rcb = Mat3<float>::Identity(); // control in base frame
        } else {
            // printf("compute torque in control frame!\n");
            // convert inertia in base frame to confrol frame
            Rcb = rotMatControlFrame.transpose() * robotics::math::quaternionToRotationMatrix(quat).transpose();
            // convert g from world frame to control frame
            g.head(3) = rotMatControlFrame.transpose() * g.head(3);
            surfaceNormal << -std::sin(controlFrameRPY[1]), 0.f, std::cos(controlFrameRPY[1]); 
            // fMaxRatio = fMaxRatio * cos(-controlFrameRPY[1]); // todo
        }
        Mat3<float> inertia = Rcb * robot->totalInertia * Rcb.transpose();
        Eigen::Matrix<float,3,4> footPosition = Rcb * robot->GetFootPositionsInBaseFrame();                
        Eigen::Matrix<float, 6, 12> massMatrix = ComputeMassMatrix(robot->totalMass, // compute in control frame or base frame, according to terrain.
                                                                    inertia,
                                                                    footPosition.transpose());
        
        std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> Ga;
        Ga = ComputeObjectiveMatrix(massMatrix, desiredAcc, accWeight, regWeight, g);
        Eigen::Matrix<float, 12, 12> G = std::get<0>(Ga);
        Eigen::Matrix<float,12,12> W = ComputeWeightMatrix(robot, contacts);
        G = G + W;
        Eigen::Matrix<float, 12, 1> a = std::get<1>(Ga);
        std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> CI;
        Eigen::Matrix<float, 12, 24> Ci;
        Eigen::Matrix<float, 24, 1> b;
        CI = ComputeConstraintMatrix(robot->totalMass, contacts, frictionCoef, fMinRatio, fMaxRatio, surfaceNormal);
        Ci = std::get<0>(CI);
        b = std::get<1>(CI);

        quadprogpp::Matrix<double> GG(12, 12);
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 12; j++) {
                GG[i][j] = double(G(j, i));
            }
        }
        quadprogpp::Vector<double> aa(12);
        for (int i = 0; i < 12; i++) {
            aa[i] = double(-a(i, 0));
        }
        quadprogpp::Matrix<double> CICI(12, 24);
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 24; j++) {
                CICI[i][j] = double(Ci(i, j));
            }
        }
        quadprogpp::Vector<double> bb(24);
        for (int i = 0; i < 24; i++) {
            bb[i] = double(-b(i, 0));
        }
        quadprogpp::Matrix<double> CECE(12, 0);
        quadprogpp::Vector<double> ee(0);
        quadprogpp::Vector<double> x(12);
        quadprogpp::solve_quadprog(GG, aa, CECE, ee, CICI, bb, x);
        //reshape x from (12,) to (4,3)
        Eigen::Matrix<float, 4, 3> X;
        int invalidResNum = 0;
        for (int index = 0; index < x.size(); index++) {
            if (isnan(x[index])) {
                invalidResNum++;
            }
        }
        if (invalidResNum > 0) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 3; ++j) {
                    X(i, j) = 0.f;
                }
            }
            std::cout << "[QP solver] No solution :" << invalidResNum << std::endl;
        } else {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 3; ++j) {
                    X(i, j) = -float(x[3 * i + j]);
                }
            }
        }
        
        return (X*Rcb).transpose(); // convert force to current base frame
    }
    
    /*
    Eigen::VectorXd OSQPTest(Eigen::Matrix<float,12,12>& G, Eigen::Matrix<float,12,1>& a, 
                            Eigen::Matrix<float,12,24>& Ci, Eigen::Matrix<float,24,1>& b)
    {
        // allocate QP problem matrices and vectors
        Eigen::SparseMatrix<double> hessian;
        Eigen::VectorXd gradient;
        Eigen::SparseMatrix<double> linearMatrix;
        Eigen::VectorXd lowerBound;
        Eigen::VectorXd upperBound;
        upperBound.resize(24);
        upperBound.array() = OsqpEigen::INFTY;
        // std::cout << "upperBound = " << upperBound << std::endl;
        lowerBound.resize(24);
        for (int i=0; i < 24; ++i)
            lowerBound[i] = double(b[i]);//Eigen::Map<Eigen::VectorXf>(b, 24).cast<double>();
        // std::cout << "lowerBound = " << lowerBound << std::endl;
        
        hessian.resize(12,12);
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 12; j++) {
                if (abs(G(j,i)) > 0.01)
                    hessian.insert(j,i) = (double)G(i, j);
            }
        }
        gradient.resize(12);
        gradient = -a.cast<double>();// -Eigen::Map<Eigen::VectorXd>(a, 12);
        
        linearMatrix.resize(24,12);
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 24; j++) {
                if (abs(Ci(i,j)) > 0.01)
                    linearMatrix.insert(j,i) = double(Ci(i, j));
            }
        }

        // std::cout <<"linearMatrix = " << Eigen::MatrixXd(linearMatrix) << std::endl;

         OsqpEigen::Solver solver;
         solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        // set the initial data of the QP solver
        solver.data()->setNumberOfVariables(12);
        solver.data()->setNumberOfConstraints(24);
        if(!solver.data()->setHessianMatrix(hessian)) throw std::domain_error("293");
        if(!solver.data()->setGradient(gradient)) throw std::domain_error("294");
        if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) throw std::domain_error("295");
        if(!solver.data()->setLowerBound(lowerBound)) throw std::domain_error("296");
        if(!solver.data()->setUpperBound(upperBound)) throw std::domain_error("297");
        // std::cout << "init solver --"<<std::endl;
        // instantiate the solver
        if(!solver.initSolver()) throw std::domain_error("300");
        // std::cout << "solve solver --"<<std::endl;
        solver.solveProblem();
        // std::cout << "end solver --"<<std::endl;
        // controller input and QPSolution vector
        Eigen::VectorXd QPSolution = solver.getSolution(); //12x1
        // std::cout << "solve = " << QPSolution <<std::endl;
        return QPSolution;

    }

    Eigen::Matrix<float, 3, 4> ComputeContactForceTest(Robot *robot,
                                                   GroundSurfaceEstimator* groundEstimator,
                                                   Eigen::Matrix<float, 6, 1> desiredAcc,
                                                   Eigen::Matrix<bool, 4, 1> contacts,
                                                   Eigen::Matrix<float, 6, 1> accWeight,
                                                   float regWeight,
                                                   float frictionCoef,
                                                   float fMinRatio,
                                                   float fMaxRatio)
    {
        
        Quat<float> quat = robot->GetBaseOrientation();
        Vec3<float> controlFrameRPY = groundEstimator->GetControlFrameRPY();
        Mat3<float> rotMatControlFrame = groundEstimator->GetAlignedDirections();
        Vec3<float> surfaceNormal = {0.f, 0.f, 1.f};
        // std::cout<< "surfaceNormal = "<< surfaceNormal.transpose()  << std::endl;
        Eigen::Matrix<float, 6, 1> g = Eigen::Matrix<float, 6, 1>::Zero();
        g(2, 0) = 9.8;
        TerrainType& goundType = groundEstimator->terrain.terrainType;
        Mat3<float> Rcb;
        if (goundType == TerrainType::PLANE || goundType==TerrainType::PLUM_PILES) {
            Rcb = Mat3<float>::Identity(); // control in base frame
        } else {
            // printf("compute torque in control frame!\n");
            // convert inertia in base frame to confrol frame
            Rcb = rotMatControlFrame.transpose() * robotics::math::quaternionToRotationMatrix(quat).transpose();
            // convert g from world frame to control frame
            g.head(3) = rotMatControlFrame.transpose() * g.head(3);
            surfaceNormal << -std::sin(controlFrameRPY[1]), 0.f, std::cos(controlFrameRPY[1]); 
            // fMaxRatio = fMaxRatio * cos(-controlFrameRPY[1]); // todo
        }
        Mat3<float> inertia = Rcb * robot->totalInertia * Rcb.transpose();
        Eigen::Matrix<float,3,4> footPosition = Rcb * robot->GetFootPositionsInBaseFrame();                
        Eigen::Matrix<float, 6, 12> massMatrix = ComputeMassMatrix(robot->totalMass, // compute in control frame or base frame, according to terrain.
                                                                    inertia,
                                                                    footPosition.transpose());
        
        std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> Ga;
        Ga = ComputeObjectiveMatrix(massMatrix, desiredAcc, accWeight, regWeight, g);
        Eigen::Matrix<float, 12, 12> G = std::get<0>(Ga);
        Eigen::Matrix<float,12,12> W = ComputeWeightMatrix(robot, contacts);
        G = G + W;
        Eigen::Matrix<float, 12, 1> a = std::get<1>(Ga);
        std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> CI;
        Eigen::Matrix<float, 12, 24> Ci;
        Eigen::Matrix<float, 24, 1> b;
        CI = ComputeConstraintMatrix(robot->totalMass, contacts, frictionCoef, fMinRatio, fMaxRatio, surfaceNormal);
        Ci = std::get<0>(CI);
        b = std::get<1>(CI);

        
        
        Eigen::VectorXd x = OSQPTest(G, a,Ci, b);
        
        //reshape x from (12,) to (4,3)
        Eigen::Matrix<float, 4, 3> X;
        int invalidResNum = 0;
        for (int index = 0; index < x.size(); index++) {
            if (isnan(x[index])) {
                invalidResNum++;
            }
        }
        if (invalidResNum > 0) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 3; ++j) {
                    X(i, j) = 0.f;
                }
            }
            std::cout << "[QP solver] No solution :" << invalidResNum << std::endl;
        } else {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 3; ++j) {
                    X(i, j) = -float(x[3 * i + j]);
                }
            }
        }
        
        return (X*Rcb).transpose(); // convert force to current base frame
    }
    */
    

    /** @brief Writen by Zhu Yijie, in world frame. Used for climbing stairs or slopes. */ 
    Eigen::Matrix<float, 3, 4> ComputeContactForce(Robot *robot,
                                                    Eigen::Matrix<float, 6, 1> desiredAcc,
                                                    Eigen::Matrix<bool, 4, 1> contacts,
                                                    Eigen::Matrix<float, 6, 1> accWeight,
                                                    Vec3<float> normal,
                                                    Vec3<float> tangent1,
                                                    Vec3<float> tangent2,
                                                    Vec4<float> fMinRatio,
                                                    Vec4<float> fMaxRatio,
                                                    float regWeight,
                                                    float frictionCoef)
    {
        Quat<float> quat = robot->GetBaseOrientation();
        Eigen::Matrix<float,3,4> footPositionsInBaseFrame = robot->GetFootPositionsInBaseFrame();
        Mat3<float> rotMat = robotics::math::quaternionToRotationMatrix(quat).transpose();
        Eigen::Matrix<float, 3, 4> footPositionsInCOMWorldFrame = robotics::math::invertRigidTransform<float,4>({0.f,0.f,0.f},quat, footPositionsInBaseFrame);
        Eigen::Matrix<float, 6, 12> massMatrix = ComputeMassMatrix(robot->totalMass,
                                                                    robot->totalInertia,
                                                                    footPositionsInCOMWorldFrame.transpose(),
                                                                    rotMat);
        std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> Ga;
        Eigen::Matrix<float, 6, 1> g = Eigen::Matrix<float, 6, 1>::Zero();
        g(2, 0) = 9.8;
        Ga = ComputeObjectiveMatrix(massMatrix, desiredAcc, accWeight, regWeight, g);
        Eigen::Matrix<float, 12, 12> G = std::get<0>(Ga);
        Eigen::Matrix<float,12,12> W = ComputeWeightMatrix(robot, contacts);
        G = G + W;
        Eigen::Matrix<float, 12, 1> a = std::get<1>(Ga);
        
        std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> CI;
        Eigen::Matrix<float, 12, 24> Ci;
        Eigen::Matrix<float, 24, 1> b;
        CI = ComputeConstraintMatrix(robot->totalMass, contacts, frictionCoef, fMinRatio, fMaxRatio, normal, tangent1, tangent2);
        Ci = std::get<0>(CI);
        b = std::get<1>(CI);

        quadprogpp::Matrix<double> GG(12, 12);
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 12; j++) {
                GG[i][j] = double(G(j, i));
            }
        }
        quadprogpp::Vector<double> aa(12);
        for (int i = 0; i < 12; i++) {
            aa[i] = double(-a(i, 0));
        }
        quadprogpp::Matrix<double> CICI(12, 24);
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 24; j++) {
                CICI[i][j] = double(Ci(i, j));
            }
        }
        quadprogpp::Vector<double> bb(24);
        for (int i = 0; i < 24; i++) {
            bb[i] = double(-b(i, 0));
        }
        quadprogpp::Matrix<double> CECE(12, 0);
        quadprogpp::Vector<double> ee(0);
        quadprogpp::Vector<double> x(12);
        quadprogpp::solve_quadprog(GG, aa, CECE, ee, CICI, bb, x);
        //reshape x from (12,) to (4,3)
        Eigen::Matrix<float, 4, 3> X;
        int invalidResNum = 0;
        for (int index = 0; index < x.size(); index++) {
            if (isnan(x[index])) {
                invalidResNum++;
            }
        }
        if (invalidResNum > 0) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 3; ++j) {
                    X(i, j) = 0.f;
                }
            }
            std::cerr << "[QP solver] No solution :" << invalidResNum << std::endl;
            throw std::domain_error("qp torque");
        } else {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 3; ++j) {
                    X(i, j) = -float(x[3 * i + j]);
                }
            }
        }
        return robotics::math::RigidTransform<float,4>({0.f,0.f,0.f}, quat, X.transpose());
    }
    

    /** @brief Writen by Zhu Yijie, in world frame. Used for climbing stairs or slopes. */ 
    Eigen::Matrix<float, 6, 12> ComputeMassMatrix(float robotMass,
                                                  Eigen::Matrix<float, 3, 3> robotInertia,
                                                  Eigen::Matrix<float, 4, 3> footPositions,
                                                  Mat3<float> rotMat)
    {
        Eigen::Matrix<float, 3, 3> I = Eigen::Matrix<float, 3, 3>::Identity(3, 3);
        Eigen::Matrix<float, 3, 3> invMass;
        Eigen::Matrix<float, 3, 3> invInertiaInWorld;
        Eigen::Matrix<float, 6, 12> massMat = Eigen::Matrix<float, 6, 12>::Zero();
        Eigen::Matrix<float, 1, 3> x;
        Eigen::Matrix<float, 3, 3> footPositionSkew;

        invMass = I / robotMass;
        Mat3<float> robotInertiaInWorld = rotMat * robotInertia * rotMat.transpose();
        invInertiaInWorld = robotInertiaInWorld.inverse();

        for (int legId = 0; legId < 4; ++legId) {
            massMat.block<3, 3>(0, legId * 3) = invMass;
            x = footPositions.row(legId);
            footPositionSkew << 0., -x[2], x[1],
                                x[2], 0., -x[0],
                                -x[1], x[0], 0.;
            massMat.block<3, 3>(3, legId * 3) = invInertiaInWorld * footPositionSkew;
        }
        return massMat;
    }


    /** @brief Writen by Zhu Yijie, in world frame. Used for climbing stairs or slopes. */
    std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> ComputeConstraintMatrix(
                                                                            float mpcBodyMass,
                                                                            Eigen::Matrix<bool, 4, 1> contacts,
                                                                            float frictionCoef,
                                                                            Vec4<float> fMinRatio,
                                                                            Vec4<float> fMaxRatio,
                                                                            Vec3<float> normal,
                                                                            Vec3<float> tangent1,
                                                                            Vec3<float> tangent2)
    { 
        Eigen::Matrix<float, 24, 12> A = Eigen::Matrix<float, 24, 12>::Zero();
        Eigen::Matrix<float, 24, 1> lb = Eigen::Matrix<float, 24, 1>::Zero();

        for (int legId = 0; legId < 4; legId++) {
            A.block<1,3>(legId*2, legId*3) = normal;
            A.block<1,3>(legId*2+1, legId*3) = -normal;
            if (contacts[legId] > 0) {
                lb(legId * 2, 0) = fMinRatio[legId] * mpcBodyMass * 9.8;
                lb(legId * 2 + 1, 0) = -fMaxRatio[legId] * mpcBodyMass * 9.8;
            } else {
                lb(legId * 2, 0) = 1e-7;
                lb(legId * 2 + 1, 0) = 1e-7;
            }
        }
        // Friction cone constraints not parallel with world frame.
        int rowId;
        int colId;
        for (int legId = 0; legId < 4; ++legId) {
            rowId = 8 + legId * 4;
            colId = legId * 3;
            lb.block<4, 1>(rowId, 0) << 0., 0., 0., 0.;
            A.block<1, 3>(rowId, colId) << (frictionCoef*normal + tangent1).transpose();
            A.block<1, 3>(rowId + 1, colId) << (frictionCoef*normal - tangent1).transpose();
            A.block<1, 3>(rowId + 2, colId) << (frictionCoef*normal + tangent2).transpose();
            A.block<1, 3>(rowId + 3, colId) << (frictionCoef*normal - tangent2).transpose();
        }
        std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> Alb(A.transpose(), lb);
        return Alb;
    }
} // namespace Quadruped

// The MIT License

// Copyright (c) 2022 
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

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

#include <QuadProg++.hh>
#include <Array.hh>

#include "common/qr_qp_torque_optimizer.h"
#include "common/qr_eigen_types.h"

Eigen::Matrix<float, 6, 12> ComputeMassMatrix(float robotMass,
                                              Mat3<float> robotInertia,
                                              Eigen::Matrix<float, 4, 3> footPositions)
{
    Mat3<float> I = Mat3<float>::Identity(3, 3);
    Mat3<float> invMass;
    Mat3<float> invInertia; // in control frame
    Eigen::Matrix<float, 6, 12> massMat = Eigen::Matrix<float, 6, 12>::Zero();
    Eigen::Matrix<float, 1, 3> x;
    Mat3<float> footPositionSkew;

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

std::tuple<Eigen::Matrix<float, 12, 24>, Vec24<float>> ComputeConstraintMatrix(
    float mpcBodyMass,
    Eigen::Matrix<bool, 4, 1> contacts,
    float frictionCoef,
    float fMinRatio,
    float fMaxRatio)
{
    float fMin;
    float fMax;
    fMin = fMinRatio * mpcBodyMass * 9.8;
    fMax = fMaxRatio * mpcBodyMass * 9.8;
    Eigen::Matrix<float, 24, 12> A = Eigen::Matrix<float, 24, 12>::Zero();
    Vec24<float> lb = Vec24<float>::Zero();

    // constrains on normal component of friction
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
    std::tuple<Eigen::Matrix<float, 12, 24>, Vec24<float>> Alb(A.transpose(), lb);
    return Alb;
}

std::tuple<Mat12<float>, Vec12<float>> ComputeObjectiveMatrix(
    Eigen::Matrix<float, 6, 12> massMatrix,
    Vec6<float> desiredAcc,
    Vec6<float> accWeight,
    float regWeight,
    Vec6<float> g)
{
    Mat6<float> Q = Mat6<float>::Zero();
    for (int i = 0; i < 6; ++i) {
        Q(i, i) = accWeight(i, 0);
    }
    Mat12<float> R = Mat12<float>::Ones();
    R = R * regWeight;
    Mat12<float> quadTerm;
    Vec12<float> linearTerm;
    quadTerm = massMatrix.transpose() * Q * massMatrix + R;
    linearTerm = (g + desiredAcc).transpose() * Q * massMatrix;
    std::tuple<Mat12<float>, Vec12<float>> quadLinear(quadTerm, linearTerm);
    return quadLinear;
}

Eigen::Matrix<float,12,12> ComputeWeightMatrix(qrRobot *robot, const Eigen::Matrix<bool, 4, 1>& contacts)
{
    Eigen::Matrix<float,12,12> W = 1e-4 * Eigen::Matrix<float,12,12>::Identity(); // 1e-4
    return W;
}

Mat3x4<float> ComputeContactForce(qrRobot *robot,
                                               qrGroundSurfaceEstimator* groundEstimator,
                                               Vec6<float> desiredAcc,
                                               Eigen::Matrix<bool, 4, 1> contacts,
                                               Vec6<float> accWeight,
                                               float regWeight,
                                               float frictionCoef,
                                               float fMinRatio,
                                               float fMaxRatio)
{    
    Quat<float> quat = robot->GetRobotState()->GetBaseOrientation();
    Vec3<float> controlFrameRPY = groundEstimator->GetControlFrameRPY();
    Mat3<float> rotMatControlFrame = groundEstimator->GetAlignedDirections();
    Vec6<float> g = Vec6<float>::Zero();
    g(2, 0) = 9.8;
    TerrainType groundType = groundEstimator->GetTerrain().terrainType;
    Mat3<float> rotMat;
    if (groundType == TerrainType::PLANE || groundType == TerrainType::PLUM_PILES) {
        rotMat = Mat3<float>::Identity(); // control in base frame
    } else {
        // convert inertia in base frame to confrol frame
        rotMat = rotMatControlFrame.transpose() * math::Quat2RotMat(quat);
        // convert g from world frame to control frame
        g.head(3) = rotMatControlFrame.transpose() * g.head(3);
        fMaxRatio = fMaxRatio * cos(-controlFrameRPY[1]);
    }
    Mat3<float> inertia = rotMat * robot->GetRobotConfig()->GetBodyInertia() * rotMat.transpose();
    Mat3x4<float> footPosition = rotMat * robot->GetRobotState()->GetFootPositionInBaseFrame();
    Eigen::Matrix<float, 6, 12> massMatrix = ComputeMassMatrix(robot->GetRobotConfig()->GetBodyMass(), // compute in control frame or base frame, according to terrain.
                                                               inertia,
                                                               footPosition.transpose());
    
    std::tuple<Mat12<float>, Vec12<float>> Ga;
    Ga = ComputeObjectiveMatrix(massMatrix, desiredAcc, accWeight, regWeight, g);
    Mat12<float> G = std::get<0>(Ga);
    Eigen::Matrix<float,12,12> W = ComputeWeightMatrix(robot, contacts);
    G = G + W;
    Vec12<float> a = std::get<1>(Ga);
    std::tuple<Eigen::Matrix<float, 12, 24>, Vec24<float>> CI;
    Eigen::Matrix<float, 12, 24> Ci;
    Vec24<float> b;
    CI = ComputeConstraintMatrix(robot->GetRobotConfig()->GetBodyMass(), contacts, frictionCoef, fMinRatio, fMaxRatio);
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
        if (std::isnan(x[index])) {
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
    
    return (X*rotMat).transpose(); // convert force to current base frame
}

Mat3x4<float> ComputeContactForce(qrRobot *robot,
                                               Vec6<float> desiredAcc,
                                               Eigen::Matrix<bool, 4, 1> contacts,
                                               Vec6<float> accWeight,
                                               Vec3<float> normal,
                                               Vec3<float> tangent1,
                                               Vec3<float> tangent2,
                                               Vec4<float> fMinRatio,
                                               Vec4<float> fMaxRatio,
                                               float regWeight,
                                               float frictionCoef)
{
    Quat<float> quat = robot->GetRobotState()->GetBaseOrientation();
    Mat3x4<float> footPositionsInBaseFrame = robot->GetRobotState()->GetFootPositionInBaseFrame();
    Mat3<float> rotMat = math::Quat2RotMat(quat);
    Mat3x4<float> footPositionsInCOMWorldFrame = math::InvertRigidTransform<float,4>({0.f,0.f,0.f},quat, footPositionsInBaseFrame);
    Eigen::Matrix<float, 6, 12> massMatrix = ComputeMassMatrix(robot->GetRobotConfig()->GetBodyMass(),
                                                               robot->GetRobotConfig()->GetBodyInertia(),
                                                               footPositionsInCOMWorldFrame.transpose(),
                                                               rotMat);
    std::tuple<Mat12<float>, Vec12<float>> Ga;
    Vec6<float> g = Vec6<float>::Zero();
    g(2, 0) = 9.8;
    Ga = ComputeObjectiveMatrix(massMatrix, desiredAcc, accWeight, regWeight, g);
    Mat12<float> G = std::get<0>(Ga);
    Eigen::Matrix<float,12,12> W = ComputeWeightMatrix(robot, contacts);
    G = G + W;
    Vec12<float> a = std::get<1>(Ga);
    
    std::tuple<Eigen::Matrix<float, 12, 24>, Vec24<float>> CI;
    Eigen::Matrix<float, 12, 24> Ci;
    Vec24<float> b;
    CI = ComputeConstraintMatrix(robot->GetRobotConfig()->GetBodyMass(), contacts, frictionCoef, fMinRatio, fMaxRatio, normal, tangent1, tangent2);
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
        if (std::isnan(x[index])) {
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
    return math::RigidTransform<float,4>({0.f,0.f,0.f}, quat, X.transpose());
}

Eigen::Matrix<float, 6, 12> ComputeMassMatrix(float robotMass,
                                                Mat3<float> robotInertia,
                                                Eigen::Matrix<float, 4, 3> footPositions,
                                                Mat3<float> rotMat)
{
    Mat3<float> I = Mat3<float>::Identity(3, 3);
    Mat3<float> invMass;
    Mat3<float> invInertiaInWorld;
    Eigen::Matrix<float, 6, 12> massMat = Eigen::Matrix<float, 6, 12>::Zero();
    Eigen::Matrix<float, 1, 3> x;
    Mat3<float> footPositionSkew;

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

std::tuple<Eigen::Matrix<float, 12, 24>, Vec24<float>> ComputeConstraintMatrix(
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
    Vec24<float> lb = Vec24<float>::Zero();

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
    std::tuple<Eigen::Matrix<float, 12, 24>, Vec24<float>> Alb(A.transpose(), lb);
    return Alb;
}

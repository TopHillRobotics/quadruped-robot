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

#include "controllers/balance_controller/qr_qp_torque_optimizer.h"
#include "QuadProg++.hh"
#include "Array.hh"

namespace Quadruped {

Eigen::Matrix<float, 6, 12> ComputeMassMatrix(
    float robotMass,
    Eigen::Matrix<float, 3, 3> robotInertia,
    Eigen::Matrix<float, 4, 3> footPositions)
{
    Eigen::Matrix<float, 3, 3> I = Eigen::Matrix<float, 3, 3>::Identity(3, 3);
    Eigen::Matrix<float, 3, 3> invMass;
    Eigen::Matrix<float, 3, 3> invInertia;
    Eigen::Matrix<float, 6, 12> massMat = Eigen::Matrix<float, 6, 12>::Zero();
    Eigen::Matrix<float, 1, 3> x;


    invMass = I / robotMass;
    invInertia = robotInertia.inverse();

    /* The matrix is in the form of:
     * | 1/M                 , 1/M,                , 1/M                 , 1/M | (3 * 12)
     * | I^-1 * foot[0].cross, I^-1 * foot[1].cross, I^-1 * foot[2].cross, I^-1 * foot[3].cross | (3 * 12)
     */
    Eigen::Matrix<float, 3, 3> footPositionSkew;
    for (int legId = 0; legId < 4; ++legId) {
        massMat.block<3, 3>(0, legId * 3) = invMass;
        x = footPositions.row(legId);
        footPositionSkew <<  0.,   -x[2],  x[1],
                             x[2],  0.,   -x[0],
                            -x[1],  x[0],  0.;
        massMat.block<3, 3>(3, legId * 3) = invInertia * footPositionSkew;
    }
    return massMat;
}


std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> ComputeConstraintMatrix(
    float bodyMass,
    Eigen::Matrix<bool, 4, 1> contacts,
    float frictionCoef,
    float fMinRatio,
    float fMaxRatio,
    Vec3<float> surfaceNormal)
{
    float fMin;
    float fMax;
    fMin = fMinRatio * bodyMass * 9.8f;
    fMax = fMaxRatio * bodyMass * 9.8f;
    Eigen::Matrix<float, 24, 12> A = Eigen::Matrix<float, 24, 12>::Zero();
    Eigen::Matrix<float, 24, 1> lb = Eigen::Matrix<float, 24, 1>::Zero();


    /* Compute the force limit part of the matrix. */
    for (int legId = 0; legId < 4; legId++) {
        A.block<1, 3>(legId * 2, legId * 3) = surfaceNormal;
        A.block<1, 3>(legId * 2 + 1, legId * 3) = -surfaceNormal;
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

    /* Compute the friction cone of the constraint matrix. */
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

    /* This step is essential to ensure the quadprogpp to get the solution. */
    R = R * regWeight;

    /* The quadratic problem consists quadratic term and linear term.
     * The constant term is ignored.
     */
    Eigen::Matrix<float, 12, 12> quadTerm;
    Eigen::Matrix<float, 12, 1> linearTerm;
    quadTerm = massMatrix.transpose() * Q * massMatrix + R;
    linearTerm = (g + desiredAcc).transpose() * Q * massMatrix;

    std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> quadLinear(quadTerm, linearTerm);
    return quadLinear;
}


Eigen::Matrix<float,12,12> ComputeWeightMatrix(qrRobot *robot, const Eigen::Matrix<bool, 4, 1>& contacts)
{
    Eigen::Matrix<float,12,12> W = 1e-4 * Eigen::Matrix<float,12,12>::Identity(); // 1e-4
    return W;
}


Eigen::Matrix<float, 3, 4> ComputeContactForce(
    qrRobot *robot,
    qrGroundSurfaceEstimator* groundEstimator,
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
    Eigen::Matrix<float, 6, 1> g = Eigen::Matrix<float, 6, 1>::Zero();
    g(2, 0) = 9.8;

    TerrainType& goundType = groundEstimator->terrain.terrainType;
    Mat3<float> Rcb;

    /* The Rcb matrix will be used to transform the inertia matrix using R * I * R^T.
     * This matrix is different in different terrains.
     */
    if (goundType == TerrainType::PLANE || goundType==TerrainType::PLUM_PILES) {
        Rcb = Mat3<float>::Identity(); // control in base frame
    } else {
        Rcb = rotMatControlFrame.transpose() * robotics::math::quaternionToRotationMatrix(quat).transpose();
        g.head(3) = rotMatControlFrame.transpose() * g.head(3);
        surfaceNormal << -std::sin(controlFrameRPY[1]), 0.f, std::cos(controlFrameRPY[1]);
    }

    /* Calculate the mass matrix. */
    Mat3<float> inertia = Rcb * robot->totalInertia * Rcb.transpose();
    Eigen::Matrix<float, 4, 3> footPosition = (Rcb * robot->GetFootPositionsInBaseFrame()).transpose();
    Eigen::Matrix<float, 6, 12> massMatrix = ComputeMassMatrix(robot->totalMass, inertia, footPosition);

    /* Calculate the objective matrix. */
    std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> Ga;
    Ga = ComputeObjectiveMatrix(massMatrix, desiredAcc, accWeight, regWeight, g);

    /* Calculate the regulation and add it to the objective matrix. */
    Eigen::Matrix<float, 12, 12> G = std::get<0>(Ga);
    Eigen::Matrix<float,12,12> W = ComputeWeightMatrix(robot, contacts);

    G = G + W;

    Eigen::Matrix<float, 12, 1> a = std::get<1>(Ga);

    /* Compute constraint of the quadratic problem. */
    std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> CI =
        ComputeConstraintMatrix(robot->totalMass, contacts, frictionCoef, fMinRatio, fMaxRatio, surfaceNormal);

    Eigen::Matrix<float, 12, 24> Ci = std::get<0>(CI);
    Eigen::Matrix<float, 24, 1> b = std::get<1>(CI);

    /* Convert Eigen matrix into quadprogpp form. */
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

    /* Reshape result x from (12, 1) to (4, 3). */
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

    return (X * Rcb).transpose(); // convert force to current base frame
}
    

Eigen::Matrix<float, 3, 4> ComputeContactForce(
    qrRobot *robot,
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
    /* rotMat transform base frame to world frame.
     * Then calculate the foot position in world frame.
     */
    Quat<float> quat = robot->GetBaseOrientation();
    Eigen::Matrix<float,3,4> footPositionsInBaseFrame = robot->GetFootPositionsInBaseFrame();
    Mat3<float> rotMat = robotics::math::quaternionToRotationMatrix(quat).transpose();

    Eigen::Matrix<float, 3, 4> footPositionsInCOMWorldFrame =
        robotics::math::invertRigidTransform<float,4>({0.f, 0.f, 0.f}, quat, footPositionsInBaseFrame);

    /* The following progress is same to another ComputeContactForce function.
     * The only difference is that all calculation is done in world framein this function.
     */
    Eigen::Matrix<float, 6, 12> massMatrix = ComputeMassMatrix(robot->totalMass,
                                                                robot->totalInertia,
                                                                footPositionsInCOMWorldFrame.transpose(),
                                                                rotMat);

    std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> Ga;
    Eigen::Matrix<float, 6, 1> g = Eigen::Matrix<float, 6, 1>::Zero();
    g(2, 0) = 9.8f;
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
    return robotics::math::RigidTransform<float,4>({0.f, 0.f, 0.f}, quat, X.transpose());
}
    

Eigen::Matrix<float, 6, 12> ComputeMassMatrix(
    float robotMass,
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

} // namespace Quadruped

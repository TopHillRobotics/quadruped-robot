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

#ifndef QR_QP_TORQUE_OPTIMIZER_H
#define QR_QP_TORQUE_OPTIMIZER_H

#include <tuple>
#include <Eigen/Dense>
#include "robots/qr_robot.h"
#include "estimators/qr_ground_surface_estimator.h"

namespace Quadruped {

/**
 * @brief Compute the inverse mass matrix of the force balance problem in control frame.
 * @attention Inputs used in this function should be expressed in control frame.
 * @param robotMass: the total mass of the quadruped.
 * @param robotInertia: the inertia matrix of the whole body in control frame.
 * This can be considered as a virtual inertia.
 * @param footPositions: the location of 4 footholds in control frame.
 * @return the inverse mass matrix.
 */
Eigen::Matrix<float, 6, 12> ComputeMassMatrix(
    float robotMass,
    Eigen::Matrix<float, 3, 3> robotInertia,
    Eigen::Matrix<float, 4, 3> footPositions);

/**
 * @brief Compute the inverse mass matrix of the force balance problem in world frame.
 * @param robotMass: the total mass of the quadruped.
 * @param robotInertia: the inertia matrix of the whole body in control frame.
 * This can be considered as a virtual inertia.
 * @param footPositions: the location of 4 footholds in control frame.
 * @param rotMat: rotation matrix that transfer from base frame to world frame.
 * @return the inverse mass matrix.
 */
Eigen::Matrix<float, 6, 12> ComputeMassMatrix(
    float robotMass,
    Eigen::Matrix<float, 3, 3> robotInertia,
    Eigen::Matrix<float, 4, 3> footPositions,
    Mat3<float> rotMat);

/**
 * @brief Compute the constrain matrix of the quadratic problem, including friction cone and force limit.
 * @param bodyMass: total mass of robot
 * @param contacts: 4-length array indicating whether feet is contact with ground.
 * @param frictionCoef: defines the interaction force effect between foot and env.
 * @param fMinRatio: min force that applys
 * @param fMaxRatio: max force that applys
 * @param surfaceNormal: the normal vector of terrain at the contacted foothold. In this function, it is fixed.
 * If user wanna adjust for debug, he can change this value.
 * @return the constraint matrix.
 */
std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> ComputeConstraintMatrix(
    float bodyMass,
    Eigen::Matrix<bool, 4, 1> contacts,
    float frictionCoef,
    float fMinRatio,
    float fMaxRatio,
    Vec3<float> surfaceNormal={0.f, 0.f, 1.f});

/**
 * @brief ComputeConstraintMatrix
 * @param bodyMass: total mass of robot
 * @param contacts: 4-length array indicating whether feet is contact with ground.
 * @param frictionCoef: defines the interaction force effect between foot and env.
 * @param fMinRatio: 4-element min force vector that applys
 * @param fMaxRatio: 4-element max force vector that applys
 * @param normal: the normal vector of the terrain at contacted footholds.
 * @param tangent1: the angle along X axis. Used for calculating the friction cone.
 * @param tangent2: the angle along Y axis.  Used for calculating the friction cone.
 * @return the constraint matrix.
 */
std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> ComputeConstraintMatrix(
  float bodyMass,
  Eigen::Matrix<bool, 4, 1> contacts,
  float frictionCoef,
  Vec4<float> fMinRatio,
  Vec4<float> fMaxRatio,
  Vec3<float> normal,
  Vec3<float> tangent1,
  Vec3<float> tangent2);

/**
 * @brief Compute the objective matrix of the quadratic problem.
 * @param massMatrix: the mass matrix computed before.
 * @param desiredAcc: desired acceleration vector computed by KP/KD control.
 * @param accWeight: the weight for the 6 acceleration components.
 * @param regWeight: a small value to keep the possitive definite.
 * @param g: the gravity acceleration vector.
 * @return desired objective matrix.
 */
std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> ComputeObjectiveMatrix(
    Eigen::Matrix<float, 6, 12> massMatrix,
    Eigen::Matrix<float, 6, 1> desiredAcc,
    Eigen::Matrix<float, 6, 1> accWeight,
    float regWeight,
    Eigen::Matrix<float, 6, 1> g);

/**
 * @brief Compute the regulation weight.
 * @param robot: pointer to Robot class.
 * This may be removed in the future.
 * @param contacts: the contact state of 4 legs.
 * @return a regulation weight matrix.
 */
Eigen::Matrix<float,12,12> ComputeWeightMatrix(qrRobot *robot, const Eigen::Matrix<bool, 4, 1>& contacts);

/**
 * @brief Compute the desired contact force by force balance method.
 * The force is expressed in base frame.
 * @param robot: pointer to robot.
 * @param groundEstimator: pointer to GroundEstimator.
 * @param desiredAcc: desired acceleration vector computed by KP/KD control.
 * @param contacts: 4-length array indicating whether feet is contact with ground.
 * @param accWeight: the weight for the 6 acceleration components.
 * @param regWeight: a small value to keep the possitive definite.
 * @param frictionCoef: defines the interaction force effect between foot and env.
 * @param fMinRatio: min force that applys
 * @param fMaxRatio: max force that applys
 * @return matrix of forces that should apply to each contacted foothold.
 */
Eigen::Matrix<float, 3, 4> ComputeContactForce(
    qrRobot *robot,
    qrGroundSurfaceEstimator* groundEstimator,
    Eigen::Matrix<float, 6, 1> desiredAcc,
    Eigen::Matrix<bool, 4, 1> contacts,
    Eigen::Matrix<float, 6, 1> accWeight,
    float regWeight = 1e-4,
    float frictionCoef = 0.5f,
    float fMinRatio = 0.01f,
    float fMaxRatio =10.f);
    
/**
 * @brief Compute the desired contact force by force balance method.
 * This function calculates the contact force in world frame.
 * @param robot: pointer to robot.
 * @param desiredAcc: desired acceleration vector computed by KP/KD control.
 * @param contacts: 4-length array indicating whether feet is contact with ground.
 * @param accWeight: the weight for the 6 acceleration components.
 * @param normal: the normal vector of the terrain in world frame.
 * @param tangent1: the angle along X axis. Used for calculating the friction cone.
 * @param tangent2: the angle along Y axis.  Used for calculating the friction cone.
 * @param fMinRatio: min force that applys. User can adjust parameter of each leg.
 * @param fMaxRatio: max force that applys. User can adjust parameter of each leg.
 * @param regWeight: a small value to keep the possitive definite.
 * @param frictionCoef: defines the interaction force effect between foot and env.
 * @return matrix of forces that should apply to each contacted foothold.
 */
Eigen::Matrix<float, 3, 4> ComputeContactForce(
    qrRobot *robot,
    Eigen::Matrix<float, 6, 1> desiredAcc,
    Eigen::Matrix<bool, 4, 1> contacts,
    Eigen::Matrix<float, 6, 1> accWeight,
    Vec3<float> normal,
    Vec3<float> tangent1,
    Vec3<float> tangent2,
    Vec4<float> fMinRatio = {0.01f, 0.01f, 0.01f, 0.01f},
    Vec4<float> fMaxRatio = {10.f, 10.f, 10.f, 10.f},
    float regWeight = 1e-4,
    float frictionCoef = 0.6f);

} // namespace Quadruped

#endif // QR_QP_TORQUE_OPTIMIZER_H

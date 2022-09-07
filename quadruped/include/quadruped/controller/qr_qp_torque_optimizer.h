// The MIT License

// Copyright (c) 2022 
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:  tophill.robotics@gmail.com

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
#include "state_estimator/qr_ground_estimator.h"

/** 
 * @brief Compute robot's mass matrix.
 * @param robotMass : float, ture mass of robot.
 * @param robotInertia : Mat3<float>, should expressed in control frame.
 * @param footPositions : Eigen::Matrix<float, 4, 3>, should expressed in control frame.
 * @return massMat : Eigen::Matrix<float, 6, 12>, in control frame.
 */
Eigen::Matrix<float, 6, 12> ComputeMassMatrix(float robotMass,
                                                Eigen::Matrix<float, 3, 3> robotInertia,
                                                Eigen::Matrix<float, 4, 3> footPositions);

/** 
 * @brief Compute constraint matrix.
 * @param mpcBodyMass total mass of robot for MPC computing
 * @param contacts 4-length array indicating whether feet is contact with ground.
 * @param frictionCoef frictionCoef defines the interaction force effect between foot and env.
 * @param fMinRatio min force that applys 
 * @param fMaxRatio max force that applys
 * @return Constraint matrix.
 */
std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> ComputeConstraintMatrix(
    float mpcBodyMass,
    Eigen::Matrix<bool, 4, 1> contacts,
    float frictionCoef,
    float fMinRatio,
    float fMaxRatio);

/** 
 * @brief Compute objective matrix.
 * @param massMatrix the mass matrix.
 * @param desiredAcc desired acceleration.
 * @param accWeight the weight of acceleration.
 * @param regWeight 
 * @param g acceleration of gravity.
 * @return Objective matrix.
 */
std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> ComputeObjectiveMatrix(
    Eigen::Matrix<float, 6, 12> massMatrix,
    Eigen::Matrix<float, 6, 1> desiredAcc,
    Eigen::Matrix<float, 6, 1> accWeight,
    float regWeight,
    Eigen::Matrix<float, 6, 1> g);

/** 
 * @brief Compute weight matrix.
 * @param robot the robot which we want to compute weight matrix.
 * @param contacts 4-length array indicating whether feet is contact with ground.
 * @return Weight matrix.
 */
Eigen::Matrix<float,12,12> ComputeWeightMatrix(qrRobot *robot, const Eigen::Matrix<bool, 4, 1>& contacts);

/** 
 * @brief Compute four legs' contact force matrix.
 * @param robot the robot which we want to compute weight matrix.
 * @param groundEstimator the ground estimator.
 * @param desiredAcc desired acceleration.
 * @param contacts 4-length array indicating whether feet is contact with ground.
 * @param accWeight the weight of acceleration.
 * @param regWeight 
 * @param frictionCoef frictionCoef defines the interaction force effect between foot and env.
 * @param fMinRatio min force that applys.
 * @param fMaxRatio max force that applys.
 * @return Four legs contact force matrix.
 */
Eigen::Matrix<float, 3, 4> ComputeContactForce(qrRobot *robot,
                                                qrGroundSurfaceEstimator* groundEstimator,
                                                Eigen::Matrix<float, 6, 1> desiredAcc,
                                                Eigen::Matrix<bool, 4, 1> contacts,
                                                Eigen::Matrix<float, 6, 1> accWeight,
                                                float regWeight=1e-4,
                                                float frictionCoef = 0.45,
                                                float fMinRatio = 0.1,
                                                float fMaxRatio = 10.);

/** 
 * @brief Compute four legs' contact force matrix. Writen by Zhu Yijie, in world frame. 
 * Used for climbing stairs or slopes.
 * @param robot the robot which we want to compute weight matrix.
 * @param groundEstimator the ground estimator.
 * @param desiredAcc desired acceleration.
 * @param contacts 4-length array indicating whether feet is contact with ground.
 * @param accWeight the weight of acceleration.
 * @param normal z axis vector.
 * @param tangent1 x axis vector.
 * @param tangent2 y axis vector.
 * @param fMinRatio min force that applys. Each leg might be different.
 * @param fMaxRatio max force that applys. Each leg might be different.
 * @param regWeight 
 * @param frictionCoef frictionCoef defines the interaction force effect between foot and env.
 * @return Four legs contact force matrix.
 */
Eigen::Matrix<float, 3, 4> ComputeContactForce(qrRobot *robot,
                                                Eigen::Matrix<float, 6, 1> desiredAcc,
                                                Eigen::Matrix<bool, 4, 1> contacts,
                                                Eigen::Matrix<float, 6, 1> accWeight,
                                                Vec3<float> normal,
                                                Vec3<float> tangent1,
                                                Vec3<float> tangent2,
                                                Vec4<float> fMinRatio={0.01f,0.01f,0.01f,0.01f},
                                                Vec4<float> fMaxRatio={10.f,10.f,10.f,10.f},
                                                float regWeight=1e-4,
                                                float frictionCoef=0.6f);

/** 
 * @brief Compute robot's mass matrix. Writen by Zhu Yijie, in world frame. 
 * Used for climbing stairs or slopes.
 * @param robotMass : float, ture mass of robot.
 * @param robotInertia : Mat3<float>, should expressed in control frame.
 * @param footPositions : Eigen::Matrix<float, 4, 3>, should expressed in control frame.
 * @param rotMat : 
 * @return massMat : Eigen::Matrix<float, 6, 12>, in control frame.
 */
Eigen::Matrix<float, 6, 12> ComputeMassMatrix(float robotMass,
                                                Eigen::Matrix<float, 3, 3> robotInertia,
                                                Eigen::Matrix<float, 4, 3> footPositions,
                                                Mat3<float> rotMat);

/** 
 * @brief Compute constraint matrix. Writen by Zhu Yijie, in world frame. 
 * Used for climbing stairs or slopes.
 * @param mpcBodyMass total mass of robot for MPC computing
 * @param contacts 4-length array indicating whether feet is contact with ground.
 * @param frictionCoef frictionCoef defines the interaction force effect between foot and env.
 * @param fMinRatio min force that applys 
 * @param fMaxRatio max force that applys
 * @param normal z axis vector.
 * @param tangent1 x axis vector.
 * @param tangent2 y axis vector.
 * @return Constraint matrix.
 */
std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> ComputeConstraintMatrix(
                                                                        float mpcBodyMass,
                                                                        Eigen::Matrix<bool, 4, 1> contacts,
                                                                        float frictionCoef,
                                                                        Vec4<float> fMinRatio,
                                                                        Vec4<float> fMaxRatio,
                                                                        Vec3<float> normal,
                                                                        Vec3<float> tangent1,
                                                                        Vec3<float> tangent2);

#endif //QR_QP_TORQUE_OPTIMIZER_H

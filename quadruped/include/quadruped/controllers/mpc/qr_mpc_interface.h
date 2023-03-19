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

#ifndef QR_MPC_INTERFACE
#define QR_MPC_INTERFACE

#include "utils/qr_se3.h"
#include "utils/qr_tools.h"
#include "robots/qr_timer.h"
#include <include/qpOASES.hpp>

#define K_MAX_GAIT_SEGMENTS 16

namespace Quadruped {

class MPCRobotState {

public:

    /**
     * @brief Print the robot state.
     */
    void print();

    /**
     * @brief Position in world frame.
     */
    Eigen::Matrix<float, 3, 1> p;

    /**
     * @brief Linear velocity in world frame.
     */
    Eigen::Matrix<float, 3, 1> v;

    /**
     * @brief Angular velocity in base frame.
     */
    Eigen::Matrix<float, 3, 1> w;

    /**
     * @brief Foothold positions in base frame.
     */
    Eigen::Matrix<float, 3, 4> footPosInBaseFrame;

    /**
     * @brief Rotation matrix from base frame to world frame.
     */
    Mat3<float> rotMat;

    /**
     * @brief Rotation matrix only considering yaw.
     */
    Mat3<float> yawRotMat;

    /**
     * @brief Inertia matrix in base frame.
     */
    Mat3<float> bodyInertia;

    /**
     * @brief Quaternion in world frame.
     */
    Eigen::Quaternionf quat;

    /**
     * @brief roll pitch yaw of the robot.
     */
    Eigen::Matrix<float, 3, 1> rpy;

    /**
     * @brief The mass of the robot.
     */
    float mass = 12;

    /**
     * @brief Predicted trajectory of the robot. Allow 16 future horizons at most.
     */
    float traj[12 * K_MAX_GAIT_SEGMENTS];

    /**
     * @brief Gait state for each legs. Each value is STANCE or SWING.
     */
    float gait[4 * K_MAX_GAIT_SEGMENTS];
};

struct ProblemConfig {

    /**
     * @brief Time for one step of MPC.
     */
    float dt;

    /**
     * @brief Defines the interaction force effect between foot and env.
     */
    float frictionCoeff;

    /**
     * @brief Max force of every leg that can exert.
     */
    float fMax;

    /**
     * @brief The length of the prediction horizon of the MPC.
     */
    int horizon;

    /**
     * @brief The total mass of the quadruped.
     */
    float totalMass;

    /**
     * @brief Weight for the 12 quadruped states, including pose and twist.
     */
    float weights[12];

    /**
     * @brief Weight for the force. This term is used in the quadratic problem to minimize the force.
     * Also used to trade off between state norm and force norm.
     */
    float alpha;
};

/**
 * @brief Setup the MPC parameters. This function will fill the static variable %problemConfig in cpp.
 * @param dt: time sonsidered by one MPC step.
 * @param horizon: future steps considered by MPC.
 * @param frictionCoeff: defines the interaction force effect between foot and env.
 * @param fMax: the max force acting on one leg.
 * @param totalMass: the total mass of the quadruped.
 * @param inertia: the inertia matrix of the quadruped in base frame.
 * @param weight: a 12-element weight vector for pose and twist.
 * @param alpha: a weight for forces in QP formulation.
 */
void SetupProblem(double dt, int horizon, double frictionCoeff, double fMax,
                  double totalMass, float *inertia, float *weight, float alpha);

/**
 * @brief Resize the static matrices before constructing MPC problem.
 * @param horizon: steps considered by MPC.
 */
void ResizeQPMats(s16 horizon);

/**
 * @brief Convert the problem to discrete time dynamics.
 * @param Ac: state matrix in continuous time.
 * @param Bc: transition matrix in continuous time.
 * @param dt: time for one MPC step.
 * @param horizon: steps considered by MPC.
 */
void ConvertToDiscreteQP(Eigen::Matrix<float, 13, 13> Ac, Eigen::Matrix<float, 13, 12> Bc, float dt, s16 horizon);

/**
 * @brief [in] Continuous time state space matrices, including A and B.
 * @param [in] I_world: inertia matrix in world frame.
 * @param [in] mass: the total mass of the quadruped.
 * @param [in] r_feet: matrice including foot positions to CoM.
 * @param [in] yawRotMat: the rotation matrix only containing yaw rotation.
 * @param [out] A: output, the matrice multiplying the state vector.
 * @param [out] B: output, the matrice multiplying the input vector.
 */
void ComputeContinuousTimeStateSpaceMatrices(
    Mat3<float> I_world, float mass, Eigen::Matrix<float, 3, 4> r_feet, Mat3<float> yawRotMat,
    Eigen::Matrix<float, 13, 13> &A, Eigen::Matrix<float, 13, 12> &B);

/**
 * @brief Solve the MPC problem.
 * This function actually construct the QP formulation and use qpOASES to solve it.
 * @param p: position of the quadruped in world frame.
 * @param v: velocity of the quadruped in world frame.
 * @param q: rotation expressed in quaternion in world frame.
 * @param w: angular velocity of the quadruped in world frame.
 * @param r: 4 vectors of footholds to CoM.
 * @param rpy: roll pitch and yaw of the quadruped.
 * @param state_trajectory: future state trajectory generated before.
 * @param gait: gait state in %horizon steps. Usually STANCE or SWING.
 */
void SolveMPCKernel(Vec3<float>& p, Vec3<float>& v, Quat<float>& q, Vec3<float>& w,
                            Eigen::Matrix<float,3,4> &r, Vec3<float>& rpy,
                            float *state_trajectory, float *gait);

/**
 * @brief Solve the MPC problem. Prepare essential data for MPC and call %SolveMPCKernel to solve it.
 * @param setup: some parameters for the MPC problem.
 */
void SolveMPC(ProblemConfig *setup);

/**
 * @brief Get value in MPC solution which is in form of qpOASES float vector.
 * @param index: the index of the result.
 * @return the result in MPC solution.
 */
double GetMPCSolution(int index);

} // Namespace Quadruped

#endif // QR_MPC_INTERFACE

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

#include "controllers/mpc/qr_mpc_interface.h"
#include <unsupported/Eigen/MatrixFunctions>

#define BIG_NUMBER 5e10

using Eigen::Dynamic;
using Eigen::Matrix;
using robotics::math::crossMatrix;

/// The problem configuration, for example, horizon length, weight and other robot infomation.
static Quadruped::ProblemConfig problemConfig;

/// Basic robot states, for example, pose, twist, rotation matrix and so on.
static Quadruped::MPCRobotState robotState;

/// Whether the MPC problem has been solved by qpOASES.
static int has_solved = 0;

/// State matrix consists of %horizon states.
static Matrix<float, Eigen::Dynamic, 13> Aqp;

/// Transform matrix consists of 4 * %horizon forces
static Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Bqp;

/// Transform matrix in discrete form.
static Matrix<float, 13, 12> Bdt;

/// State matrix in discrete form.
static Matrix<float, 13, 13> Adt;

/// Auxilliary matrix while calculating discrete dynamics.
static Matrix<float, 25, 25> ABc, expmm;

/// The predictive state trajectory.
static Matrix<float, Eigen::Dynamic, 1> X_d;

/// Upper bound for the constraint of forces.
static Matrix<float, Eigen::Dynamic, 1> U_b;

/// Friction cone and force limit matrix for constraint of forces.
static Matrix<float, Eigen::Dynamic, Eigen::Dynamic> fmat;

/// Hessian matrix used in QP form of MPC problem.
static Matrix<float, Eigen::Dynamic, Eigen::Dynamic> qH;

/// Linear term used in QP form of MPC problem.
static Matrix<float, Eigen::Dynamic, 1> qg;

/// A matrix with small value for constructing Hessian matrix.
static Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Idendity12Horizon;

/// Hessian matrix in qpOASES form.
static qpOASES::real_t *H_qpoases = nullptr;

/// g vector in qpOASES form.
static qpOASES::real_t *g_qpoases = nullptr;

/// Constraint matrix in qpOASES form.
static qpOASES::real_t *A_qpoases = nullptr;

/// Lower bound vector in qpOASES form.
static qpOASES::real_t *lb_qpoases = nullptr;

/// Upper bound vector in qpOASES form
static qpOASES::real_t *ub_qpoases = nullptr;

/// Solution of MPC in qpOASES form.
static qpOASES::real_t *q_soln = nullptr;

/// Current robot state, including pose and twist, along with a gravity component.
static Eigen::Matrix<float, 13, 1> x0;

/// State Matrix of simplified dynamics in continuous time.
static Eigen::Matrix<float, 13, 13> A_ct;

/// Transform Matrix of simplified dynamics in continuous time.
static Eigen::Matrix<float, 13, 12> B_ct_r;

/// Temporary matrix to construct the Hessian matrix.
static Eigen::MatrixXf temp;

namespace  {

/**
 * @brief Convert Eigen Matrix to c++ float array.
 * @param dst: destination float array.
 * @param src: source Eigen matrix.
 * @param n_items: number of elements in the matrix.
 */
inline void EigenToFloatArray(float *dst, float *src, s32 n_items)
{
    for (s32 i = 0; i < n_items; i++)
        *dst++ = *src++;
}

/**
 * @brief Convert Eigen Matrix to qpOASES real_t type.
 * @param dst: destination qpOASES real_t type.
 * @param src: source Eigen matrix.
 * @param rows: number of rows in Eigen matrix.
 * @param cols: number of columns in Eigen matrix.
 */
void EigenToOASES(qpOASES::real_t *dst, Matrix<float, Dynamic, Dynamic> src, s16 rows, s16 cols)
{
    s32 a = 0;
    for (s16 r = 0; r < rows; ++r) {
        for (s16 c = 0; c < cols; ++c) {
            dst[a] = src(r, c);
            ++a;
        }
    }
}

} // Anonymous namespace


namespace Quadruped {

void MPCRobotState::print()
{
    using std::cout;
    using std::endl;
    cout << "Robot State:" << endl
         << "Position\n"
         << p.transpose()
         << "\nVelocity\n"
         << v.transpose() << "\nAngular Veloctiy\n"
         << w.transpose() << "\nRotation\n"
         << rotMat << "\nYaw Rotation\n"
         << yawRotMat << "\nFoot Locations\n"
         << footPosInBaseFrame << "\nInertia\n"
         << bodyInertia << endl;
}


void SetupProblem(double dt, int horizon, double frictionCoeff, double fMax, double totalMass, float *inertia, float *weights, float alpha)
{
    printf("SetupProblem: f_max = %f, mass = %f, horizon = %d\n", fMax, totalMass, horizon);
    problemConfig.totalMass = totalMass;
    problemConfig.horizon    = horizon;
    problemConfig.fMax      = fMax;
    problemConfig.frictionCoeff         = frictionCoeff;
    problemConfig.dt         = dt;
    problemConfig.alpha      = alpha;

    memcpy((void *)problemConfig.weights, (void *)weights, sizeof(float) * 12);

    robotState.bodyInertia.diagonal() << inertia[0], inertia[1], inertia[2];
    robotState.mass = totalMass;
    ResizeQPMats(horizon);
}


void ResizeQPMats(s16 horizon)
{
    int mcount = 0;
    int h2 = horizon * horizon;

    Aqp.resize(13 * horizon, Eigen::NoChange);
    mcount += 13 * horizon * 1;

    Bqp.resize(13 * horizon, 12 * horizon);
    mcount += 13 * h2 * 12;

    X_d.resize(13 * horizon, Eigen::NoChange);
    mcount += 13 * horizon;

    U_b.resize(20 * horizon, Eigen::NoChange);
    mcount += 20 * horizon;

    fmat.resize(20 * horizon, 12 * horizon);
    mcount += 20 * 12 * h2;

    qH.resize(12 * horizon, 12 * horizon);
    mcount += 12 * 12 * h2;

    qg.resize(12 * horizon, Eigen::NoChange);
    mcount += 12 * horizon;

    Idendity12Horizon.resize(12 * horizon, 12 * horizon);
    mcount += 12 * 12 * horizon;

    mcount = 0;
    
    temp.resize(12 * horizon, 13 * horizon);

    Aqp.setZero();
    Bqp.setZero();
    X_d.setZero();
    U_b.setZero();
    fmat.setZero();
    qH.setZero();
    Idendity12Horizon.setIdentity();

    s16 k = 0;
    for (s16 i = 0; i < problemConfig.horizon; ++i) {
        for (s16 j = 0; j < 4; ++j) {
            U_b(5 * k + 0) = BIG_NUMBER;
            U_b(5 * k + 1) = BIG_NUMBER;
            U_b(5 * k + 2) = BIG_NUMBER;
            U_b(5 * k + 3) = BIG_NUMBER;
            U_b(5 * k + 4) = BIG_NUMBER;
            ++k;
        }
    }
    float mu_ = 1.f / problemConfig.frictionCoeff;
    Matrix<float, 5, 3> f_block;
    f_block << mu_, 0,   1.f,
              -mu_, 0,   1.f,
               0,   mu_, 1.f,
               0,  -mu_, 1.f,
               0,   0,   1.f;

    for (s16 i = 0; i < problemConfig.horizon * 4; ++i) {
        fmat.block(i * 5, i * 3, 5, 3) = f_block;
    }

    H_qpoases = (qpOASES::real_t *)realloc(H_qpoases, 12 * 12 * horizon * horizon * sizeof(qpOASES::real_t));
    mcount += 12 * 12 * h2;
    g_qpoases = (qpOASES::real_t *)realloc(g_qpoases, 12 * 1 * horizon * sizeof(qpOASES::real_t));
    mcount += 12 * horizon;
    A_qpoases = (qpOASES::real_t *)realloc(A_qpoases, 12 * 20 * horizon * horizon * sizeof(qpOASES::real_t));
    mcount += 12 * 20 * h2;
    lb_qpoases = (qpOASES::real_t *)realloc(lb_qpoases, 20 * 1 * horizon * sizeof(qpOASES::real_t));
    mcount += 20 * horizon;
    ub_qpoases = (qpOASES::real_t *)realloc(ub_qpoases, 20 * 1 * horizon * sizeof(qpOASES::real_t));
    mcount += 20 * horizon;
    q_soln = (qpOASES::real_t *)realloc(q_soln, 12 * horizon * sizeof(qpOASES::real_t));
    mcount += 12 * horizon;
}


void ConvertToDiscreteQP(Matrix<float, 13, 13> Ac, Matrix<float, 13, 12> Bc, float dt, s16 horizon)
{
    /* The equation of the dynamics is:
     * d[x, u]^T/dt = [ A B | 0 0 ] [x u]^T.
     * Let [ A B | 0 0 ] = ABc, then the solution is (x, u)^T = e^{[A b | 0 0] * t} * (x0, u0).
     * If considering this problem as discrete, then x[n + 1] = Adt * x[n] + Bdt * u[n].
     */
    ABc.setZero();
    ABc.block(0, 0, 13, 13) = Ac;
    ABc.block(0, 13, 13, 12) = Bc;
    ABc = dt * ABc;
    expmm = ABc.exp();
    Adt = expmm.block(0, 0, 13, 13);
    Bdt = expmm.block(0, 13, 13, 12);

    Matrix<float, 13, 13> powerMats[20];
    powerMats[0].setIdentity();
    for (int i = 1; i < horizon + 1; ++i) {
        powerMats[i] = Adt * powerMats[i - 1];
    }

    /* A_qp seems like:
     * [Adt  Adt^2  Adt^3 ...]
     *
     * B_qp seems like:
     * [Bdt  Adt * Bdt  Adt^2 * Bdt  ...]
     */
    for (s16 r = 0; r < horizon; ++r) {
        Aqp.block(13 * r, 0, 13, 13) = powerMats[r + 1];
        for (s16 c = 0; c < horizon; ++c) {
            if (r >= c) {
                s16 a_num = r - c;
                Bqp.block(13 * r, 12 * c, 13, 12) = powerMats[a_num] * Bdt;
            }
        }
    }
}


void ComputeContinuousTimeStateSpaceMatrices(
    Mat3<float> I_world, float mass, Matrix<float, 3, 4> r_feet, Mat3<float> yawRotMat,
    Matrix<float, 13, 13> &A, Matrix<float, 13, 12> &B)
{
    /* A matrix seems like:
     * | 0_33  0_33 Ry_33  0_33 0   |
     * | 0_33  0_33  0_33  E_33 0   |
     * | 0_33  0_33  0_33  0_33 0   |
     * | 0_33  0_33  0_33  0_33 N^3 |
     * N^3 is a vector, [0 0 1]^T.
     */
    A.setZero();
    A(3, 9) = 1.f;
    A(4, 10) = 1.f;
    A(5, 11) = 1.f;

    A(11, 12) = 1.f;
    A.block(0, 6, 3, 3) = yawRotMat.transpose();

    /* B matrix seems like:
     * |              0_33               0_33              0_33              0_33 |
     * |              0_33               0_33              0_33              0_33 |
     * | I_{-1}.cross(r_0)  I_{-1}.cross(r_1) I_{-1}.cross(r_2) I_{-1}.cross(r_3) |
     * |         I_33/mass          I_33/mass         I_33/mass         I_33/mass |
     * I_{-1} is the inverse inertia matrix, usually a diagnose matrix.
     * r_i is the vector from ith foothold to CoM.
     */
    B.setZero();
    Mat3<float> I_world_inv = I_world.inverse();
    Mat3<float> m_inv = Mat3<float>::Identity() / mass;

    for (u8 b = 0; b < 4; ++b) {
        B.block(6, b * 3, 3, 3) = I_world_inv * crossMatrix(r_feet.col(b));
        B.block(9, b * 3, 3, 3) = m_inv;
    }
}


void SolveMPCKernel(Vec3<float>& p, Vec3<float>& v, Quat<float>& q, Vec3<float>& w,
                            Eigen::Matrix<float,3,4> &r, Vec3<float>& rpy,
                            float *state_trajectory, float *gait)
{
    /* Setup robot state for MPC. */
    ::EigenToFloatArray(robotState.gait, gait, 4 * problemConfig.horizon);
    memcpy((void *)robotState.traj, (void *)state_trajectory, sizeof(float) * 12 * problemConfig.horizon);
    robotState.rpy = rpy;
    robotState.p = p;
    robotState.v = v;
    robotState.quat.w() = q[0];
    robotState.quat.x() = q[1];
    robotState.quat.y() = q[2];
    robotState.quat.z() = q[3];
    robotState.w = w;
    robotState.footPosInBaseFrame = r;
    robotState.rotMat = robotState.quat.toRotationMatrix();
    robotState.yawRotMat = robotState.rotMat;

    SolveMPC(&problemConfig);

    has_solved = 1;
}


void SolveMPC(ProblemConfig *setup)
{
    /* Initial state with gravity. */
    x0 << robotState.rpy, robotState.p, robotState.w, robotState.v, -9.8f;

    /* Get continuous time state space matrices. */
    Mat3<float> I_world = robotState.yawRotMat * robotState.bodyInertia * robotState.yawRotMat.transpose();//original
    ComputeContinuousTimeStateSpaceMatrices(I_world, robotState.mass, robotState.footPosInBaseFrame, robotState.yawRotMat, A_ct, B_ct_r);

    /* Transform to discrete dynamics and get the QP formulation:
     * X = Aqp x0 + Bqp U.
     */
    ConvertToDiscreteQP(A_ct, B_ct_r, setup->dt, setup->horizon);// 0.03ms

    /* Be careful that the 13th element is gravity component.
     * The weight should repeat for %horizon times in QP formulation.
     */
    Matrix<float, 13, 1> full_weight;
    for (u8 i = 0; i < 12; ++i)
        full_weight(i) = setup->weights[i];
    full_weight(12) = 0.f;
    
    /* Copy the predictive trajectory to X_d. */
    for (s16 i = 0, k = 0; i < setup->horizon; ++i) {
        for (s16 j = 0; j < 12; ++j) {
            X_d(13 * i + j, 0) = robotState.traj[12 * i + j];
        }
        for (s16 j = 0; j < 4; ++j) {
            U_b(5 * k + 4) = robotState.gait[i * 4 + j] * setup->fMax;
            ++k;
        }
    }
    
    /* %temp is used to store 2Bqp^T * S.
     * %Bij stores the transpose of the block,
     * while %temp stores the block of 2Bqp^T * S.
     */
    temp.setZero();
    Eigen::Matrix<float, 12, 13> subTemp, Bij;
    for (u8 i = 0; i < setup->horizon; ++i) {
        for (u8 j = i; j < setup->horizon; ++j) {
            Bij = Bqp.block(j * 13, i * 12, 13, 12).transpose();
            for (u8 n(0); n < 13; ++n) {
                subTemp.col(n) = Bij.col(n) * (2 * full_weight(n));
            }
            temp.block(i * 12, j * 13, 12, 13) = subTemp;
        }
    }

    /* H = 2(Bqp^T * L * Bqp + K).
     * g = 2 * Bqp^T * L * ( Aqp *x0 - xd).
     */
    qH = temp * Bqp + (2 * setup->alpha) * Idendity12Horizon;
    qg = temp * (Aqp * x0 - X_d);

    int num_constraints = 20 * setup->horizon; /* Every leg has 5 constraints. */
    int num_variables = 12 * setup->horizon; /* 4 force vectors. */

    /* Convert Eigen matrix to qpOASES matrix. */
    ::EigenToOASES(H_qpoases, qH, num_variables, num_variables);
    ::EigenToOASES(g_qpoases, qg, num_variables, 1);
    ::EigenToOASES(A_qpoases, fmat, num_constraints, num_variables);
    ::EigenToOASES(ub_qpoases, U_b, num_constraints, 1);

    for (int i = 0; i < num_constraints; ++i) {
        lb_qpoases[i] = 0.0f;
    }

    /* Setup qpOASES to solve the problem. */
    qpOASES::int_t nWSR = 100;
    qpOASES::QProblem problem(num_variables, num_constraints);
    qpOASES::Options option;

    option.setToMPC();
    option.printLevel = qpOASES::PL_NONE;
    problem.setOptions(option);

    int rval = problem.init(H_qpoases, g_qpoases, A_qpoases, NULL, NULL, lb_qpoases, ub_qpoases, nWSR);

    int rval2 = problem.getPrimalSolution(q_soln);

    if (rval2 != qpOASES::SUCCESSFUL_RETURN) {
        printf("failed to solve!\n");
    }
}


double GetMPCSolution(int index)
{
    if (!has_solved) return 0.f;
    double *qs = q_soln;
    return qs[index];
}

} // namespace Quadruped

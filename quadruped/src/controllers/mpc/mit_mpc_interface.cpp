#include "controllers/mpc/mit_mpc_interface.h"
#include "robots/timer.h"
#include <unsupported/Eigen/MatrixFunctions>
#define BIG_NUMBER 5e10
using Eigen::Dynamic;
using Eigen::Matrix;
using robotics::math::crossMatrix;
using std::cout;
using std::endl;

void RobotState::set(float *p_, float *v_, float *q_, float *w_, float *r_, float *rpy_, float total_mass)
{
    for (u8 i = 0; i < 3; ++i) {
        this->p(i) = p_[i];
        this->v(i) = v_[i];
        this->w(i) = w_[i];
        this->rpy(i) = rpy_[i];
    }
    this->q.w() = q_[0];
    this->q.x() = q_[1];
    this->q.y() = q_[2];
    this->q.z() = q_[3];
    // this->q << q_[0], q_[1], q_[2],q_[3];

    for (u8 leg(0); leg < 4; ++leg) {
        for (u8 axis(0); axis < 3; ++axis) {
            this->r_feet(axis, leg) = r_[leg * 3 + axis];
        }
    }

    R = this->q.toRotationMatrix();
    R_yaw = R;

    // Matrix<float, 3, 1> Id;
    // // Id << .07f, 0.26f, 0.242f; // mini cheetah
    // Id << 0.24f, 0.8f, 1.0f;// a1
    // // Id << 0.4f, 1.15f, 1.0f; // aliengo + z1 arm
    // I_body.diagonal() = Id;
    // m = total_mass;
    //TODO: Consider normalizing quaternion??
}

void RobotState::print()
{
    cout << "Robot State:" << endl
         << "Position\n"
         << p.transpose()
         << "\nVelocity\n"
         << v.transpose() << "\nAngular Veloctiy\n"
         << w.transpose() << "\nRotation\n"
         << R << "\nYaw Rotation\n"
         << R_yaw << "\nFoot Locations\n"
         << r_feet << "\nInertia\n"
         << I_body << endl;
}

ProblemConfig problemConfig;
UpdateDataT update;
RobotState rs;
int has_solved = 0;

Matrix<float, Dynamic, 13> A_qp;
Matrix<float, Dynamic, Dynamic> B_qp;
Matrix<float, 13, 12> Bdt;
Matrix<float, 13, 13> Adt;
Matrix<float, 25, 25> ABc, expmm;
Matrix<float, Dynamic, Dynamic> S;
Matrix<float, Dynamic, 1> X_d;
Matrix<float, Dynamic, 1> U_b;
Matrix<float, Dynamic, Dynamic> fmat;
Matrix<float, Dynamic, Dynamic> qH;
Matrix<float, Dynamic, 1> qg;
Matrix<float, Dynamic, Dynamic> eye_12h;

qpOASES::real_t *H_qpoases = nullptr;
qpOASES::real_t *g_qpoases = nullptr;
qpOASES::real_t *A_qpoases = nullptr;
qpOASES::real_t *lb_qpoases = nullptr;
qpOASES::real_t *ub_qpoases = nullptr;
qpOASES::real_t *q_soln = nullptr;
qpOASES::real_t *H_red = nullptr;
qpOASES::real_t *g_red = nullptr;
qpOASES::real_t *A_red = nullptr;
qpOASES::real_t *lb_red = nullptr;
qpOASES::real_t *ub_red = nullptr;
qpOASES::real_t *q_red = nullptr;
char var_elim[2000];
char con_elim[2000];

Eigen::Matrix<float, 13, 1> x_0;
Mat3<float> I_world;
Eigen::Matrix<float, 13, 13> A_ct;
Eigen::Matrix<float, 13, 12> B_ct_r;

void SetupProblem(double dt, int horizon, double mu, double f_max, double total_mass, float *inertia, float *weights, float alpha)
{
    problemConfig.total_mass = total_mass;
    problemConfig.horizon = horizon;
    problemConfig.f_max = f_max;
    problemConfig.mu = mu;
    problemConfig.dt = dt;
    problemConfig.alpha = alpha;
    memcpy((void *)problemConfig.weights, (void *)weights, sizeof(float) * 12);

    rs.I_body.diagonal() << inertia[0], inertia[1], inertia[2];
    rs.m = total_mass;
    resize_qp_mats(horizon);
}

void resize_qp_mats(s16 horizon)
{
    int mcount = 0;
    int h2 = horizon * horizon;

    A_qp.resize(13 * horizon, Eigen::NoChange);
    mcount += 13 * horizon * 1;

    B_qp.resize(13 * horizon, 12 * horizon);
    mcount += 13 * h2 * 12;

    S.resize(13 * horizon, 13 * horizon);
    mcount += 13 * 13 * h2;

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

    eye_12h.resize(12 * horizon, 12 * horizon);
    mcount += 12 * 12 * horizon;

    //printf("realloc'd %d floating point numbers.\n",mcount);
    mcount = 0;

    A_qp.setZero();
    B_qp.setZero();
    S.setZero();
    X_d.setZero();
    U_b.setZero();
    fmat.setZero();
    qH.setZero();
    eye_12h.setIdentity();

    s16 k = 0;
    for (s16 i = 0; i < problemConfig.horizon; ++i) {
        for (s16 j = 0; j < 4; ++j) {
            U_b(5 * k + 0) = BIG_NUMBER;
            U_b(5 * k + 1) = BIG_NUMBER;
            U_b(5 * k + 2) = BIG_NUMBER;
            U_b(5 * k + 3) = BIG_NUMBER;
            U_b(5 * k + 4) = BIG_NUMBER;//update->gait[i * 4 + j] * setup->f_max;
            ++k;
        }
    }
    float mu_ = 1.f / problemConfig.mu;
    Matrix<float, 5, 3> f_block;
    f_block << mu_, 0, 1.f,
        -mu_, 0, 1.f,
        0, mu_, 1.f,
        0, -mu_, 1.f,
        0, 0, 1.f;

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

    H_red = (qpOASES::real_t *)realloc(H_red, 12 * 12 * horizon * horizon * sizeof(qpOASES::real_t));
    mcount += 12 * 12 * h2;
    g_red = (qpOASES::real_t *)realloc(g_red, 12 * 1 * horizon * sizeof(qpOASES::real_t));
    mcount += 12 * horizon;
    A_red = (qpOASES::real_t *)realloc(A_red, 12 * 20 * horizon * horizon * sizeof(qpOASES::real_t));
    mcount += 12 * 20 * h2;
    lb_red = (qpOASES::real_t *)realloc(lb_red, 20 * 1 * horizon * sizeof(qpOASES::real_t));
    mcount += 20 * horizon;
    ub_red = (qpOASES::real_t *)realloc(ub_red, 20 * 1 * horizon * sizeof(qpOASES::real_t));
    mcount += 20 * horizon;
    q_red = (qpOASES::real_t *)realloc(q_red, 12 * horizon * sizeof(qpOASES::real_t));
    mcount += 12 * horizon;
}

inline void mflt_to_flt(float *dst, float *src, s32 n_items)
{
    for (s32 i = 0; i < n_items; i++)
        *dst++ = *src++;
}

void SolveMPCKernel(float *p, float *v, float *q, float *w,
                    float *r, float *rpy,
                    float *state_trajectory, float *gait)
{
    mflt_to_flt(update.gait, gait, 4 * problemConfig.horizon);
    // memcpy((void *)update.gait, (void *)gait, sizeof(float) * 4 * problemConfig.horizon);
    memcpy((void *)update.rpy, (void *)rpy, sizeof(float) * 3);
    memcpy((void *)update.p, (void *)p, sizeof(float) * 3);
    memcpy((void *)update.v, (void *)v, sizeof(float) * 3);
    memcpy((void *)update.q, (void *)q, sizeof(float) * 4);
    memcpy((void *)update.w, (void *)w, sizeof(float) * 3);
    memcpy((void *)update.r, (void *)r, sizeof(float) * 12);
    memcpy((void *)update.traj, (void *)state_trajectory, sizeof(float) * 12 * problemConfig.horizon);

    // MITTimer t2;
    solve_mpc(&update, &problemConfig);
    // printf("solve_mpc SOLVE TIME: %.3f  ms\n", t2.getMs());
    has_solved = 1;
    // printf("mpc problem has solved = %d\n", has_solved);
}

double GetMPCSolution(int index)
{
    if (!has_solved) return 0.f;
    double *qs = q_soln;
    return qs[index];
}

inline bool near_zero(float a)
{
    return (a < 0.01 && a > -.01);
}

inline bool near_one(float a)
{
    return near_zero(a - 1);
}

void matrix_to_real(qpOASES::real_t *dst, Matrix<float, Dynamic, Dynamic> src, s16 rows, s16 cols)
{
    s32 a = 0;
    for (s16 r = 0; r < rows; ++r) {
        for (s16 c = 0; c < cols; ++c) {
            dst[a] = src(r, c);
            ++a;
        }
    }
}

void c2qp(Matrix<float, 13, 13> Ac, Matrix<float, 13, 12> Bc, float dt, s16 horizon)
{
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

    for (s16 r = 0; r < horizon; ++r) {
        A_qp.block(13 * r, 0, 13, 13) = powerMats[r + 1];//Adt.pow(r+1);
        for (s16 c = 0; c < horizon; ++c) {
            if (r >= c) {
                s16 a_num = r - c;
                B_qp.block(13 * r, 12 * c, 13, 12) = powerMats[a_num] /*Adt.pow(a_num)*/ * Bdt;
            }
        }
    }
}

//continuous time state space matrices.
void ct_ss_mats(Mat3<float> I_world, float m, Matrix<float, 3, 4> r_feet, Mat3<float> R_yaw, Matrix<float, 13, 13> &A, Matrix<float, 13, 12> &B)
{
    A.setZero();
    A(3, 9) = 1.f;
    A(4, 10) = 1.f;
    A(5, 11) = 1.f;

    A(11, 12) = 1.f;
    A.block(0, 6, 3, 3) = R_yaw.transpose();

    B.setZero();
    Mat3<float> I_world_inv = I_world.inverse();
    Mat3<float> m_inv = Mat3<float>::Identity() / m;

    for (u8 b = 0; b < 4; ++b) {
        B.block(6, b * 3, 3, 3) = I_world_inv * crossMatrix(r_feet.col(b));
        B.block(9, b * 3, 3, 3) = m_inv;
    }
}

void print_problem_setup(ProblemConfig *setup)
{
    printf("MASS: %.3f\n", setup->total_mass);
    printf("DT: %.3f\n", setup->dt);
    printf("Mu: %.3f\n", setup->mu);
    printf("F_Max: %.3f\n", setup->f_max);
    printf("Horizon: %d\n", setup->horizon);
    pnv("Alpha", setup->alpha);
    print_named_array("weights", setup->weights, 1, 12);
}

void print_update_data(UpdateDataT *update, s16 horizon)
{
    print_named_array("p", update->p, 1, 3);
    print_named_array("v", update->v, 1, 3);
    print_named_array("q", update->q, 1, 4);
    print_named_array("w", update->r, 3, 4);
    print_named_array("trajectory", update->traj, horizon, 12);
    print_named_array("gait", update->gait, horizon, 4);
}

void solve_mpc(UpdateDataT *update, ProblemConfig *setup)
{
    // MITTimer t1;
    rs.set(update->p, update->v, update->q, update->w, update->r, update->rpy, setup->total_mass);

    //roll pitch yaw
    //initial state (13 state representation)
    x_0 << rs.rpy, rs.p, rs.w, rs.v, -9.8f;

    I_world = rs.R_yaw * rs.I_body * rs.R_yaw.transpose();//original
    ct_ss_mats(I_world, rs.m, rs.r_feet, rs.R_yaw, A_ct, B_ct_r);
    //QP matrices
    c2qp(A_ct, B_ct_r, setup->dt, setup->horizon);// 0.03ms

    //weights
    Matrix<float, 13, 1> full_weight;
    for (u8 i = 0; i < 12; i++)
        full_weight(i) = setup->weights[i];
    full_weight(12) = 0.f;
    S.diagonal() = full_weight.replicate(setup->horizon, 1);

    //trajectory
    for (s16 i = 0; i < setup->horizon; i++) {
        for (s16 j = 0; j < 12; j++)
            X_d(13 * i + j, 0) = update->traj[12 * i + j];
    }

    s16 k = 0;
    for (s16 i = 0; i < setup->horizon; ++i) {
        for (s16 j = 0; j < 4; ++j) {
            U_b(5 * k + 4) = update->gait[i * 4 + j] * setup->f_max;
            k++;
        }
    }
    // printf("qp1 solve time: %.3f ms\n", t1.getMs());

    Eigen::MatrixXf temp;
    temp.resize(12 * setup->horizon, 13 * setup->horizon);
    temp.setZero();
    Eigen::Matrix<float, 12, 13> subTemp, Bij;
    for (u8 i(0); i < setup->horizon; ++i) {
        for (u8 j(i); j < setup->horizon; ++j) {
            // B^(ij) = Bqp_{j,i}.T
            Bij = B_qp.block(j * 13, i * 12, 13, 12).transpose();
            // for (u8 m(0); m<12; ++m) {
            for (u8 n(0); n < 13; ++n) {
                // B^(ij) * S^{j} = B^{ij}_{m}*S_{j}^{m}
                subTemp.col(n) = Bij.col(n) * (2 * full_weight(n));// S(n,n);
            }
            // }
            temp.block(i * 12, j * 13, 12, 13) = subTemp;
        }
    }
    qH = temp * B_qp + (2 * setup->alpha) * eye_12h;// 0.07ms
    qg = temp * (A_qp * x_0 - X_d);                 // 0.03ms   - (13*5, 1)

    s16 num_constraints = 20 * setup->horizon;
    s16 num_variables = 12 * setup->horizon;

    // MITTimer solve_timer1;
    matrix_to_real(H_qpoases, qH, num_variables, num_variables);
    matrix_to_real(g_qpoases, qg, num_variables, 1);
    matrix_to_real(A_qpoases, fmat, num_constraints, num_variables);
    matrix_to_real(ub_qpoases, U_b, num_constraints, 1);

    for (s16 i = 0; i < num_constraints; ++i) {
        lb_qpoases[i] = 0.0f;
    }

    qpOASES::int_t nWSR = 100;

    int new_vars = num_variables;
    int new_cons = num_constraints;
    // std::cout << "num_variables = "<< num_variables << ", num_constraints = " << num_constraints <<std::endl;

    for (int i = 0; i < num_constraints; ++i) {
        con_elim[i] = 0;
    }
    for (int i = 0; i < num_variables; ++i) {
        var_elim[i] = 0;
    }

    for (int i = 0; i < num_constraints; ++i) {
        if (!(near_zero(lb_qpoases[i]) && near_zero(ub_qpoases[i]))) continue;
        double *c_row = &A_qpoases[i * num_variables];
        for (int j = 0; j < num_variables; ++j) {
            if (near_one(c_row[j])) {
                new_vars -= 3;
                new_cons -= 5;
                int cs = (j * 5) / 3 - 3;
                var_elim[j - 2] = 1;
                var_elim[j - 1] = 1;
                var_elim[j] = 1;
                con_elim[cs] = 1;
                con_elim[cs + 1] = 1;
                con_elim[cs + 2] = 1;
                con_elim[cs + 3] = 1;
                con_elim[cs + 4] = 1;
            }
        }
    }
    // printf("qp1 solve time: %.3f ms\n", solve_timer1.getMs());

    // MITTimer solve_timer1;
    int var_ind[new_vars];
    int con_ind[new_cons];
    int vc = 0;
    for (int i = 0; i < num_variables; ++i) {
        if (!var_elim[i]) {
            if (!(vc < new_vars)) {
                printf("BAD ERROR 1\n");
            }
            var_ind[vc] = i;
            ++vc;
        }
    }
    vc = 0;
    for (int i = 0; i < num_constraints; ++i) {
        if (!con_elim[i]) {
            if (!(vc < new_cons)) {
                printf("BAD ERROR 1\n");
            }
            con_ind[vc] = i;
            ++vc;
        }
    }

    for (int i = 0; i < new_vars; ++i) {
        int olda = var_ind[i];
        g_red[i] = g_qpoases[olda];
        for (int j = 0; j < new_vars; ++j) {
            int oldb = var_ind[j];
            H_red[i * new_vars + j] = H_qpoases[olda * num_variables + oldb];
        }
    }

    for (int con = 0; con < new_cons; ++con) {
        for (int st = 0; st < new_vars; ++st) {
            float cval = A_qpoases[(num_variables * con_ind[con]) + var_ind[st]];
            A_red[con * new_vars + st] = cval;
        }
    }
    for (int i = 0; i < new_cons; ++i) {
        int old = con_ind[i];
        ub_red[i] = ub_qpoases[old];
        lb_red[i] = lb_qpoases[old];
    }
    // printf("qp1 solve time: %.3f ms, size %d, %d\n", solve_timer1.getMs(), new_vars, new_cons);

    // MITTimer solve_timer; // 0.13 ms
    qpOASES::QProblem problem_red(new_vars, new_cons);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_red.setOptions(op);
    int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
    // (void)rval;
    int rval2 = problem_red.getPrimalSolution(q_red);
    if (rval2 != qpOASES::SUCCESSFUL_RETURN)
        printf("failed to solve!\n");
    // printf("qp2 solve time: %.3f ms, size %d, %d\n", solve_timer.getMs(), new_vars, new_cons);

    vc = 0;
    for (int i = 0; i < num_variables; ++i) {
        if (var_elim[i]) {
            q_soln[i] = 0.0f;
        } else {
            q_soln[i] = q_red[vc];
            ++vc;
        }
    }
}

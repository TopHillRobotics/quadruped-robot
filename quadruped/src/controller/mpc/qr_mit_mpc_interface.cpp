#include "controller/mpc/qr_mit_mpc_interface.h"

#include "qpOASES.hpp"

#include "robots/qr_timer.h"

using Eigen::Dynamic;
using Eigen::Matrix;
using std::cout;
using std::endl;

void RobotState::set(fpt *p_, fpt *v_, fpt *q_, fpt *w_, fpt *r_, fpt yaw_, fpt total_mass)
{
    for (u8 i = 0; i < 3; i++) {
        this->p(i) = p_[i];
        this->v(i) = v_[i];
        this->w(i) = w_[i];
    }
    this->q.w() = q_[0];
    this->q.x() = q_[1];
    this->q.y() = q_[2];
    this->q.z() = q_[3];
    this->yaw = yaw_;

    // for (u8 rs = 0; rs < 3; rs++)
    //     for (u8 c = 0; c < 4; c++)
    //         this->r_feet(rs, c) = r_[rs * 4 + c];

    for (u8 leg(0); leg < 4; ++leg) {
        for (u8 axis(0); axis < 3; ++axis) {
            this->r_feet(axis, leg) = r_[leg *3 + axis];
        }
    }

    R = this->q.toRotationMatrix();

    // fpt yc = cos(yaw_);
    // fpt ys = sin(yaw_);
    // R_yaw <<  yc,  -ys, 0,
    //           ys,  yc,  0,
    //           0,   0,   1;
    R_yaw = R;

    Matrix<fpt, 3, 1> Id;
    // Id << .07f, 0.26f, 0.242f; // mini cheetah
    // Id << 0.24f, 0.8f, 1.0f;// a1
    Id << 0.4f, 1.15f, 1.0f; // aliengo + z1 arm
    I_body.diagonal() = Id;
    m = total_mass;
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

problem_setup problem_configuration;
u8 gait_data[K_MAX_GAIT_SEGMENTS];
pthread_mutex_t problem_cfg_mt;
pthread_mutex_t update_mt;
update_data_t update;
pthread_t solve_thread;

u8 first_run = 1;

void initialize_mpc()
{
    // printf("Initializing MPC!\n");
    if (pthread_mutex_init(&problem_cfg_mt, NULL) != 0)
        printf("[MPC ERROR] Failed to initialize problem configuration mutex.\n");

    if (pthread_mutex_init(&update_mt, NULL) != 0)
        printf("[MPC ERROR] Failed to initialize update data mutex.\n");

#ifdef K_DEBUG
    printf("[MPC] Debugging enabled.\n");
    printf("[MPC] Size of problem setup struct: %ld bytes.\n", sizeof(problem_setup));
    printf("      Size of problem update struct: %ld bytes.\n", sizeof(update_data_t));
    printf("      Size of MATLAB floating point type: %ld bytes.\n", sizeof(double));
    printf("      Size of fpt: %ld bytes.\n", sizeof(fpt));
#else
        //printf("[MPC] Debugging disabled.\n");
#endif
}

void setup_problem(double dt, int horizon, double mu, double f_max, double total_mass)
{
    // std::cout << "[MPC] setup problem" <<std::endl;
    //mu = 0.6;
    if (first_run) {
        first_run = false;
        initialize_mpc();
    }

#ifdef K_DEBUG
    printf("[MPC] Got new problem configuration!\n");
    printf("[MPC] Prediction horizon length: %d\n      Force limit: %.3f, friction %.3f\n      dt: %.3f\n",
           horizon, f_max, mu, dt);
#endif

    //pthread_mutex_lock(&problem_cfg_mt);
    problem_configuration.total_mass = total_mass;
    problem_configuration.horizon = horizon;
    problem_configuration.f_max = f_max;
    problem_configuration.mu = mu;
    problem_configuration.dt = dt;
    //pthread_mutex_unlock(&problem_cfg_mt);
    resize_qp_mats(horizon);
}

//inline to motivate gcc to unroll the loop in here.
inline void mfp_to_flt(fpt *dst, double *src, s32 n_items)
{
    for (s32 i = 0; i < n_items; i++)
        *dst++ = *src++;
}

inline void mint_to_u8(u8 *dst, int *src, s32 n_items)
{
    for (s32 i = 0; i < n_items; i++)
        *dst++ = *src++;
}

inline void mflt_to_flt(fpt *dst, fpt *src, s32 n_items)
{
    for (s32 i = 0; i < n_items; i++)
        *dst++ = *src++;
}

int has_solved = 0;

//void *call_solve(void* ptr)
//{
//  solve_mpc(&update, &problem_configuration);
//}
//safely copies problem data and starts the solver
void update_problem_data(double *p, double *v, double *q, double *w, double *r, double yaw, double *weights, double *state_trajectory, double alpha, float *gait)
{
    mfp_to_flt(update.p, p, 3);
    mfp_to_flt(update.v, v, 3);
    mfp_to_flt(update.q, q, 4);
    mfp_to_flt(update.w, w, 3);
    mfp_to_flt(update.r, r, 12);
    update.yaw = yaw;
    mfp_to_flt(update.weights, weights, 12);
    //this is safe, the solver isn't running, and update_problem_data and setup_problem
    //are called from the same thread
    mfp_to_flt(update.traj, state_trajectory, 12 * problem_configuration.horizon);
    update.alpha = alpha;
    mflt_to_flt(update.gait, gait, 4 * problem_configuration.horizon);

    solve_mpc(&update, &problem_configuration);
    has_solved = 1;
}

void update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate, double use_jcqp)
{
    update.max_iterations = max_iter;
    update.rho = rho;
    update.sigma = sigma;
    update.solver_alpha = solver_alpha;
    update.terminate = terminate;
    if (use_jcqp > 1.5)
        update.use_jcqp = 2;
    else if (use_jcqp > 0.5)
        update.use_jcqp = 1;
    else
        update.use_jcqp = 0;
}

void update_problem_data_floats(float *p, float *v, float *q, float *w,
                                float *r, float yaw, float *weights,
                                float *state_trajectory, float alpha, float *gait)
{
    update.alpha = alpha;
    update.yaw = yaw;
    // MITTimer t1;
    mflt_to_flt(update.gait, gait, 4 * problem_configuration.horizon);
    memcpy((void *)update.p, (void *)p, sizeof(float) * 3);
    memcpy((void *)update.v, (void *)v, sizeof(float) * 3);
    memcpy((void *)update.q, (void *)q, sizeof(float) * 4);
    memcpy((void *)update.w, (void *)w, sizeof(float) * 3);
    memcpy((void *)update.r, (void *)r, sizeof(float) * 12);
    memcpy((void *)update.weights, (void *)weights, sizeof(float) * 12);
    memcpy((void *)update.traj, (void *)state_trajectory, sizeof(float) * 12 * problem_configuration.horizon);
    // printf("memcpy SOLVE TIME: %.3f  ms\n", t1.getMs());

#ifdef K_DEBUG
    std::cout << "---------------" << std::endl;
    std::cout << "yaw = " << update.yaw << " alpha = " << update.alpha << std::endl;
    std::cout << "p = " << update.p[0] << " " << update.p[1] << " " << update.p[2] << std::endl;
    std::cout << "v = " << update.v[0] << " " << update.v[1] << " " << update.v[2] << std::endl;
    std::cout << "q = " << update.q[0] << " " << update.q[1] << " " << update.q[2] << " " << update.q[3] << std::endl;
    std::cout << "w = " << update.w[0] << " " << update.w[1] << " " << update.w[2] << std::endl;
    std::cout << "r = " << std::endl;
    for (int i = 0; i < 12; i++) {
        std::cout << update.r[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "weights = " << std::endl;
    for (int i = 0; i < 12; i++) {
        std::cout << update.weights[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "trajAll = " << std::endl;
    for (int i = 0; i < problem_configuration.horizon; i++) {
        for (int j = 0; j < 12; j++) {
            std::cout << update.traj[12 * i + j] << " ";
        }
    }
    std::cout << std::endl;
    std::cout << "gait = " << std::endl;
    for (int i = 0; i < problem_configuration.horizon; i++) {
        for (int j = 0; j < 4; j++) {
            std::cout << (int)update.gait[4 * i + j] << " ";
        }
    }
    std::cout << std::endl;
    std::cout << "---------------" << std::endl;
#endif// K_DEBUG

    // MITTimer t2;
    solve_mpc(&update, &problem_configuration);
    // printf("solve_mpc SOLVE TIME: %.3f  ms\n", t2.getMs());
    has_solved = 1;
    // printf("mpc problem has solved = %d\n", has_solved);
}

void update_x_drag(float x_drag)
{
    // printf("x-drag = %f\n", x_drag);
    update.x_drag = x_drag;
}

double get_solution(int index)
{
    if (!has_solved) return 0.f;
    double *qs = get_q_soln();
    return qs[index];
}

//#define K_PRINT_EVERYTHING
#define BIG_NUMBER 5e10
//big enough to act like infinity, small enough to avoid numerical weirdness.
RobotState rs;
//qpOASES::real_t a;

Matrix<fpt, Dynamic, 13> A_qp;
Matrix<fpt, Dynamic, Dynamic> B_qp;
Matrix<fpt, 13, 12> Bdt;
Matrix<fpt, 13, 13> Adt;
Matrix<fpt, 25, 25> ABc, expmm;
Matrix<fpt, Dynamic, Dynamic> S;
Matrix<fpt, Dynamic, 1> X_d;
Matrix<fpt, Dynamic, 1> U_b;
Matrix<fpt, Dynamic, Dynamic> fmat;

Matrix<fpt, Dynamic, Dynamic> qH;
Matrix<fpt, Dynamic, 1> qg;

Matrix<fpt, Dynamic, Dynamic> eye_12h;

qpOASES::real_t *H_qpoases;
qpOASES::real_t *g_qpoases;
qpOASES::real_t *A_qpoases;
qpOASES::real_t *lb_qpoases;
qpOASES::real_t *ub_qpoases;
qpOASES::real_t *q_soln;

qpOASES::real_t *H_red;
qpOASES::real_t *g_red;
qpOASES::real_t *A_red;
qpOASES::real_t *lb_red;
qpOASES::real_t *ub_red;
qpOASES::real_t *q_red;
u8 real_allocated = 0;

char var_elim[2000];
char con_elim[2000];

double *get_q_soln()
{
    return q_soln;
}

bool near_zero(fpt a)
{
    return (a < 0.01 && a > -.01);
}

bool near_one(fpt a)
{
    return near_zero(a - 1);
}

void matrix_to_real(qpOASES::real_t *dst, Matrix<fpt, Dynamic, Dynamic> src, s16 rows, s16 cols)
{
    s32 a = 0;
    for (s16 r = 0; r < rows; ++r) {
        for (s16 c = 0; c < cols; ++c) {
            dst[a] = src(r, c);
            a++;
        }
    }
}

void c2qp(Matrix<fpt, 13, 13> Ac, Matrix<fpt, 13, 12> Bc, fpt dt, s16 horizon)
{
    ABc.setZero();
    ABc.block(0, 0, 13, 13) = Ac;
    ABc.block(0, 13, 13, 12) = Bc;
    ABc = dt * ABc;
    // printf("320\n");
    // const uint16_t size = 20;
    // Matrix<double, size, size> a = Matrix<double, size, size>::Random(size,size);
    // auto b = a.exp();
    // std::cout << b << std::endl;

    // Eigen::Matrix<double,25,25> newABc = ABc.cast<double>();
    // newABc.setIdentity();
    // std::cout << "newABc = " << newABc << std::endl;

    // auto newexpmm = newABc.exp();
    // std::cout << "newexpmm = " << newexpmm <<std::endl;
    expmm = ABc.exp();// newexpmm.cast<float>();
    // throw std::domain_error("323");
    Adt = expmm.block(0, 0, 13, 13);
    Bdt = expmm.block(0, 13, 13, 12);
#ifdef K_PRINT_EVERYTHING
    cout << "Adt: \n"
         << Adt << "\nBdt:\n"
         << Bdt << endl;
#endif
    // if (horizon > 19) {
        // throw std::runtime_error("horizon is too long!");
    // }

    Matrix<fpt, 13, 13> powerMats[20];
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

#ifdef K_PRINT_EVERYTHING
    cout << "AQP:\n"
         << A_qp << "\nBQP:\n"
         << B_qp << endl;
#endif
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
    for (s16 i = 0; i < problem_configuration.horizon; i++) {
        for (s16 j = 0; j < 4; j++) {
            U_b(5 * k + 0) = BIG_NUMBER;
            U_b(5 * k + 1) = BIG_NUMBER;
            U_b(5 * k + 2) = BIG_NUMBER;
            U_b(5 * k + 3) = BIG_NUMBER;
            U_b(5 * k + 4) = BIG_NUMBER;//update->gait[i * 4 + j] * setup->f_max;
            k++;
        }
    }
    fpt mu_ = 1.f / problem_configuration.mu;
    Matrix<fpt, 5, 3> f_block;
    f_block << mu_, 0, 1.f,
            -mu_, 0, 1.f,
            0, mu_, 1.f,
            0, -mu_, 1.f,
            0, 0, 1.f;

    for (s16 i = 0; i < problem_configuration.horizon * 4; i++) {
        fmat.block(i * 5, i * 3, 5, 3) = f_block;
    }

    //TODO: use realloc instead of free/malloc on size changes

    if (real_allocated) {
        free(H_qpoases);
        free(g_qpoases);
        free(A_qpoases);
        free(lb_qpoases);
        free(ub_qpoases);
        free(q_soln);
        free(H_red);
        free(g_red);
        free(A_red);
        free(lb_red);
        free(ub_red);
        free(q_red);
    }

    H_qpoases = (qpOASES::real_t *)malloc(12 * 12 * horizon * horizon * sizeof(qpOASES::real_t));
    mcount += 12 * 12 * h2;
    g_qpoases = (qpOASES::real_t *)malloc(12 * 1 * horizon * sizeof(qpOASES::real_t));
    mcount += 12 * horizon;
    A_qpoases = (qpOASES::real_t *)malloc(12 * 20 * horizon * horizon * sizeof(qpOASES::real_t));
    mcount += 12 * 20 * h2;
    lb_qpoases = (qpOASES::real_t *)malloc(20 * 1 * horizon * sizeof(qpOASES::real_t));
    mcount += 20 * horizon;
    ub_qpoases = (qpOASES::real_t *)malloc(20 * 1 * horizon * sizeof(qpOASES::real_t));
    mcount += 20 * horizon;
    q_soln = (qpOASES::real_t *)malloc(12 * horizon * sizeof(qpOASES::real_t));
    mcount += 12 * horizon;

    H_red = (qpOASES::real_t *)malloc(12 * 12 * horizon * horizon * sizeof(qpOASES::real_t));
    mcount += 12 * 12 * h2;
    g_red = (qpOASES::real_t *)malloc(12 * 1 * horizon * sizeof(qpOASES::real_t));
    mcount += 12 * horizon;
    A_red = (qpOASES::real_t *)malloc(12 * 20 * horizon * horizon * sizeof(qpOASES::real_t));
    mcount += 12 * 20 * h2;
    lb_red = (qpOASES::real_t *)malloc(20 * 1 * horizon * sizeof(qpOASES::real_t));
    mcount += 20 * horizon;
    ub_red = (qpOASES::real_t *)malloc(20 * 1 * horizon * sizeof(qpOASES::real_t));
    mcount += 20 * horizon;
    q_red = (qpOASES::real_t *)malloc(12 * horizon * sizeof(qpOASES::real_t));
    mcount += 12 * horizon;
    real_allocated = 1;

    //printf("malloc'd %d floating point numbers.\n",mcount);
#ifdef K_DEBUG
    printf("RESIZED MATRICES FOR HORIZON: %d\n", horizon);
#endif
}

inline Matrix<fpt, 3, 3> cross_mat(Matrix<fpt, 3, 3> I_inv, Matrix<fpt, 3, 1> r)
{
    Matrix<fpt, 3, 3> cm;
    cm << 0.f, -r(2), r(1),
        r(2), 0.f, -r(0),
        -r(1), r(0), 0.f;
    return I_inv * cm;
}

//continuous time state space matrices.
void ct_ss_mats(Matrix<fpt, 3, 3> I_world, fpt m, Matrix<fpt, 3, 4> r_feet, Matrix<fpt, 3, 3> R_yaw, Matrix<fpt, 13, 13> &A, Matrix<fpt, 13, 12> &B, float x_drag)
{
    A.setZero();
    A(3, 9) = 1.f;
    A(11, 9) = x_drag;
    A(4, 10) = 1.f;
    A(5, 11) = 1.f;

    A(11, 12) = 1.f;
    A.block(0, 6, 3, 3) = R_yaw.transpose();

    B.setZero();
    Matrix<fpt, 3, 3> I_inv = I_world.inverse();

    for (s16 b = 0; b < 4; b++) {
        B.block(6, b * 3, 3, 3) = cross_mat(I_inv, r_feet.col(b));
        B.block(9, b * 3, 3, 3) = Matrix<fpt, 3, 3>::Identity() / m;
    }
}

void quat_to_rpy(Eigen::Quaternionf q, Matrix<fpt, 3, 1> &rpy)
{
    //from my MATLAB implementation
    //edge case!
    fpt as = t_min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
    rpy(0) = atan2(2.f * (q.x() * q.y() + q.w() * q.z()), sq(q.w()) + sq(q.x()) - sq(q.y()) - sq(q.z()));// yaw
    rpy(1) = asin(as);                                                                                   // pitch
    rpy(2) = atan2(2.f * (q.y() * q.z() + q.w() * q.x()), sq(q.w()) - sq(q.x()) - sq(q.y()) + sq(q.z()));// roll
}

void print_problem_setup(problem_setup *setup)
{
    printf("MASS: %.3f\n", setup->total_mass);
    printf("DT: %.3f\n", setup->dt);
    printf("Mu: %.3f\n", setup->mu);
    printf("F_Max: %.3f\n", setup->f_max);
    printf("Horizon: %d\n", setup->horizon);
}

void print_update_data(update_data_t *update, s16 horizon)
{
    print_named_array("p", update->p, 1, 3);
    print_named_array("v", update->v, 1, 3);
    print_named_array("q", update->q, 1, 4);
    print_named_array("w", update->r, 3, 4);
    pnv("Yaw", update->yaw);
    print_named_array("weights", update->weights, 1, 12);
    print_named_array("trajectory", update->traj, horizon, 12);
    pnv("Alpha", update->alpha);
    print_named_array("gait", update->gait, horizon, 4);
}

Eigen::Matrix<fpt, 13, 1> x_0;
Eigen::Matrix<fpt, 3, 3> I_world;
Eigen::Matrix<fpt, 13, 13> A_ct;
Eigen::Matrix<fpt, 13, 12> B_ct_r;
float tt = 0;
int numm = 0;
// for osqp solver
// #include "OsqpEigen/OsqpEigen.h"
// OsqpEigen::Solver solver;
// int last_new_vars = 0;
// int last_new_cons = 0;

void solve_mpc(update_data_t *update, problem_setup *setup)
{
    // MITTimer t1;
    rs.set(update->p, update->v, update->q, update->w, update->r, update->yaw, setup->total_mass);
#ifdef K_PRINT_EVERYTHING
    printf("-----------------\n");
    printf("   PROBLEM DATA  \n");
    printf("-----------------\n");
    print_problem_setup(setup);

    printf("-----------------\n");
    printf("    ROBOT DATA   \n");
    printf("-----------------\n");
    rs.print();
    print_update_data(update, setup->horizon);
#endif

    //roll pitch yaw
    Matrix<fpt, 3, 1> rpy;
    quat_to_rpy(rs.q, rpy);
    //initial state (13 state representation)
    x_0 << rpy(2), rpy(1), rpy(0), rs.p, rs.w, rs.v, -9.8f;
    I_world = rs.R_yaw * rs.I_body * rs.R_yaw.transpose();//original
    ct_ss_mats(I_world, rs.m, rs.r_feet, rs.R_yaw, A_ct, B_ct_r, update->x_drag);

#ifdef K_PRINT_EVERYTHING
    cout << "Initial state: \n"
         << x_0 << endl;
    cout << "World Inertia: \n"
         << I_world << endl;
    cout << "A CT: \n"
         << A_ct << endl;
    cout << "B CT (simplified): \n"
         << B_ct_r << endl;
#endif

    //QP matrices
    c2qp(A_ct, B_ct_r, setup->dt, setup->horizon); // 0.03ms

    //weights
    Matrix<fpt, 13, 1> full_weight;
    for (u8 i = 0; i < 12; i++)
        full_weight(i) = update->weights[i];
    full_weight(12) = 0.f;
    S.diagonal() = full_weight.replicate(setup->horizon, 1);

    //trajectory
    for (s16 i = 0; i < setup->horizon; i++) {
        for (s16 j = 0; j < 12; j++)
            X_d(13 * i + j, 0) = update->traj[12 * i + j];
    }

    //note - I'm not doing the shifting here.
    s16 k = 0;
    for (s16 i = 0; i < setup->horizon; ++i) {
        for (s16 j = 0; j < 4; ++j) {
            // U_b(5 * k + 0) = BIG_NUMBER;
            // U_b(5 * k + 1) = BIG_NUMBER;
            // U_b(5 * k + 2) = BIG_NUMBER;
            // U_b(5 * k + 3) = BIG_NUMBER;
            U_b(5 * k + 4) = update->gait[i * 4 + j] * setup->f_max;
            k++;
        }
    }
    // printf("qp1 solve time: %.3f ms\n", t1.getMs());


    // fpt mu = 1.f / setup->mu;
    // Matrix<fpt, 5, 3> f_block;
    // f_block << mu, 0, 1.f,
    //         -mu, 0, 1.f,
    //         0, mu, 1.f,
    //         0, -mu, 1.f,
    //         0, 0, 1.f;

    // for (s16 i = 0; i < setup->horizon * 4; i++) {
    //     fmat.block(i * 5, i * 3, 5, 3) = f_block;
    // }

    // these two matrix computation is time-costly.
    // for (int i=0; i < 20; ++i) {
    // MITTimer T2;
            // std::cout << "s = " <<S << std::endl;
        // Eigen::MatrixXf temp = 2 * B_qp.transpose() * S;   // (13*5, 12*5).T * (13*5, 13*5) ==> (12*5, 13*5)
        Eigen::MatrixXf temp;
        temp.resize(12*setup->horizon, 13*setup->horizon);
        temp.setZero();
        Eigen::Matrix<float, 12, 13> subTemp, Bij;
        for (u8 i(0); i<setup->horizon; ++i) {
            for (u8 j(i); j< setup->horizon; ++j) {
                // B^(ij) = Bqp_{j,i}.T
                Bij = B_qp.block(j*13, i*12, 13, 12).transpose();
                // for (u8 m(0); m<12; ++m) {
                    for (u8 n(0); n < 13; ++n) {
                        // B^(ij) * S^{j} = B^{ij}_{m}*S_{j}^{m}
                        subTemp.col(n) = Bij.col(n)*(2*full_weight(n)); // S(n,n);
                    }
                // }
                temp.block(i*12,j*13,12,13) = subTemp;
            }
        }
        // std::cout << "norm = " << (temp - temp1).norm() << std::endl;
        qH = temp * B_qp + (2 * update->alpha) * eye_12h; // 0.07ms
        qg = temp * (A_qp * x_0 - X_d); // 0.03ms   - (13*5, 1)
    // }

    s16 num_constraints = 20 * setup->horizon;
    s16 num_variables = 12 * setup->horizon;

    // QpProblem<double> jcqp(num_variables, num_constraints); // 0.06ms

    // tt += T2.getMs();
    // numm++;
    // printf("qp1.2 solve time: %.3f ms\n", tt / numm);

    if (update->use_jcqp == 1) {
        // jcqp.A = fmat.cast<double>();
        // jcqp.P = qH.cast<double>();
        // jcqp.q = qg.cast<double>();
        // jcqp.u = U_b.cast<double>();
        // for (s16 i = 0; i < num_constraints; i++)
        //     jcqp.l[i] = 0.;

        // jcqp.settings.sigma = update->sigma;
        // jcqp.settings.alpha = update->solver_alpha;
        // jcqp.settings.terminate = update->terminate;
        // jcqp.settings.rho = update->rho;
        // jcqp.settings.maxIterations = update->max_iterations;
        // jcqp.runFromDense(update->max_iterations, true, false);
    } else {
        // MITTimer solve_timer1;
        matrix_to_real(H_qpoases, qH, num_variables, num_variables);
        matrix_to_real(g_qpoases, qg, num_variables, 1);
        matrix_to_real(A_qpoases, fmat, num_constraints, num_variables);
        matrix_to_real(ub_qpoases, U_b, num_constraints, 1);

        for (s16 i = 0; i < num_constraints; ++i)
            lb_qpoases[i] = 0.0f;

        qpOASES::int_t nWSR = 100;

        int new_vars = num_variables;
        int new_cons = num_constraints;
        // std::cout << "num_variables = "<< num_variables << ", num_constraints = " << num_constraints <<std::endl;

        for (int i = 0; i < num_constraints; ++i)
            con_elim[i] = 0;

        for (int i = 0; i < num_variables; ++i)
            var_elim[i] = 0;

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


        // std::cout << "new num_variables = "<< new_vars << ", num_constraints = " << new_cons <<std::endl;
        //if(new_vars != num_variables)
        if (1 == 1) {
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
                    vc++;
                }
            }
            vc = 0;
            for (int i = 0; i < num_constraints; ++i) {
                if (!con_elim[i]) {
                    if (!(vc < new_cons)) {
                        printf("BAD ERROR 1\n");
                    }
                    con_ind[vc] = i;
                    vc++;
                }
            }

            // osqp solver
            /*
            if (update->use_jcqp == -1) {
                MITTimer T1;
                Eigen::MatrixXd hessian; hessian.resize(new_vars, new_vars);
                Eigen::VectorXd gradient; gradient.resize(new_vars);
                Eigen::MatrixXd linearMatrix; linearMatrix.resize(new_cons, new_vars);
                Eigen::VectorXd lowerBound; lowerBound.resize(new_cons);
                Eigen::VectorXd upperBound; upperBound.resize(new_cons);

                for (int i(0); i < new_vars; ++i) {
                    int olda = var_ind[i];
                    gradient[i] = g_qpoases[olda];
                    for (int j = 0; j < new_vars; j++) {
                        int oldb = var_ind[j];
                        hessian(i, j) = H_qpoases[olda * num_variables + oldb];
                    }
                }

                for (int con = 0; con < new_cons; con++) {
                    for (int st = 0; st < new_vars; st++) {
                        float cval = A_qpoases[(num_variables * con_ind[con]) + var_ind[st]];
                        linearMatrix(con, st) = cval;
                    }
                }
                for (int i = 0; i < new_cons; i++) {
                    int old = con_ind[i];
                    upperBound[i] = ub_qpoases[old];
                    lowerBound[i] = lb_qpoases[old];
                }

                Eigen::SparseMatrix<double> H = hessian.sparseView();
                Eigen::SparseMatrix<double> G = linearMatrix.sparseView();

                if (!solver.isInitialized() ) {
                    solver.settings()->setVerbosity(false);
                    solver.settings()->setWarmStart(true);
                    solver.settings()->setPolish(false);
                    solver.settings()->setTimeLimit(0.0002);
                    solver.settings()->setAdaptiveRho(true);
                    solver.settings()->setAbsoluteTolerance(0.01);
                    solver.settings()->setMaxIteration(100);
                    solver.settings()->setCheckTermination(1);
                    // solver.settings()->setLinearSystemSolver(MKL_PARDISO_SOLVER);

                    // set the initial data of the QP solver
                    solver.data()->setNumberOfVariables(new_vars);
                    solver.data()->setNumberOfConstraints(new_cons);
                    if(!solver.data()->setHessianMatrix(H)) throw std::domain_error("695");;
                    if(!solver.data()->setGradient(gradient)) throw std::domain_error("696");
                    if(!solver.data()->setLinearConstraintsMatrix(G)) throw std::domain_error("697");
                    if(!solver.data()->setLowerBound(lowerBound)) throw std::domain_error("698");
                    if(!solver.data()->setUpperBound(upperBound)) throw std::domain_error("699");
                    if(!solver.initSolver()) throw std::domain_error("732");

                } else {
                    if (new_vars!= last_new_vars || new_cons != last_new_cons) {
                        last_new_vars = new_vars;
                        last_new_cons = new_cons;
                        solver.clearSolver();
                        solver.data()->clearHessianMatrix();
                        solver.data()->clearLinearConstraintsMatrix();
                        solver.data()->setNumberOfConstraints(new_cons);
                        solver.data()->setNumberOfVariables(new_vars);
                        solver.data()->setHessianMatrix(H);
                        solver.data()->setGradient(gradient);
                        solver.data()->setLinearConstraintsMatrix(G);
                        solver.data()->setLowerBound(lowerBound);
                        solver.data()->setUpperBound(upperBound);
                        solver.initSolver();
                    } else {
                        solver.updateHessianMatrix(H);
                        solver.updateGradient(gradient);
                        solver.updateLinearConstraintsMatrix(G);
                        solver.updateLowerBound(lowerBound);
                        solver.updateUpperBound(upperBound);
                    }
                }
                printf("set probelm time %f ms\n", T1.getMs());

                solver.solveProblem();
                printf("solveProblem Solve time %f ms\n", T1.getMs());

                Eigen::VectorXd QPSolution = solver.getSolution();
                vc = 0;
                for (int i = 0; i < num_variables; i++) {
                    if (var_elim[i]) {
                        q_soln[i] = 0.0f;
                    } else {
                        q_soln[i] = QPSolution[vc];
                        vc++;
                    }
                }
                return;
            }
            */

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


            if (update->use_jcqp == 0) { // 0.13 ms
                // MITTimer solve_timer;
                qpOASES::QProblem problem_red(new_vars, new_cons);
                qpOASES::Options op;
                op.setToMPC();
                op.printLevel = qpOASES::PL_NONE;
                problem_red.setOptions(op);
                //int_t nWSR = 50000;

                int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
                (void)rval;
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
                        vc++;
                    }
                }
            } else {// use jcqp == 2
                QpProblem<double> reducedProblem(new_vars, new_cons);

                reducedProblem.A = DenseMatrix<double>(new_cons, new_vars);
                int i = 0;
                for (int r = 0; r < new_cons; r++) {
                    for (int c = 0; c < new_vars; c++) {
                        reducedProblem.A(r, c) = A_red[i++];
                    }
                }

                reducedProblem.P = DenseMatrix<double>(new_vars, new_vars);
                i = 0;
                for (int r = 0; r < new_vars; r++) {
                    for (int c = 0; c < new_vars; c++) {
                        reducedProblem.P(r, c) = H_red[i++];
                    }
                }

                reducedProblem.q = Eigen::Matrix<double, Eigen::Dynamic, 1>(new_vars);
                for (int r = 0; r < new_vars; r++) {
                    reducedProblem.q[r] = g_red[r];
                }

                reducedProblem.u = Eigen::Matrix<double, Eigen::Dynamic, 1>(new_cons);
                for (int r = 0; r < new_cons; r++) {
                    reducedProblem.u[r] = ub_red[r];
                }

                reducedProblem.l = Eigen::Matrix<double, Eigen::Dynamic, 1>(new_cons);
                for (int r = 0; r < new_cons; r++) {
                    reducedProblem.l[r] = lb_red[r];
                }

                //        jcqp.A = fmat.cast<double>();
                //        jcqp.P = qH.cast<double>();
                //        jcqp.q = qg.cast<double>();
                //        jcqp.u = U_b.cast<double>();
                //        for(s16 i = 0; i < 20*setup->horizon; i++)
                //          jcqp.l[i] = 0.;

                reducedProblem.settings.sigma = update->sigma;
                reducedProblem.settings.alpha = update->solver_alpha;
                reducedProblem.settings.terminate = update->terminate;
                reducedProblem.settings.rho = update->rho;
                reducedProblem.settings.maxIterations = update->max_iterations;
                reducedProblem.runFromDense(update->max_iterations, true, false);

                vc = 0;
                for (int kk = 0; kk < num_variables; kk++) {
                    if (var_elim[kk]) {
                        q_soln[kk] = 0.0f;
                    } else {
                        q_soln[kk] = reducedProblem.getSolution()[vc];
                        vc++;
                    }
                }
            }
        }
    }
    // if (update->use_jcqp == 1) {
    //     for (int i = 0; i < 12 * setup->horizon; i++) {
    //         q_soln[i] = jcqp.getSolution()[i];
    //     }
    // }
}

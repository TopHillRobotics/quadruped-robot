#ifndef COVVEXMPC_INTERFACE
#define COVVEXMPC_INTERFACE
#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

#include "utils/qp_problem.hpp"
#include "utils/se3.h"
#include "utils/tools.h"
#include <cstdio>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <include/qpOASES.hpp>

#define K_MAX_GAIT_SEGMENTS 16

class RobotState {
public:
    void set(float *p, float *v, float *q, float *w, float *r, float *rpy, float total_mass);
    void print();
    Eigen::Matrix<float, 3, 1> p, v, w;//定义了机器人的在世界坐标系下的，位置p，速度p˙，以及机身坐标系下的旋转角ω,均为3×1矩阵
    Eigen::Matrix<float, 3, 4> r_feet; //机身参考系下的足端位置，3×4矩阵
    Mat3<float> R;                     //机身坐标系到世界坐标系的旋转矩阵
    Mat3<float> R_yaw;                 //偏航角旋转矩阵
    Mat3<float> I_body;                //机身坐标系下的惯量矩阵
    Eigen::Quaternionf q;              //四元素表示的世界坐标系下的旋转
    Eigen::Matrix<float, 3, 1> rpy;
    float m = 12;// 12,20              //机器人质量
};

struct ProblemConfig {
    float dt;
    float mu;
    float f_max;
    int horizon;
    float total_mass;
    float weights[12];
    float alpha;
};

/**
 * @brief robot sensor data at timeStamp T, needed by mpc
 */
struct UpdateDataT {
    float p[3];
    float v[3];
    float q[4];
    float w[3];
    float r[12];
    float rpy[3];
    float traj[12 * K_MAX_GAIT_SEGMENTS];
    float gait[K_MAX_GAIT_SEGMENTS];
};

EXTERNC void SetupProblem(double dt, int horizon, double mu, double f_max,
                          double total_mass, float *inertia, float *weight, float alpha);
EXTERNC double GetMPCSolution(int index);
EXTERNC void SolveMPCKernel(float *p, float *v, float *q, float *w,
                            float *r, float *rpy,
                            float *state_trajectory, float *gait);

void resize_qp_mats(s16 horizon);
void solve_mpc(UpdateDataT *update, ProblemConfig *setup);
void ct_ss_mats(Mat3<float> I_world, float m, Eigen::Matrix<float, 3, 4> r_feet, Mat3<float> R_yaw,
                Eigen::Matrix<float, 13, 13> &A, Eigen::Matrix<float, 13, 12> &B);
void c2qp(Eigen::Matrix<float, 13, 13> Ac, Eigen::Matrix<float, 13, 12> Bc, float dt, s16 horizon);

#endif// COVVEXMPC_INTERFACE
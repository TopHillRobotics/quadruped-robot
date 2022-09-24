#ifndef QR_MIT_MPC_INTERFACE_H
#define QR_MIT_MPC_INTERFACE_H

#define K_MAX_GAIT_SEGMENTS 16

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

#include <sys/time.h>
#include <cstdio>
#include <vector>
#include <pthread.h>
#include <string.h>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include "common/qr_cTypes.h"
#include "qr_qp_problem.h"

// TODO: check what is the difference between RobotState and qrRobotState
class RobotState
{
    public:
        void set(fpt* p, fpt* v, fpt* q, fpt* w, fpt* r, fpt yaw, fpt total_mass);
        //void compute_rotations();
        void print();
        Eigen::Matrix<fpt,3,1> p, v, w;//定义了机器人的在世界坐标系下的，位置p，速度p˙，以及机身坐标系下的旋转角ω,均为3×1矩阵
        Eigen::Matrix<fpt,3,4> r_feet; //机身参考系下的足端位置，3×4矩阵
        Eigen::Matrix<fpt,3,3> R;      //机身坐标系到世界坐标系的旋转矩阵
        Eigen::Matrix<fpt,3,3> R_yaw;  //偏航角旋转矩阵
        Eigen::Matrix<fpt,3,3> I_body; //机身坐标系下的惯量矩阵
        Eigen::Quaternionf q;          //四元素表示的世界坐标系下的旋转
        fpt yaw;                       //偏航角
        fpt m = 12; // 12,20           //机器人质量
};

struct problem_setup
{
  fpt dt;
  fpt mu;
  fpt f_max;
  int horizon;
  fpt total_mass;
};

struct update_data_t
{
  fpt p[3];
  fpt v[3];
  fpt q[4];
  fpt w[3];
  fpt r[12];
  fpt yaw;
  fpt weights[12];
  fpt traj[12*K_MAX_GAIT_SEGMENTS];
  fpt alpha;
//   unsigned char gait[K_MAX_GAIT_SEGMENTS];
  fpt gait[K_MAX_GAIT_SEGMENTS];

  unsigned char hack_pad[1000];
  int max_iterations;
  double rho, sigma, solver_alpha, terminate;
  int use_jcqp;
  fpt x_drag;
};

EXTERNC void setup_problem(double dt, int horizon, double mu, double f_max, double total_mass);
EXTERNC void update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait);
EXTERNC double get_solution(int index);
EXTERNC void update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate, double use_jcqp);
EXTERNC void update_problem_data_floats(fpt* p, fpt* v, fpt* q, fpt* w,
                                        fpt* r, fpt yaw, fpt* weights,
                                        fpt* state_trajectory, fpt alpha, fpt* gait);

void update_x_drag(fpt x_drag);

template <class T>
void print_array(T* array, u16 rows, u16 cols)
{
    for(u16 r = 0; r < rows; r++)
    {
        for(u16 c = 0; c < cols; c++)
            std::cout<<(fpt)array[c+r*cols]<<" ";
        printf("\n");
    }
}

template <class T>
void print_named_array(const char* name, T* array, u16 rows, u16 cols)
{
    printf("%s:\n",name);
    print_array(array,rows,cols);
}

//print named variable
template <class T>
void pnv(const char* name, T v)
{
    printf("%s: ",name);
    std::cout<<v<<std::endl;
}

template <class T>
T t_min(T a, T b)
{
    if(a<b) return a;
    return b;
}

template <class T>
T sq(T a)
{
    return a*a;
}

void solve_mpc(update_data_t* update, problem_setup* setup);
void quat_to_rpy(Eigen::Quaternionf q, Eigen::Matrix<fpt,3,1>& rpy);
void ct_ss_mats(Eigen::Matrix<fpt,3,3> I_world, fpt m, Eigen::Matrix<fpt,3,4> r_feet, Eigen::Matrix<fpt,3,3> R_yaw,
                Eigen::Matrix<fpt,13,13>& A, Eigen::Matrix<fpt,13,12>& B);
void resize_qp_mats(s16 horizon);
void c2qp(Eigen::Matrix<fpt,13,13> Ac, Eigen::Matrix<fpt,13,12> Bc,fpt dt,s16 horizon);
double* get_q_soln();


#endif // QR_MIT_MPC_INTERFACE_H

/*
 * tiny_ekf_struct.h: common data structure for TinyEKF
 *
 * You should #include this file after using #define for N (states) and M
*  (observations)
 *
 * Copyright (C) 2016 Simon D. Levy
 *
 * MIT License
 */

#ifndef TINYEKF_TINYEKF_STRUCT_H
#define TINYEKF_TINYEKF_STRUCT_H


// typedef struct {

//     int n;          /* number of state values */
//     int m;          /* number of observables */

//     double x[Nsta];    /* state vector */

//     double P[Nsta][Nsta];  /* prediction error covariance */
//     double Q[Nsta][Nsta];  /* process noise covariance */
//     double R[Mobs][Mobs];  /* measurement error covariance */

//     double G[Nsta][Mobs];  /* Kalman gain; a.k.a. K */

//     double F[Nsta][Nsta];  /* Jacobian of process model */
//     double H[Mobs][Nsta];  /* Jacobian of measurement model */

//     double Ht[Nsta][Mobs]; /* transpose of measurement Jacobian */
//     double Ft[Nsta][Nsta]; /* transpose of process Jacobian */
//     double Pp[Nsta][Nsta]; /* P, post-prediction, pre-update */

//     double fx[Nsta];   /* output of user defined f() state-transition function */
//     double hx[Mobs];   /* output of user defined h() measurement function */

//     /* temporary storage */
//     double tmp0[Nsta][Nsta];
//     double tmp1[Nsta][Mobs];
//     double tmp2[Mobs][Nsta];
//     double tmp3[Mobs][Mobs];
//     double tmp4[Mobs][Mobs];
//     double tmp5[Mobs]; 

// } ekf_t;      



template<unsigned int N=3, unsigned int M=3>
struct ekf_t{

    int n=N;          /* number of state values */
    int m=M;          /* number of observables */

    double x[N];    /* state vector */

    double P[N][N];  /* prediction error covariance */
    double Q[N][N];  /* process noise covariance */
    double R[M][M];  /* measurement error covariance */

    double G[N][M];  /* Kalman gain; a.k.a. K */

    double F[N][N];  /* Jacobian of process model */
    double H[M][N];  /* Jacobian of measurement model */

    double Ht[N][M]; /* transpose of measurement Jacobian */
    double Ft[N][N]; /* transpose of process Jacobian */
    double Pp[N][N]; /* P, post-prediction, pre-update */

    double fx[N];   /* output of user defined f() state-transition function */
    double hx[M];   /* output of user defined h() measurement function */

    /* temporary storage */
    double tmp0[N][N];
    double tmp1[N][M];
    double tmp2[M][N];
    double tmp3[M][M];
    double tmp4[M][M];
    double tmp5[M]; 

};

#endif // TINYEKF_TINYEKF_STRUCT_H
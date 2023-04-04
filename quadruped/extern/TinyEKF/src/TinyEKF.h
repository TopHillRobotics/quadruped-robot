/*
 * TinyEKF: Extended Kalman Filter for Arduino and TeensyBoard.
 *
 * Copyright (C) 2015 Simon D. Levy
 *
 * MIT License
 */

#ifndef TINYEKF_TINYEKF_TINYEKF_H
#define TINYEKF_TINYEKF_TINYEKF_H

#include <stdio.h>
#include <stdlib.h>
#include "tiny_ekf_struct.h"
//#include "tiny_ekf.h"

// Support both Arduino and command-line versions
 #ifdef _cplusplus
 extern "C" {
 #endif
     #include "tiny_ekf.h"
     void ekf_init(void *, int, int);
     int ekf_step(void *, double *);
 #ifdef _cplusplus
 }
 #endif

/**
 * A header-only class for the Extended Kalman Filter.  Your implementing class should #define the constant N and 
 * and then #include <TinyEKF.h>  You will also need to implement a model() method for your application.
 */

template<unsigned int N=Nsta, unsigned int M=Mobs>
class TinyEKF {

private:
    ekf_t<N,M> ekf;

public:    // protected:
    // ekf_t ekf;
    double * x; // The current state
    
    // Initializes a TinyEKF object
    TinyEKF() 
    { 
        ekf_init(&this->ekf, N, M); 
        this->x = this->ekf.x; 
    }

    TinyEKF(float x, float initialVariance, float accelerometerVariance, float sensorVariance)
    {
        ekf_init(&this->ekf, N, M);
        for(int i=0; i < N; ++i) {
            this->ekf.x[i] = x;
            this->x = this->ekf.x;
        }
        // F.setIdentity();
        // C.setIdentity();
        
        // P.setIdentity();
        // Q.setIdentity();
        // R.setIdentity(); 
        // P = P * initialVariance;
        for(int i = 0; i < N; ++i) {
            this->setQ(i, i, accelerometerVariance);
        }
        for (int i = 0; i < M; ++i) {
            this->setR(i, i, sensorVariance);
        }
    };

    TinyEKF(float* x, float initialVariance, float* accelerometerVariance, float* sensorVariance)
    {
        ekf_init(&this->ekf, N, M);
        for(int i=0; i < N; ++i) {
            this->ekf.x[i] = x[i];
            this->x = this->ekf.x;
        }
        // F.setIdentity();
        // C.setIdentity();
        
        // P.setIdentity();
        // Q.setIdentity();
        // R.setIdentity(); 
        // P = P * initialVariance;
        for(int i = 0; i < N; ++i) {
            this->setQ(i, i, accelerometerVariance[i]);
            // this->setP(i, i, initialVariance);
        }
        for (int i = 0; i < M; ++i) {
            this->setR(i, i, sensorVariance[i]);
        }
    };

    /**
     * Deallocates memory for a TinyEKF object.
     */
    ~TinyEKF() { }

    /**
     * Implement this function for your EKF model.
     * @param fx gets output of state-transition function <i>f(x<sub>0 .. n-1</sub>)</i>
     * @param F gets <i>n &times; n</i> Jacobian of <i>f(x)</i>
     * @param hx gets output of observation function <i>h(x<sub>0 .. n-1</sub>)</i>
     * @param H gets <i>m &times; n</i> Jacobian of <i>h(x)</i>
     */
    // virtual void model(double fx[N], double F[N][N], double hx[M], double H[M][N]) = 0;
    void model(double fx[N], double F[N][N], double hx[M], double H[M][N], double deltaV[N], double z[M])
    {
        // Process model is f(x) = x
        fx[0] = this->x[0] + deltaV[0];
        fx[1] = this->x[1] + deltaV[1];
        fx[2] = this->x[2] + deltaV[2];
        // so process model Jacobian is identity matrix
        F[0][0] = 1;
        F[1][1] = 1;
        F[2][2] = 1;
        // Measurement function simplifies the relationshaip between state and senor reading for convenience.
        // a more realistic measurement function would distinguish between state value and measured value;
        hx[0] = fx[0];  //z[0]
        hx[1] = fx[1];
        hx[2] = fx[2];
        // Jacobian of measurement function
        H[0][0] = 1;
        H[1][1] = 1;
        H[2][2] = 1;
    }
    /**
     * Sets the specified value of the prediction error covariance. <i>P<sub>i,j</sub> = value</i>
     * @param i row index
     * @param j column index
     * @param value value to set
     */
    void setP(int i, int j, double value) 
    { 
        this->ekf.P[i][j] = value; 
    }

    /**
     * Sets the specified value of the process noise covariance. <i>Q<sub>i,j</sub> = value</i>
     * @param i row index
     * @param j column index
     * @param value value to set
     */
    void setQ(int i, int j, double value) 
    { 
        this->ekf.Q[i][j] = value; 
    }

    /**
     * Sets the specified value of the observation noise covariance. <i>R<sub>i,j</sub> = value</i>
     * @param i row index
     * @param j column index
     * @param value value to set
     */
    void setR(int i, int j, double value) 
    { 
        this->ekf.R[i][j] = value; 
    }

public:

    /**
     * Returns the state element at a given index.
     * @param i the index (at least 0 and less than <i>n</i>
     * @return state value at index
     */
    double getX(int i) 
    { 
        return this->ekf.x[i]; 
    }


    /**
     * Sets the state element at a given index.
     * @param i the index (at least 0 and less than <i>n</i>
     * @param value value to set
     */
    void setX(int i, double value) 
    { 
        this->ekf.x[i] = value; 
    }

    /**
         Performs one step of the prediction and update.
        * @param z observation vector, length <i>m</i>
        * @return true on success, false on failure caused by non-positive-definite matrix.
        */
    bool step(double* deltaV, double * z)
    {
        this->model(this->ekf.fx, this->ekf.F, this->ekf.hx, this->ekf.H, deltaV, z); 

        return ekf_step(&this->ekf, z) ? false : true;
    }
    
    bool step(double fx[N], double F[N][N], double hx[M], double H[M][N], double* z)
    {
        memcpy(this->ekf.fx, fx, N*sizeof(double)); 
        memcpy(this->ekf.F,  F, N*N*sizeof(double));
        memcpy(this->ekf.hx, hx, M*sizeof(double));
        memcpy(this->ekf.H,  H, M*N*sizeof(double));
        
        return ekf_step(&this->ekf, z) ? false : true;
    }
};


#endif // TINYEKF_TINYEKF_TINYEKF_H

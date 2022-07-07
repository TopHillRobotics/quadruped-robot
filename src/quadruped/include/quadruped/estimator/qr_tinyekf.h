// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:tophill.robotics@gmail.com

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

#ifndef QR_TINYEKF_H
#define QR_TINYEKF_H

// must define here because TinyEKF.h will use this
#define Nsta 3 // dimension of state
#define Mobs 3 // dimension of observation

#include <Eigen/Dense>
#include <tinyekf/TinyEKF.h>

#include "common/qr_eigen_types.h"

class qrTinyEKF: public TinyEKF {

public:

  /**
   * @brief constructor of qrTinyEKF
   */
  qrTinyEKF();

  /**
   * @brief destructor of qrTinyEKF
   */
  virtual ~qrTinyEKF();

  /**
   * @brief qrTinyEKF
   * @param x: state
   * @param modelVariance: variance of the state model
   * @param sensorVariance: variance of the observation
   */
  qrTinyEKF(float x, float modelVariance, float sensorVariance);


  /**
   * @see TinyEKF
   * @param deltaV: ideal state conversion
   */
  bool step(double* deltaX, double* z);

  /**
   * @brief same as step, but use eigen vector
   */
  bool step(Vec3<float> deltaX, Vec3<float> z);

protected:

  /**
   * @see TinyEKF
   */
  void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]);

  /**
   * @see TinyEKF
   * @param deltaV: state increment
   */
  void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta], double deltaV[Nsta]);
};

#endif // QR_TINYEKF_H

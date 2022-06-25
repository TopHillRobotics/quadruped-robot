#ifndef QR_TINYEKF_H
#define QR_TINYEKF_H

// must define here because TinyEKF.h will use this
#define Nsta 3 // dimension of state
#define Mobs 3 // dimension of observation

#include <Eigen/Dense>

#include <TinyEKF.h>

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
  bool step(Eigen::Matrix<float, 3, 1> deltaX, Eigen::Matrix<float, 3, 1> z);

protected:

  /**
   * @see TinyEKF
   */
  void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) override;

  /**
   * @see TinyEKF
   * @param deltaV: state increment
   */
  void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta], double deltaV[Nsta]);
};

#endif // QR_TINYEKF_H

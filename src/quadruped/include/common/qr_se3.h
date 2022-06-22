#ifndef QR_SE3_H
#define QR_SE3_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "common/qr_eigentypes.h"

namespace Math {

  /**
   * @brief The Axis enum specifies axis in coordinate. 0: X axis; 1: Y axis; 2: Z Axis
   */
  enum Axis{X, Y, Z};

  /**
   * @brief convert roll ptch yaw to rotation matrix
   * @param rpy: reference to vector of roll ptch yaw
   */
  template<typename MT>
  Mat3<typename MT::Scalar> rpy2RotMat(const Eigen::MatrixBase<MT> &rpy);

  /**
   * @brief convert raw pitch yaw to quaterion
   * @param rpy: reference to vector of roll ptch yaw
   * @return quaterion
   */
  template<typename MT>
  Quat<typename MT::Scalar> rpy2Quat(const Eigen::MatrixBase<MT> &rpy);

  /**
   * @brief convert rotation matrix to quaterion
   * @param R: reference to rotation matrix
   * @return quaterion
   */
  template<typename MT>
  Quat<typename MT::Scalar> rotMat2Quat(const Eigen::MatrixBase<MT> &R);

  /**
   * @param axis: the rotation axis
   * @param theta: the rotation angle
   * @return basic rotation matrix
   */
  template<typename T>
  Mat3<T> basicRotMat(Axis axis, T theta);

  /**
   * @brief the rotation order is X axis, Y axis and Z axis, all fixed coordinate
   * @param xTheta: rotation around x axis
   * @param yTheta: rotation around y axis
   * @param zTheta: rotation around z axis
   */
  template<typename T>
  Mat3<T> genericRotMat(T xTheta, T yTheta, T zTheta);
}

template<typename MT>
Quat<typename MT::Scalar> Math::rpy2Quat(const Eigen::MatrixBase<MT> &rpy)
{
  Mat3<typename MT::Scalar> R = rpy2RotMat(rpy);
  Quat<typename MT::Scalar> q = rotMat2Quat(R);
  return q;
}

template<typename MT>
Mat3<typename MT::Scalar> Math::rpy2RotMat(const Eigen::MatrixBase<MT> &rpy)
{
  static_assert(MT::ColsAtCompileTime == 1 && MT::RowsAtCompileTime == 3,
                "must have 3x1 vector");
  return genericRotMat(rpy[0], rpy[1], rpy[2]);
}

template<typename MT>
Quat<typename MT::Scalar> Math::rotMat2Quat(const Eigen::MatrixBase<MT> &R)
{
  static_assert(MT::ColsAtCompileTime == 3 && MT::RowsAtCompileTime == 3,
                "Must have 3x3 matrix");

  Quat<typename MT::Scalar> q;
  typename MT::Scalar tr = R.trace();

  if(tr > 0.0){
    typename MT::Scalar S = sqrt(tr + 1.0) * 2.0;
    q(0) = 0.25 * S;
    q(1) = (R(2, 1) - R(1, 2)) / S;
    q(2) = (R(0, 2) - R(2, 0)) / S;
    q(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) && (R(0, 0) > R(2, 2))) {
    typename MT::Scalar S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;
    q(0) = (R(2, 1) - R(1, 2)) / S;
    q(1) = 0.25 * S;
    q(2) = (R(0, 1) + R(1, 0)) / S;
    q(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
    typename MT::Scalar S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;
    q(0) = (R(0, 2) - R(2, 0)) / S;
    q(1) = (R(0, 1) + R(1, 0)) / S;
    q(2) = 0.25 * S;
    q(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
    typename MT::Scalar S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;
    q(0) = (R(1, 0) - R(0, 1)) / S;
    q(1) = (R(0, 2) + R(2, 0)) / S;
    q(2) = (R(1, 2) + R(2, 1)) / S;
    q(3) = 0.25 * S;
  }
  return q;
}

template<typename T>
inline Mat3<T> Math::genericRotMat(T xTheta, T yTheta, T zTheta)
{
  return basicRotMat(Axis::Z, zTheta) * basicRotMat(Axis::Y, yTheta) * basicRotMat(Axis::X, xTheta);
}

// TODO: discuss this
template<typename T>
Mat3<T> Math::basicRotMat(Axis axis, T theta)
{
  static_assert(std::is_floating_point<T>::value,
                  "must use floating point value");

  T s = std::sin(theta);
  T c = std::cos(theta);

  Mat3<T> R;
  switch(axis){
  case Axis::X:
    R << 1.0f, 0.0f,  0.0f,
         0.0f, c   ,  -s,
         0.0f, s   ,  c;
    break;
  case Axis::Y:
    R << c   , 0.0f, s,
         0.0f, 1.0f, 0.0f,
         -s  , 0.0f, c;
    break;
  case Axis::Z:
    R << c   ,  -s  , 0.0f,
         s   ,  c   ,  0.0f,
         0.0f,  0.0f, 1.0f;
  }
  return R;
}

#endif // QR_SE3_H

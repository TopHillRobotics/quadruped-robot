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

#ifndef QR_SE3_H
#define QR_SE3_H

#include <cmath>
#include <iostream>
#include "common/qr_algebra.h"
#include "common/qr_eigen_types.h"

#include <Eigen/Dense>
namespace math {

  /**
   * @brief The Axis enum specifies axis in coordinate. 0: X axis; 1: Y axis; 2: Z Axis
   */
  enum Axis{X, Y, Z};

  /**
   * @brief convert roll ptch yaw to rotation matrix
   * @param rpy: reference to vector of roll ptch yaw
   */
  template<typename MT>
  Mat3<typename MT::Scalar> Rpy2RotMat(const Eigen::MatrixBase<MT> &rpy);

  /**
   * @brief convert raw pitch yaw to quaterion
   * @param rpy: reference to vector of roll ptch yaw
   * @return quaterion
   */
  template<typename MT>
  Quat<typename MT::Scalar> Rpy2Quat(const Eigen::MatrixBase<MT> &rpy);

  /**
   * @brief convert rotation matrix to quaterion
   * @param R: reference to rotation matrix
   * @return quaterion
   */
  template<typename MT>
  Quat<typename MT::Scalar> RotMat2Quat(const Eigen::MatrixBase<MT> &R);

  /**
   * @param axis: the rotation axis
   * @param theta: the rotation angle
   * @return basic rotation matrix
   */
  template<typename T>
  Mat3<T> BasicRotMat(Axis axis, T theta);

  /**
   * @brief the rotation order is X axis, Y axis and Z axis, all fixed coordinate
   * @param xTheta: rotation around x axis
   * @param yTheta: rotation around y axis
   * @param zTheta: rotation around z axis
   */
  template<typename T>
  Mat3<T> GenericRotMat(T xTheta, T yTheta, T zTheta);

  /**
   * @brief Put the skew-symmetric component of 3x3 matrix m into a 3x1 vector
   */
  template<typename T>
  Vec3<typename T::Scalar> Mat2SkewVec(const Eigen::MatrixBase<T> &m);

  /**
   * @brief convert quaternion to rotation matrix
   * @param q: quaternion
   */
  template<typename T>
  Mat3<typename T::Scalar> Quat2RotMat(const Eigen::MatrixBase<T> &q);

  /**
   * @brief Tranform points coordination represented in base frame to world frame.
   * @param Vec3<T> the 3d position of the origin of the base frame w.r.t. world frame;
   * @param Quat<T> the orientation of the base frame (quaternion, wxyz) w.r.t. worlf frame;
   * @param Eigen::Matrix<T, 3, N> The input point(s) is represent in the body frame
   * @return Eigen::Matrix<T, 3, N> The output point(s) is represent in the world frame, but these two point are the same point in physics.
   */
  template<typename T, int N = 1>
  Eigen::Matrix<T, 3, N> InvertRigidTransform(Vec3<T> t, Quat<T> R, Eigen::Matrix<T, 3, N> points);

  /**
   * @brief Tranform points coordination represented in world frame to base frame.
   * @param Vec3<T> the 3d position of the origin of the base frame w.r.t. world frame;
   * @param Quat<T> the orientation of the base frame (quaternion, wxyz) w.r.t. worlf frame;
   * @param Eigen::Matrix<T, 3, N> The input point(s) is represent in the world frame
   * @return Eigen::Matrix<T, 3, N> The output point(s) is represent in the base frame, but these two point are the same point in physics.
   */
  template<typename T, int N = 1>
  Eigen::Matrix<T, 3, N> RigidTransform(Vec3<T> t, Quat<T> R, Eigen::Matrix<T, 3, N> points);

  /**
   * @brief Transform Vector r in B frame to A frame By Quaternion conventions
   * @param quat Quat<float> wxyz;
   * @param r_b Vec3<float>, the vector/position presented in B frame; ;
   */
  template<typename T>
  Vec3<T> TransformVecByQuat(Quat<T> quat, Vec3<T> r_b);

  /**
   * @brief calculate the inverse of quaternion
   * @param q: quaternion to calculate the inverse
   */
  template<typename T>
  Quat<typename T::Scalar> QuatInverse(const Eigen::MatrixBase<T> &q);

  /**
   * @brief convert rotation matrix to row, pitch, yaw
   * @param R: rotation matrix
   */
  template<typename T>
  Vec3<typename T::Scalar> RotMat2Rpy(const Eigen::MatrixBase<T> &R);

  /**
   * @brief convert quaternion to row, pitch, yaw
   * @param q: quaternion
   */
  template<typename T>
  Vec3<typename T::Scalar> Quat2Rpy(const Eigen::MatrixBase<T> &q);

  /**
   * @brief convert vector to skew matrix
   * @param v: vector
   */
  template<typename T>
  Mat3<typename T::Scalar> Vector2SkewMat(const Eigen::MatrixBase<T> &v);

  /**
   * @brief convert skew matrix to skew vector
   * @param m: skew matrix
   */
  template<typename T>
  Vec3<typename T::Scalar> Mat2SkewVec(const Eigen::MatrixBase<T> &m);
}

template<typename MT>
Quat<typename MT::Scalar> math::Rpy2Quat(const Eigen::MatrixBase<MT> &rpy)
{
  Mat3<typename MT::Scalar> R = Rpy2RotMat(rpy);
  Quat<typename MT::Scalar> q = RotMat2Quat(R);
  return q;
}

template<typename MT>
Mat3<typename MT::Scalar> math::Rpy2RotMat(const Eigen::MatrixBase<MT> &rpy)
{
  static_assert(MT::ColsAtCompileTime == 1 && MT::RowsAtCompileTime == 3,
                "must have 3x1 vector");
  return GenericRotMat(rpy[0], rpy[1], rpy[2]);
}

template<typename MT>
Quat<typename MT::Scalar> math::RotMat2Quat(const Eigen::MatrixBase<MT> &R)
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
inline Mat3<T> math::GenericRotMat(T xTheta, T yTheta, T zTheta)
{
  return BasicRotMat(Axis::Z, zTheta) * BasicRotMat(Axis::Y, yTheta) * BasicRotMat(Axis::X, xTheta);
}

// TODO: discuss this
template<typename T>
Mat3<T> math::BasicRotMat(Axis axis, T theta)
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

template<typename T>
Vec3<typename T::Scalar> math::Mat2SkewVec(const Eigen::MatrixBase<T> &m){
  static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                            "Must have 3x3 matrix");
  return 0.5 * Vec3<typename T::Scalar>(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0), (m(1, 0) - m(0, 1)));
}

template<typename T>
Mat3<typename T::Scalar> math::Quat2RotMat(
    const Eigen::MatrixBase<T> &q)
{
    static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                  "Must have 4x1 quat");
    typename T::Scalar e0 = q(0);
    typename T::Scalar e1 = q(1);
    typename T::Scalar e2 = q(2);
    typename T::Scalar e3 = q(3);

    Mat3<typename T::Scalar> R;

    R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
        2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
        1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
        2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
        1 - 2 * (e1 * e1 + e2 * e2);
    return R;
}

template<typename T, int N>
Eigen::Matrix<T, 3, N> math::InvertRigidTransform(Vec3<T> t, Quat<T> R, Eigen::Matrix<T, 3, N> points)
{
    // Eigen::Isometry3f transform; or Eigen::Transform<T,3,Eigen::Isometry>
    Isometry3<T> transform = Isometry3<T>::Identity();
    transform.translate(t);
    Eigen::Quaternion<T> r{R(0), R(1), R(2), R(3)};
    transform.rotate(r);
    return transform * points;
}

template<typename T, int N>
Eigen::Matrix<T, 3, N> math::RigidTransform(Vec3<T> t, Quat<T> R, Eigen::Matrix<T, 3, N> points)
{
    Isometry3<T> transform = Isometry3<T>::Identity();

    Eigen::Quaternion<T> r{R(0), R(1), R(2), R(3)};
    transform.rotate(r.inverse());
    transform.translate(-t);
    return transform * points;
}

template<typename T>
Vec3<T> math::TransformVecByQuat(Quat<T> quat, Vec3<T> r_b)
{
    T q0 = quat[0];
    Vec3<T> q_ = quat.tail(3);
    Vec3<T> r_a = (2*q0*q0-1)*r_b + 2*q0*Vector2SkewMat(q_)*r_b + 2*q_*q_.dot(r_b);
    return r_a;
}

template<typename T>
Quat<typename T::Scalar> math::QuatInverse(const Eigen::MatrixBase<T> &q)
{
    Quat<typename T::Scalar> qInverse;
    qInverse << q[0], -q.tail(3);
    qInverse /= q.dot(q);
    return qInverse;
}

template<typename T>
Vec3<typename T::Scalar> math::RotMat2Rpy(const Eigen::MatrixBase<T> &R)
{
    static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                  "Must have 3x3 matrix");
    Quat<typename T::Scalar> q = RotMat2Quat(R);
    Vec3<typename T::Scalar> rpy = Quat2Rpy(q);
    return rpy;
}


template<typename T>
Vec3<typename T::Scalar> math::Quat2Rpy(const Eigen::MatrixBase<T> &q)
{
    static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                  "Must have 4x1 quat");
    Vec3<typename T::Scalar> rpy;
    typename T::Scalar as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
    rpy(2) =
        std::atan2(2 * (q[1] * q[2] + q[0] * q[3]),
                   square(q[0]) + square(q[1]) - square(q[2]) - square(q[3]));
    rpy(1) = std::asin(as);
    rpy(0) =
        std::atan2(2 * (q[2] * q[3] + q[0] * q[1]),
                   square(q[0]) - square(q[1]) - square(q[2]) + square(q[3]));
    return rpy;
}

template<typename T>
Mat3<typename T::Scalar> math::Vector2SkewMat(const Eigen::MatrixBase<T> &v)
{
    static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                  "Must have 3x1 matrix");
    Mat3<typename T::Scalar> m;
    m << 0, -v[2], v[1],
        v[2], 0, -v[0],
        -v[1], v[0], 0;
    return m;
}

template<typename T>
Vec3<typename T::Scalar> Mat2SkewVec(const Eigen::MatrixBase<T> &m)
{
    static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                  "Must have 3x3 matrix");
    return 0.5 * Vec3<typename T::Scalar>(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0), (m(1, 0) - m(0, 1)));
}

#endif // QR_SE3_H

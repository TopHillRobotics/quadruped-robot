#ifndef QR_EIGENTYPES_H
#define QR_EIGENTYPES_H

#include <Eigen/Dense>

  // TODO: fill this file
  /**
   * @brief a 3x3 matrix with type T
   */
  template<typename T>
  using Mat3 = typename Eigen::Matrix<T, 3, 3>;

  /**
   * @brief a 4x1 vector with type T
   */
  template<typename T>
  using Quat = typename Eigen::Matrix<T, 4, 1>;

#endif // QR_EIGENTYPES_H

#ifndef QR_EIGEN_TYPES_H
#define QR_EIGEN_TYPES_H

#include <Eigen/Dense>

/**
 * @brief a 4x1 vector with type T
 */
template<typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

template<typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

/**
 * @brief a 3x1 vector with type T
 */
template<typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

/**
 * @brief a 4x1 vector with type T
 */
template<typename T>
using Vec4 = typename Eigen::Matrix<T, 4, 1>;

/**
 * @brief a 6x1 vector with type T
 */
template<typename T>
using Vec6 = typename Eigen::Matrix<T, 6, 1>;

/**
 * @brief a 12x1 vector with type T
 */
template<typename T>
using Vec12 = typename Eigen::Matrix<T, 12, 1>;

/**
 * @brief a 24x1 vector with type T
 */
template<typename T>
using Vec24 = typename Eigen::Matrix<T, 24, 1>;

/**
 * @brief a 3x3 vector with type T
 */
template<typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

/**
 * @brief a 4x4 vector with type T
 */
template<typename T>
using Mat4 = typename Eigen::Matrix<T, 4, 4>;

/**
 * @brief a 6x6 vector with type T
 */
template<typename T>
using Mat6 = typename Eigen::Matrix<T, 6, 6>;

/**
 * @brief a 12x12 vector with type T
 */
template<typename T>
using Mat12 = typename Eigen::Matrix<T, 12, 12>;

/**
 * @brief a 3x4 matrix with type T
 */
template<typename T>
using Mat3x4 = Eigen::Matrix<T, 3, 4>;

/**
 * @brief a dynamic size matrix with type T
 */
template<typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

/**
 * @brief an homogeneous transformation in a 3-D space
 */
template<typename T>
using Isometry3 = typename Eigen::Transform<T, 3, Eigen::Isometry>;

#endif // QR_EIGEN_TYPES_H

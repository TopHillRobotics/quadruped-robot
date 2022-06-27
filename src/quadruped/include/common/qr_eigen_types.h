#ifndef QR_EIGEN_TYPES_H
#define QR_EIGEN_TYPES_H

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
 * @brief a dynamic size matrix with type T
 */
template<typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

/**
 * @brief a 4x4 vector with type T
 */
template<typename T>
using Mat4 = typename Eigen::Matrix<T, 4, 4>;

#endif // QR_EIGEN_TYPES_H

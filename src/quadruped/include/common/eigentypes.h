#ifndef EIGENTYPES_H
#define EIGENTYPES_H

#include <Eigen/Dense>

// TODO: fill this file
/**
 * @brief a 3x3 matrix with type T
 */
template<typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

#endif // EIGENTYPES_H

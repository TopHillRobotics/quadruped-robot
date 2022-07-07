#ifndef QR_MATH_H
#define QR_MATH_H

#include <vector>
#include <Eigen/Dense>

/**
 * @brief calculate the average of vectors
 */
template<typename T>
Eigen::Matrix<T, 3, 1> mean(std::vector<Eigen::Matrix<T, 3, 1>> &vs){
  Eigen::Matrix<T, 3, 1> m =Eigen::Matrix<T, 3, 1>::Zero();
  for (auto &v: vs) {
      m += v;
  }
  m /= vs.size();
  return m;
}

#endif // QR_MATH_H

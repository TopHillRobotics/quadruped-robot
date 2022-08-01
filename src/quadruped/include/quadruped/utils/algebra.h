/*! @file MathUtilities.h
 *  @brief Utility functions for math
 *
 */
#ifndef PROJECT_MATH_ALGEBRA_H
#define PROJECT_MATH_ALGEBRA_H

#include <Eigen/Dense>

namespace robotics {
    namespace math {

        /** @brief Square a number */
        template<typename T>
        T square(T a)
        {
            return a * a;
        }

        /** @brief Are two eigen matrices almost equal? */
        template<typename T, typename T2>
        bool almostEqual(const Eigen::MatrixBase<T> &a, const Eigen::MatrixBase<T> &b, T2 tol)
        {
            long x = T::RowsAtCompileTime;
            long y = T::ColsAtCompileTime;

            if (T::RowsAtCompileTime == Eigen::Dynamic ||
                T::ColsAtCompileTime == Eigen::Dynamic) {
                assert(a.rows() == b.rows());
                assert(a.cols() == b.cols());
                x = a.rows();
                y = a.cols();
            }

            for (long i = 0; i < x; i++) {
                for (long j = 0; j < y; j++) {
                    T2 error = std::abs(a(i, j) - b(i, j));
                    if (error >= tol) return false;
                }
            }
            return true;
        }

        /** @brief Are two float type number almost equal? */
        template<typename T>
        bool almostEqual(const T &a, const T b, T tol)
        {
            T error = std::abs(a - b);
            if (error >= tol) {
                return false;
            }
            return true;
        }

    } //  namespace math
} // namespace robotics

#endif  // PROJECT_MATH_ALGEBRA_H

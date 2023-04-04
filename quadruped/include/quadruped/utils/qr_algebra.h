// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

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

#ifndef QR_ALGEBRA_H
#define QR_ALGEBRA_H

#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>

#include "qr_cpptypes.h"


namespace robotics {

namespace math {

template<typename T>
T clip_min(T command, const T& minVal) {
    if (command < minVal) {
        return minVal;
    } else {
        return command;
    }
}

template<typename T>
T clip_max(T command, const T& maxVal) {
    if (command > maxVal) {
        return maxVal;
    } else {
        return command;
    }
}

template<typename T>
T clip(T command, const T& minVal, const T& maxVal) {
    if (command < minVal) {
        return minVal;
    } else if (command > maxVal) {
        return maxVal;
    } else {
        return command;
    }
}

/**
 * @brief Square a number.
 */
template<typename T>
T square(T a) {
    return a * a;
}

/**
 * @brief Are two eigen matrices almost equal.
 */
template<typename T, typename T2>
bool almostEqual(const Eigen::MatrixBase<T> &a, const Eigen::MatrixBase<T> &b, T2 tol) {
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

/**
 * @brief Are two float type number almost equal.
 */
template<typename T>
bool almostEqual(const T &a, const T b, T tol) {
    T error = std::abs(a - b);
    if (error >= tol) {
        return false;
    }
    return true;
}

/**
 * @brief Compute the pseudo inverse of a matrix.
 * @param matrix: input matrix.
 * @param sigmaThreshold: threshold for singular values being zero.
 * @param invMatrix: output matrix.
 */
template <typename T>
void pseudoInverse(const DMat<T>& matrix, double sigmaThreshold, DMat<T>& invMatrix) {
    if (matrix.rows()==1 && matrix.cols()==1) {
        invMatrix.resize(1, 1);
        if (matrix.coeff(0, 0) > sigmaThreshold) {
            invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
        } else {
            invMatrix.coeffRef(0, 0) = 0.0;
        }
        return;
    }

    Eigen::JacobiSVD<DMat<T>> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    /* not sure if we need to svd.sort()... probably not. */
    const int nrows(svd.singularValues().rows());
    DMat<T> invS = DMat<T>::Zero(nrows, nrows);
    for (int ii(0); ii < nrows; ++ii) {
        if (svd.singularValues().coeff(ii) > sigmaThreshold) {
            invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
        }
    }
    invMatrix.noalias() = svd.matrixV() * invS * svd.matrixU().transpose();
}

} // Namespace math

} // Namespace robotics

template<typename T>
using DenseMatrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

#endif  // QR_ALGEBRA_H

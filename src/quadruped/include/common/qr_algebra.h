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

#ifndef QR_ALGEBRA_H
#define QR_ALGEBRA_H

#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>

#include "qr_sparse_matrix.hpp"

namespace Math {

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

    /*!
    * Compute the pseudo inverse of a matrix
    * @param matrix : input matrix
    * @param sigmaThreshold : threshold for singular values being zero
    * @param invMatrix : output matrix
    */
    template <typename T>
    void pseudoInverse(DMat<T> const& matrix, double sigmaThreshold,
                    DMat<T>& invMatrix)
    {
        if ((1 == matrix.rows()) && (1 == matrix.cols())) {
            invMatrix.resize(1, 1);
            if (matrix.coeff(0, 0) > sigmaThreshold) {
                invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
            } else {
                invMatrix.coeffRef(0, 0) = 0.0;
            }
            return;
        }

        Eigen::JacobiSVD<DMat<T>> svd(matrix,
                                        Eigen::ComputeThinU | Eigen::ComputeThinV);
        // not sure if we need to svd.sort()... probably not
        int const nrows(svd.singularValues().rows());
        DMat<T> invS;
        invS = DMat<T>::Zero(nrows, nrows);
        for (int ii(0); ii < nrows; ++ii) {
            if (svd.singularValues().coeff(ii) > sigmaThreshold) {
                invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
            } else {
                // invS.coeffRef(ii, ii) = 1.0/ sigmaThreshold;
                // printf("sigular value is too small: %f\n",
                // svd.singularValues().coeff(ii));
            }
        }
        invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();
    }
} //  namespace math

template<typename T>
using DenseMatrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;


template<typename T>
class CholeskyDenseSolver
{
public:
  CholeskyDenseSolver(bool print) : _print(print) { }
  ~CholeskyDenseSolver() {
    delete[] pivots;
    delete[] solve1;
    delete[] solve2;
  }
  void setup(const DenseMatrix<T>& kktMat);
  void solve(Eigen::Matrix<T, Eigen::Dynamic, 1>& in);
  void set_print(bool print) {
    _print = print;
  }

  DenseMatrix<T>& getInternal() { return L; }
  DenseMatrix<T> getReconstructedPermuted();

private:
  DenseMatrix<T> L;
  void solveAVX(Eigen::Matrix<T, Eigen::Dynamic, 1>& in);
  void setupAVX(T* mat, T* result, T* vec, u64 row);
  T* solve1 = nullptr, *solve2 = nullptr;
  Eigen::Matrix<T, Eigen::Dynamic, 1> D;
  s64* pivots = nullptr;
  s64 n = 0;
  bool _print;
};

template<typename T>
class CholeskySparseSolver
{
public:
  CholeskySparseSolver() = default;
  void preSetup(const DenseMatrix<T>& kktMat, bool b_print = true);
  void preSetup(const std::vector<SparseTriple<T>>& kktMat, u32 n, bool b_print = true);
  void setup(bool b_print = true);
  void solve(Eigen::Matrix<T, Eigen::Dynamic, 1>& out);
  void amdOrder(MatCSC<T>& mat, u32* perm, u32* iperm);

  ~CholeskySparseSolver() {
    A.freeAll();
    L.freeAll();
    delete[] reverseOrder;
    delete[] nnzLCol;
    delete[] P;
    delete[] D;
    delete[] rP;
    delete[] rD;
    delete[] parent;
    delete[] tempSolve;
  }
private:
  void reorder();
  u32 symbolicFactor();
  void factor();
  void sanityCheck();
  void solveOrder();
  SparseTriple<T> A_triple;
  MatCSC<T> A;
  MatCSC<T> L;
  u32 n;
  T* tempSolve = nullptr;
  T* reverseOrder = nullptr;
  u32* nnzLCol = nullptr; // # of nonzeros per column in L
  u32* P = nullptr;       // reorder permutation
  u32* rP = nullptr;      // reorder inverse permutation
  T*   D = nullptr;       // Diagonal
  T*   rD = nullptr;      // inverse diagonal
  s32* parent = nullptr;  // tree
};

#endif  // QR_ALGEBRA_H

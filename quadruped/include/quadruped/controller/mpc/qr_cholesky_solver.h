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

#ifndef QR_CHOLESKY_SOLVER_H
#define QR_CHOLESKY_SOLVER_H

#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
#include "controller/mpc/qr_sparse_matrix.h"

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

#include "qr_cholesky_solver_impl.h"

#endif // QR_CHOLESKY_SOLVER_H

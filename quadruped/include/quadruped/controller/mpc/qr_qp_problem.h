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

#ifndef QR_QP_PROBLEM_H
#define QR_QP_PROBLEM_H

#include <vector>
#include <eigen3/Eigen/SparseCholesky>
#include <eigen3/Eigen/Sparse>

#include "common/qr_cTypes.h"
#include "common/qr_algebra.h"
#include "qr_cholesky_solver.h"

// 0.5 * x'Px + q'x
// l <= Ax <= u

template<typename T>
struct QpProblemSettings {
  s64 maxIterations = 1000000;
  T rho = 2;
  T sigma = 1e-5;
  T alpha = 1.6;

  // numerical hacks
  T infty = 1e10;
  T eqlTol = 1e-10;
  T rhoEqualityScale = 1e3;
  T rhoInfty = 1e-6;

  T terminate = 1e-3;

  void print()
  {
    printf("rho: %f\n"
           "sigma: %f\n"
           "alpha: %f\n"
           "terminate: %f\n"
           "max_iter: %ld\n" , rho, sigma, alpha, terminate, maxIterations);

  }
};

enum class ConstraintType {
  INFINITE,
  INEQUALITY,
  EQUALITY
};

template<typename T>
struct ConstraintInfo {
  T rho;
  T invRho;
  ConstraintType type;
};

template<typename T>
class QpProblem
{
public:

    QpProblem(s64 n_, s64 m_, bool print_timings = true)
     : A(m_,n_), P(n_,n_),
    l(m_), u(m_), q(n_),
    n(n_), m(m_),
    _print(print_timings),
    _kkt(n_ + m_, n_ + m_),
    _cholDenseSolver(print_timings),
    _xzTilde(n_ + m_), _y(m_),
    _x0(n_), _x1(n_), _z0(m_), _z1(m_),
    _Ar(m_), _Pr(n_), _AtR(n_),
    _deltaY(m_), _AtDeltaY(n_),
    _deltaX(n_), _PDeltaX(n_), _ADeltaX(m_)
    {
        q.setZero();
        _constraintInfos.resize(m);
      (void)(n_);
      (void)(m_);
    }

    void runFromDense(s64 nIterations = -1, bool sparse = false, bool b_print = true);
    void runFromTriples(s64 nIterations = -1, bool b_print = true);

    Eigen::Matrix<T, Eigen::Dynamic, 1>& getSolution() { return *_x; }



  // public data
  QpProblemSettings<T> settings;
  DenseMatrix<T> A, P;
  std::vector<SparseTriple<T>> A_triples, P_triples;
  Eigen::Matrix<T, Eigen::Dynamic, 1> l, u, q;
  s64 n, m;

  ~QpProblem() {
    //printf("done!\n");
  }

private:
  void coldStart();
  void computeConstraintInfos();
  void setupLinearSolverCommon();
  void stepSetup();
  void solveLinearSystem();
  void stepX();
  void stepZ();
  void stepY();
  void setupTriples();
  T calcAndDisplayResidual(bool print);
  T infNorm(const Eigen::Matrix<T, Eigen::Dynamic, 1>& v);

  bool _print;
  DenseMatrix<T> _kkt;
  std::vector<SparseTriple<T>> _kktTriples;

  CholeskyDenseSolver<T> _cholDenseSolver;
  CholeskySparseSolver<T> _cholSparseSolver;
  Eigen::SparseMatrix<T> Asparse, Psparse;

  Eigen::Matrix<T, Eigen::Dynamic, 1> _xzTilde, _y;
  Eigen::Matrix<T, Eigen::Dynamic, 1> _x0, _x1, _z0, _z1;
  Eigen::Matrix<T, Eigen::Dynamic, 1> *_x, *_z, *_xPrev, *_zPrev;
  Eigen::Matrix<T, Eigen::Dynamic, 1> _Ar, _Pr, _AtR; // residuals
  Eigen::Matrix<T, Eigen::Dynamic, 1> _deltaY, _AtDeltaY, _deltaX, _PDeltaX, _ADeltaX; // infeasibilities
  std::vector<ConstraintInfo<T>> _constraintInfos;

  bool _hotStarted = false, _sparse = false;

};

#include "qr_qp_problem_impl.h"

#endif // QR_QP_PROBLEM_H

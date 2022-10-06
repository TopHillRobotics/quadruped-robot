// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:  tophill.robotics@gmail.com

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

#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include "controller/mpc/qr_sparse_cmpc.h"
#include "osqp/osqp.h"

// X0 != x[0]
// X0 uses u[0] and dt[0] to get to x[0]
// so same number of steps in u, dt, x

// 0 - roll
// 1 - pitch
// 2 - yaw
// 3 - x_pos
// 4 - y_pos
// 5 - z_pos
// 6 - roll_rate
// 7 - pitch_rate
// 8 - yaw_rate
// 9 - x_vel
// 10- y_vel
// 11- z_vel

struct OsqpCSC {
  u32 nnz, m, n;
  c_float* values = nullptr;
  c_int* colPtrs = nullptr;
  c_int* rowIdx = nullptr;

  void alloc(u32 _n, u32 _nnz) {
    nnz = _nnz;
    n = _n;
    m = _n;
    values = new c_float[nnz];
    colPtrs = new c_int[n + 1];
    rowIdx = new c_int[nnz];
  }

  void freeAll() {
    delete[] values;
    delete[] colPtrs;
    delete[] rowIdx;
  }
};

static OsqpCSC compress(std::vector<SparseTriple<double>>& entries, u32 m, u32 n) {
  OsqpCSC result;
  u32 nnz = entries.size();
  result.alloc(n, nnz);
  result.m = m;

  u32 i = 0;
  result.colPtrs[0] = 0;
  for(u32 c = 0; c < n; c++) {
    u32 cNNZ = 0;
    while(entries[i].c == c && i < nnz) {
      result.values[i] = entries[i].value;
      result.rowIdx[i] = entries[i].r;
      assert(entries[i].r < m);
      assert(entries[i].c < n);
      assert(i < result.nnz);
      i++;
      cNNZ++;
    }
    result.colPtrs[c + 1] = result.colPtrs[c] + cNNZ;
    assert(c+1 < result.n + 1);
    assert(i == nnz || entries[i].c > c);
  }

  assert(i == nnz);
  return result;
}

SparseCMPC::SparseCMPC() {

}

void SparseCMPC::run() {
  // Timer timer;
  // check trajectory length
  if((_stateTrajectory.size() != _contactTrajectory.size()) || (_contactTrajectory.size() != _dtTrajectory.size())) {
    throw std::runtime_error("SparseCMPC trajectory length error!");
  }

  // reset
  _g.setZero();
  _g[11] = -9.81;
  _constraintCount = 0;
  _trajectoryLength = _stateTrajectory.size();
  _constraintTriples.clear();
  _costTriples.clear();
  _ub.clear();
  _lb.clear();
  _contactCounts.clear();
  _runningContactCounts.clear();
  _bBlockCount = 0;

  // build initial state and data
  buildX0();
  buildCT();
  buildDT();
  //printf("t1: %.3f\n", timer.getMs());
  // timer.start();

  // build optimization problem
  addX0Constraint();
  addDynamicsConstraints();
  addForceConstraints();
  addFrictionConstraints();
  addQuadraticStateCost();
  addLinearStateCost();
  addQuadraticControlCost();
  //printf("t2: %.3f\n", timer.getMs());

  // Solve!
  //runSolver();
  runSolverOSQP();
}

/*!
 * Configure initial state of robot
 * _rpy0, _x0
 */
void SparseCMPC::buildX0() {
  _rpy0 = math::quatToRPY(_q0);
  _x0 << _rpy0, _p0, _w0, _v0;
}

/*!
 * Build continuous time matrices.
 * uses trajectory yaw for yaw integration, not trajectory omega.
 */
void SparseCMPC::buildCT() {
//  for(u32 foot = 0; foot < 4; foot++) {
//    Vec3<double> pFoot = _pFeet.block(foot*3,0,3,1);
//    printf("FOOT %d: %6.3f, %6.3f %6.3f\n", foot, pFoot[0], pFoot[1], pFoot[2]);
//  }
  // one 12x12 A matrix per timestep
  _aMat.clear();
  _aMat.resize(_trajectoryLength);

  // B "blocks" are for a single foot for a single iteration (so 12x3)
  _bBlockIds.clear();
  _bBlocks.clear();

  // loop over trajectory
  for(u32 i = 0; i < _trajectoryLength; i++) {
    // rotation from world into yaw frame
    //Mat3<double> Ryaw = math::coordinateRotation(math::CoordinateAxis::Z, _stateTrajectory[i][2]); // todo check
    Mat3<double> Ryaw = math::coordinateRotation(math::CoordinateAxis::Z, _rpy0[2]);

    // transform inertia to world and invert
    Mat3<double> Iworld = Ryaw.transpose() * _Ibody * Ryaw;
    Mat3<double> Iinv = Iworld.inverse();

    // build A matrix (continuous time)
    Mat12<double>& A = _aMat[i];
    A.setZero();
    A(3,9) = 1; // x position integration
    A(4,10) = 1; // y position integration
    A(5,11) = 1; // z position integration
    A.block(0,6,3,3) = Ryaw; // omega integration

    // build up to four B blocks for the four feet
    auto& contactState = _contactTrajectory[i];
    for(uint32_t foot = 0; foot < 4; foot++) {
      if(contactState[foot]) { // if foot is touching ground, add it.
        _bBlocks.emplace_back();
        _bBlockIds.push_back({foot, i});
        auto& B = _bBlocks.back();
        Vec3<double> pFoot = _pFeet.block(foot*3,0,3,1);

        B.setZero();
        B.block(6,0,3,3) = Iinv * math::crossMatrix(pFoot);    // r x f torque
        B.block(9,0,3,3) = Mat3<double>::Identity() / _mass;  // f = ma
      }
    }
  }
  _bBlockCount = _bBlockIds.size();
}

/*!
 * Build discrete time matrices
 */
void SparseCMPC::buildDT() {
  u32 runningContactCount = 0;
  for(u32 i = 0; i < _trajectoryLength; i++) {
    u32 contactCount = _contactTrajectory[i].cast<int>().sum();
    _contactCounts.push_back(contactCount);
    _runningContactCounts.push_back(runningContactCount);
    c2d(i, runningContactCount, contactCount);
    runningContactCount += contactCount;
  }
}


// for now, all xs occur before all us
u32 SparseCMPC::getStateIndex(u32 trajIdx) {
  assert(trajIdx < _trajectoryLength);
  return trajIdx * 12;
}

u32 SparseCMPC::getControlIndex(u32 bBlockIdx) {
  assert(bBlockIdx < _bBlockCount);
  return (_trajectoryLength * 12) + (bBlockIdx * 3);
}

u32 SparseCMPC::addConstraint(u32 size) {
  u32 rv = _constraintCount;
  _constraintCount += size;
  return rv;
}

void SparseCMPC::addConstraintTriple(double value, u32 row, u32 col) {
  assert(col < 12 * _trajectoryLength + 3 * _bBlockCount);
  assert(row < _constraintCount);
  if(value != 0) {
    _constraintTriples.push_back({value, row, col});
  }
}


void SparseCMPC::addX0Constraint() {
  // x[0] = A[0] * X0 + B[0] * u[0] + g*dt;
  // x[0] - (B[0] * u[0]) = A[0]*X0 + g*dt;

  // select x[0] with identity matrix
  u32 state_idx = getStateIndex(0);
  u32 constraint_idx = addConstraint(12);
  for(u32 i = 0; i < 12; i++) { // diagonal of the identity
    _constraintTriples.push_back({1,constraint_idx + i, state_idx + i});
  }

  if(_runningContactCounts[0]) throw std::runtime_error("contact count error!");

  u32 ctrl_cnt = _contactCounts[0];

  for(u32 i = 0; i < ctrl_cnt; i++) { // Bblocks within this b (contact feet)
    // select -B[0]*u[0]
    u32 bbIdx = _runningContactCounts[0] + i;
    for(u32 ax = 0; ax < 3; ax++) { // columns within the b block (forces axes)
      for(u32 row = 0; row < 12; row++) { // rows within the b block
        addConstraintTriple(-_bBlocks[bbIdx](row, ax), constraint_idx + row, getControlIndex(bbIdx) + ax);
      }
    }
  }

  // compute right hand side A[0]*X0 + g*dt.
  Vec12<double> rhs = _aMat[0] * _x0 + _g * _dtTrajectory[0];

//  printf("x0: \n");
//  std::cout << _x0.transpose() << "\n";

  // add to problem.
  for(u32 i = 0; i < 12; i++) {
    _ub.push_back(rhs[i]);
    _lb.push_back(rhs[i]);
  }

}

void SparseCMPC::addDynamicsConstraints() {

  // x[n] = A[n] * x[n-1] + B[n] * u[n] + g * dt[n]
  for(u32 i = 1; i < _trajectoryLength; i++) {
    u32 next_state_idx = getStateIndex(i);
    u32 prev_state_idx = getStateIndex(i - 1);
    u32 constraint_idx = addConstraint(12);

    // get I * x[n]
    for(u32 j = 0; j < 12; j++) {
      _constraintTriples.push_back({1, constraint_idx + j, next_state_idx + j});
    }

    // get -A[n] * x[n-1]
    for(u32 r = 0; r < 12; r++) {
      for(u32 c = 0; c < 12; c++) {
        addConstraintTriple(-_aMat[i](r,c), constraint_idx + r, prev_state_idx + c);
      }
    }

    // get -B[n] * u[n]
    u32 contact_count = _contactCounts[i];
    u32 bb_idx = _runningContactCounts[i];
    for(u32 contact = 0; contact < contact_count; contact++) {
      for(u32 row = 0; row < 12; row++) {
        for(u32 col = 0; col < 3; col++) {
          addConstraintTriple(-_bBlocks[bb_idx + contact](row, col),
            constraint_idx + row,
            getControlIndex(bb_idx + contact) + col);
        }
      }
    }

    // rhs g*dt
    Vec12<double> rhs = _g * _dtTrajectory[i];
    for(u32 j = 0; j < 12; j++) {
      _ub.push_back(rhs[j]);
      _lb.push_back(rhs[j]);
    }

//    u32 idx2 = addConstraint(1);
//    addConstraintTriple(1., idx2, getStateIndex(i) + 5);
//    _lb.push_back(0);
//    _ub.push_back(0.3);

  }
}

void SparseCMPC::addForceConstraints() {
  // constrain all Z forces:
  for(u32 i = 0; i < _bBlockCount; i++) {
    _ub.push_back(_maxForce);
    _lb.push_back(0);
    addConstraintTriple(1, // directly get force
      addConstraint(1),    // new constraint
      getControlIndex(i) + 2 // ith contact's z force
      );
  }
}

void SparseCMPC::addFrictionConstraints() {
  double muInv = 1. / _mu;
  for(u32 i = 0; i < _bBlockCount; i++) {
    // four friction constraints
    u32 constraint_idx = addConstraint(4);
    u32 control_idx    = getControlIndex(i);
    for(u32 c = 0; c < 4; c++) {
      _lb.push_back(0);
      _ub.push_back(1e15); // inf.
    }

    // x/mu + z > 0
    addConstraintTriple(muInv, constraint_idx + 0, control_idx + 0);
    addConstraintTriple(1, constraint_idx + 0, control_idx + 2);

    //-x/mu + z > 0
    addConstraintTriple(-muInv, constraint_idx + 1, control_idx + 0);
    addConstraintTriple(1, constraint_idx + 1, control_idx + 2);

    // y/mu + z > 0
    addConstraintTriple(muInv, constraint_idx + 2, control_idx + 1);
    addConstraintTriple(1, constraint_idx + 2, control_idx + 2);

    //-y/mu + z > 0
    addConstraintTriple(-muInv, constraint_idx + 3, control_idx + 1);
    addConstraintTriple(1, constraint_idx + 3, control_idx + 2);
  }
}

void SparseCMPC::addQuadraticStateCost() {
  // xt Q x
  for(u32 i = 0; i < _trajectoryLength; i++) {
    u32 idx = getStateIndex(i);
    for(u32 j = 0; j < 12; j++) {
      _costTriples.push_back({_weights[j], idx + j, idx + j});
    }
  }
}

void SparseCMPC::addQuadraticControlCost() {
  for(u32 i = 0; i < _bBlockCount; i++) {
    u32 idx = getControlIndex(i);
    for(u32 j = 0; j < 3; j++) {
      _costTriples.push_back({_alpha, idx + j, idx + j});
    }
  }
}

void SparseCMPC::addLinearStateCost() {
  _linearCost.resize(12 * _trajectoryLength + 3 * _bBlockCount);
  for(auto& v : _linearCost) v = 0;
  // -2 * w * x_des
  //printf("z des traj: ");
  for(u32 i = 0; i < _trajectoryLength; i++) {
    u32 idx = getStateIndex(i);
    for(u32 j = 0; j < 12; j++) {
      _linearCost.at(idx + j) = -1. * _stateTrajectory[i][j] * _weights[j];
    }
   // printf("%.3f ", _stateTrajectory[i][5]);
  }
  //printf("\n");

}

void SparseCMPC::runSolver() {
  u32 varCount = 12 * _trajectoryLength + 3 * _bBlockCount;
  //printf("[SparseCMPC] Run %d, %d\n", varCount, _constraintCount);
  assert(_constraintCount == _ub.size());
  assert(_constraintCount == _lb.size());
  assert(varCount == _linearCost.size());


  QpProblem<double> solver(varCount, _constraintCount, false);
  solver.A_triples = std::move(_constraintTriples);
  solver.P_triples = std::move(_costTriples);
  solver.u = Eigen::Matrix<double, Eigen::Dynamic, 1>(_ub.size());
  solver.l = Eigen::Matrix<double, Eigen::Dynamic, 1>(_ub.size());
  for(u32 i = 0; i < _constraintCount; i++) {
    solver.u[i] = _ub[i];
    solver.l[i] = _lb[i];
  }
  solver.q = Eigen::Matrix<double, Eigen::Dynamic, 1>(varCount);
  for(u32 i = 0; i < varCount; i++) {
    solver.q[i] = _linearCost[i];
  }

  solver.settings.alpha = 1.5;
  solver.settings.rho = 1e-3;
  solver.settings.terminate = 1e-6;
  solver.settings.sigma = 1e-6;


  solver.runFromTriples(-1, true);
  _result = solver.getSolution().cast<float>();
}

//static const char* names[] = {"roll", "pitch", "yaw", "x", "y", "z", "roll-rate", "pitch-rate", "yaw-rate", "xv", "yv", "zv"};

Vec12<float> SparseCMPC::getResult() {

//  for(u32 i = 0; i < 12; i++) {
//    printf("%s: ", names[i]);
//    for(u32 j = 0; j < _trajectoryLength; j++) {
//      printf("%6.3f  ", _result[getStateIndex(j) + i]);
//    }
//    printf("\n");
//  }
//
//  printf("zf traj: ");
//  for(u32 i = 0; i < _bBlockCount; i++) {
//    printf("[%2d %3.3f]  ", _bBlockIds[i].timestep, _result[getControlIndex(i) + 2]);
//  }
//  printf("\n");
  Vec12<float> result;
  result.setZero();
  for(u32 i = 0; i < _bBlockIds.size(); i++) {
    auto& id = _bBlockIds[i];
    if(id.timestep == 0) {
      //printf("result for foot %d %.3f %.3f %.3f\n", id.foot, _result[getControlIndex(i) + 0], _result[getControlIndex(i) + 1], _result[getControlIndex(i) + 2]);
      for(u32 j = 0; j < 3; j++) {
        result[id.foot*3 + j] = _result[getControlIndex(i) + j];
      }
    }
  }

  return result;
}

void SparseCMPC::c2d(u32 trajIdx, u32 bBlockStartIdx, u32 block_count) {

  Eigen::Matrix<double, 24, 24> AB, expmm;
  AB.setZero();
  AB.block(0,0,12,12) = _aMat[trajIdx];

  for(u32 i = bBlockStartIdx; i < bBlockStartIdx + block_count; i++) {
    BblockID id = _bBlockIds[i];
    if(id.timestep != trajIdx) throw std::runtime_error("c2d timestep error");
    AB.block(0,12 + 3 * id.foot, 12, 3) = _bBlocks[i];
  }

  AB *= _dtTrajectory[trajIdx];

  expmm = AB.exp();
  _aMat[trajIdx] = expmm.block(0,0,12,12);

  for(u32 i = bBlockStartIdx; i < bBlockStartIdx + block_count; i++) {
    //BblockID id = _bBlockIds[i];
    //_bBlocks[i] = expmm.block(0,12 + 3 * id.foot, 12, 3);
    _bBlocks[i] *= _dtTrajectory[trajIdx];
  }
}

void SparseCMPC::runSolverOSQP() {
  // Timer timer;
  u32 varCount = 12 * _trajectoryLength + 3 * _bBlockCount;
  //printf("[SparseCMPC] Run with OSQP %d, %d\n", varCount, _constraintCount);
  assert(_constraintCount == _ub.size());
  assert(_constraintCount == _lb.size());
  assert(varCount == _linearCost.size());

  // Quadratic term
  sortAndSumTriples(_costTriples);
  OsqpCSC quadraticCostMatrix = compress(_costTriples, varCount, varCount);

  sortAndSumTriples(_constraintTriples);
  OsqpCSC constraintMatrix = compress(_constraintTriples, _constraintCount, varCount);

  OSQPSettings* settings = (OSQPSettings*)malloc(sizeof(OSQPSettings));
  OSQPWorkspace* workspace;
  // (void)workspace;
  OSQPData* data = (OSQPData*)malloc(sizeof(OSQPData));

  data->n = varCount;
  data->m = _constraintCount;
  data->P = csc_matrix(varCount, varCount, quadraticCostMatrix.nnz,
      quadraticCostMatrix.values, quadraticCostMatrix.rowIdx, quadraticCostMatrix.colPtrs);
  data->q = _linearCost.data();
  data->A = csc_matrix(_constraintCount, varCount, constraintMatrix.nnz,
      constraintMatrix.values, constraintMatrix.rowIdx, constraintMatrix.colPtrs);
  data->l = _lb.data();
  data->u = _ub.data();

  //printf("t3: %.3f\n", timer.getMs());
  // timer.start();

//# define EPS_ABS (1E-3)
//# define EPS_REL (1E-3)
//# define EPS_PRIM_INF (1E-4)
//# define EPS_DUAL_INF (1E-4)
  osqp_set_default_settings(settings);
  settings->eps_abs = 1e-5;
  settings->eps_rel = 1e-5;
  //settings->max_iter = 300;
  //settings->alpha = 1.0; //todo try me

  osqp_setup(&workspace, data, settings);

  //printf("t4: %.3f\n", timer.getMs());
  // timer.start();

  osqp_solve(workspace);

  //printf("t5: %.3f\n", timer.getMs());

  _result = Eigen::Matrix<float, Eigen::Dynamic, 1>(varCount);
  for(u32 i = 0; i < varCount; i++) {
    _result[i] = workspace->solution->x[i];
  }

//  osqp_cleanup(workspace);

//  QpProblem<double> solver(varCount, _constraintCount, false);
//  solver.A_triples = std::move(_constraintTriples);
//  solver.P_triples = std::move(_costTriples);
//  solver.u = Vector<double>(_ub.size());
//  solver.l = Vector<double>(_ub.size());
//  for(u32 i = 0; i < _constraintCount; i++) {
//    solver.u[i] = _ub[i];
//    solver.l[i] = _lb[i];
//  }
//  solver.q = Vector<double>(varCount);
//  for(u32 i = 0; i < varCount; i++) {
//    solver.q[i] = _linearCost[i];
//  }
//
//  solver.settings.alpha = 1.5;
//  solver.settings.rho = 1e-3;
//  solver.settings.terminate = 1e-6;
//  solver.settings.sigma = 1e-6;
//
//
//  solver.runFromTriples(-1, true);
//  _result = solver.getSolution().cast<float>();

  quadraticCostMatrix.freeAll();
  constraintMatrix.freeAll();
}

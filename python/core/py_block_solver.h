#pragma once

#include "g2o/config.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2opy.h"

#ifdef G2O_HAVE_CHOLMOD
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#endif
#ifdef G2O_HAVE_CSPARSE
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#endif

namespace g2o {

// helper class
class PyBlockSolverBase {
 public:
  virtual ~PyBlockSolverBase() = default;
  virtual std::unique_ptr<Solver> solver() = 0;
  virtual std::unique_ptr<BlockSolverBase> base_solver() = 0;
};

void declareBlockSolver(py::module& m);

}  // end namespace g2o

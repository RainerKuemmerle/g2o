#include "py_block_solver.h"

#include <pybind11/eigen.h>

#include "g2o/config.h"
#include "g2o/core/solver.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

void declareBlockSolver(py::module& m) {
  py::class_<BlockSolverBase, Solver>(m, "BlockSolverBase");

  py::class_<PyBlockSolverBase>(m, "PyBlockSolverBase");

  typedef BlockSolver_3_2::PoseMatrixType MatrixSE2;
  typedef BlockSolver_6_3::PoseMatrixType MatrixSE3;
  typedef BlockSolver_7_3::PoseMatrixType MatrixSim3;
  typedef BlockSolverX::PoseMatrixType MatrixX;

#if G2O_HAVE_CHOLMOD
  templatedPyLinearSolver<g2o::LinearSolverCholmod<MatrixSE2>, BlockSolver_3_2>(m, "CholmodSE2");
  templatedPyLinearSolver<g2o::LinearSolverCholmod<MatrixSE3>, BlockSolver_6_3>(m, "CholmodSE3");
  templatedPyLinearSolver<g2o::LinearSolverCholmod<MatrixSim3>, BlockSolver_7_3>(m, "CholmodSim3");
  templatedPyLinearSolver<g2o::LinearSolverCholmod<MatrixX>, BlockSolverX>(m, "CholmodX");
#endif

#if G2O_HAVE_CSPARSE
  templatedPyLinearSolver<g2o::LinearSolverCSparse<MatrixSE2>, BlockSolver_3_2>(m, "CSparseSE2");
  templatedPyLinearSolver<g2o::LinearSolverCSparse<MatrixSE3>, BlockSolver_6_3>(m, "CSparseSE3");
  templatedPyLinearSolver<g2o::LinearSolverCSparse<MatrixSim3>, BlockSolver_7_3>(m, "CSparseSim3");
  templatedPyLinearSolver<g2o::LinearSolverCSparse<MatrixX>, BlockSolverX>(m, "CSparseX");
#endif

  templatedPyLinearSolver<g2o::LinearSolverEigen<MatrixSE2>, BlockSolver_3_2>(m, "EigenSE2");
  templatedPyLinearSolver<g2o::LinearSolverEigen<MatrixSE3>, BlockSolver_6_3>(m, "EigenSE3");
  templatedPyLinearSolver<g2o::LinearSolverEigen<MatrixSim3>, BlockSolver_7_3>(m, "EigenSim3");
  templatedPyLinearSolver<g2o::LinearSolverEigen<MatrixX>, BlockSolverX>(m, "EigenX");

  templatedPyLinearSolver_<g2o::LinearSolverDense<MatrixSE2>, BlockSolver_3_2>(m, "DenseSE2");
  templatedPyLinearSolver_<g2o::LinearSolverDense<MatrixSE3>, BlockSolver_6_3>(m, "DenseSE3");
  templatedPyLinearSolver_<g2o::LinearSolverDense<MatrixSim3>, BlockSolver_7_3>(m, "DenseSim3");
  templatedPyLinearSolver_<g2o::LinearSolverDense<MatrixX>, BlockSolverX>(m, "DenseX");

  templatedPyLinearSolver_<g2o::LinearSolverPCG<MatrixSE2>, BlockSolver_3_2>(m, "PCGSE2");
  templatedPyLinearSolver_<g2o::LinearSolverPCG<MatrixSE3>, BlockSolver_6_3>(m, "PCGSE3");
  templatedPyLinearSolver_<g2o::LinearSolverPCG<MatrixSim3>, BlockSolver_7_3>(m, "PCGSim3");
  templatedPyLinearSolver_<g2o::LinearSolverPCG<MatrixX>, BlockSolverX>(m, "PCGX");

  templatedPyBlockSolver<BlockSolver_3_2>(m, "SE2");
  templatedPyBlockSolver<BlockSolver_6_3>(m, "SE3");
  templatedPyBlockSolver<BlockSolver_7_3>(m, "Sim3");
  templatedPyBlockSolver<BlockSolverX>(m, "X");
}

}  // end namespace g2o

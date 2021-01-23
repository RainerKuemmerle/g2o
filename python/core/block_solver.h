#pragma once

#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#define _CHOLMOD_FOUND 1
#define _CSPARSE_FOUND 1
#define _EIGEN_FOUND 1

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

// helper class
class PyBlockSolverBase {
 public:
  std::unique_ptr<Solver> block_solver;
  std::unique_ptr<BlockSolverBase> block_base_solver;

  virtual std::unique_ptr<Solver> solver() { return std::move(block_solver); };

  virtual std::unique_ptr<BlockSolverBase> base_solver() { return std::move(block_base_solver); };
};

namespace {

template <typename LinearSolverT, typename BlockSolverT>
class PyLinearSolver {
 public:
  // std::unique_ptr<typename BlockSolverT::LinearSolverType> solver;
  std::unique_ptr<LinearSolverT> solver;

  PyLinearSolver() : solver(g2o::make_unique<LinearSolverT>()){};
};

template <typename LinearSolverT, typename BlockSolverT>
void templatedPyLinearSolver(py::module& m, const std::string& suffix) {
  using CLS = PyLinearSolver<LinearSolverT, BlockSolverT>;

  py::class_<CLS>(m, ("LinearSolver" + suffix).c_str())
      .def(py::init<>())
      .def("set_block_ordering",
           [](CLS& ls, bool blockOrdering) { ls.solver->setBlockOrdering(blockOrdering); });
}

template <typename LinearSolverT, typename BlockSolverT>
void templatedPyLinearSolver_(py::module& m, const std::string& suffix) {
  using CLS = PyLinearSolver<LinearSolverT, BlockSolverT>;

  py::class_<CLS>(m, ("LinearSolver" + suffix).c_str()).def(py::init<>());
}

template <typename BlockSolverT>
class PyBlockSolver : public PyBlockSolverBase {
 public:
  std::unique_ptr<BlockSolverT> block_solver;

#if _CHOLMOD_FOUND
  PyBlockSolver(PyLinearSolver<g2o::LinearSolverCholmod<typename BlockSolverT::PoseMatrixType>,
                               BlockSolverT>& linearSolver)
      : block_solver(g2o::make_unique<BlockSolverT>(
            (std::unique_ptr<typename BlockSolverT::LinearSolverType>)std::move(
                linearSolver.solver))){};
#endif

#if _CSPARSE_FOUND
  PyBlockSolver(PyLinearSolver<g2o::LinearSolverCSparse<typename BlockSolverT::PoseMatrixType>,
                               BlockSolverT>& linearSolver)
      : block_solver(g2o::make_unique<BlockSolverT>(
            (std::unique_ptr<typename BlockSolverT::LinearSolverType>)std::move(
                linearSolver.solver))){};
#endif

#if _EIGEN_FOUND
  PyBlockSolver(PyLinearSolver<g2o::LinearSolverEigen<typename BlockSolverT::PoseMatrixType>,
                               BlockSolverT>& linearSolver)
      : block_solver(g2o::make_unique<BlockSolverT>(
            (std::unique_ptr<typename BlockSolverT::LinearSolverType>)std::move(
                linearSolver.solver))){};
#endif

  PyBlockSolver(PyLinearSolver<g2o::LinearSolverDense<typename BlockSolverT::PoseMatrixType>,
                               BlockSolverT>& linearSolver)
      : block_solver(g2o::make_unique<BlockSolverT>(
            (std::unique_ptr<typename BlockSolverT::LinearSolverType>)std::move(
                linearSolver.solver))){};

  PyBlockSolver(PyLinearSolver<g2o::LinearSolverPCG<typename BlockSolverT::PoseMatrixType>,
                               BlockSolverT>& linearSolver)
      : block_solver(g2o::make_unique<BlockSolverT>(
            (std::unique_ptr<typename BlockSolverT::LinearSolverType>)std::move(
                linearSolver.solver))){};

  std::unique_ptr<Solver> solver() { return std::move(block_solver); };
  std::unique_ptr<BlockSolverBase> base_solver() { return std::move(block_solver); };
};

template <typename BlockSolverT>
void templatedPyBlockSolver(py::module& m, const std::string& suffix) {
  using CLS = PyBlockSolver<BlockSolverT>;

  py::class_<CLS, PyBlockSolverBase>(m, ("BlockSolver" + suffix).c_str())
#if _CHOLMOD_FOUND
      .def(py::init<PyLinearSolver<g2o::LinearSolverCholmod<typename BlockSolverT::PoseMatrixType>,
                                   BlockSolverT>&>())
#endif
#if _CSPARSE_FOUND
      .def(py::init<PyLinearSolver<g2o::LinearSolverCSparse<typename BlockSolverT::PoseMatrixType>,
                                   BlockSolverT>&>())
#endif
#if _EIGEN_FOUND
      .def(py::init<PyLinearSolver<g2o::LinearSolverEigen<typename BlockSolverT::PoseMatrixType>,
                                   BlockSolverT>&>())
#endif
      .def(py::init<PyLinearSolver<g2o::LinearSolverDense<typename BlockSolverT::PoseMatrixType>,
                                   BlockSolverT>&>())
      .def(py::init<PyLinearSolver<g2o::LinearSolverPCG<typename BlockSolverT::PoseMatrixType>,
                                   BlockSolverT>&>());
}

}  // namespace

void declareBlockSolver(py::module& m) {
  py::class_<BlockSolverBase, Solver>(m, "BlockSolverBase");

  py::class_<PyBlockSolverBase>(m, "PyBlockSolverBase");

  typedef BlockSolver_3_2::PoseMatrixType MatrixSE2;
  typedef BlockSolver_6_3::PoseMatrixType MatrixSE3;
  typedef BlockSolver_7_3::PoseMatrixType MatrixSim3;
  typedef BlockSolverX::PoseMatrixType MatrixX;

#if _CHOLMOD_FOUND
  templatedPyLinearSolver<g2o::LinearSolverCholmod<MatrixSE2>, BlockSolver_3_2>(m, "CholmodSE2");
  templatedPyLinearSolver<g2o::LinearSolverCholmod<MatrixSE3>, BlockSolver_6_3>(m, "CholmodSE3");
  templatedPyLinearSolver<g2o::LinearSolverCholmod<MatrixSim3>, BlockSolver_7_3>(m, "CholmodSim3");
  templatedPyLinearSolver<g2o::LinearSolverCholmod<MatrixX>, BlockSolverX>(m, "CholmodX");
#endif

#if _CSPARSE_FOUND
  templatedPyLinearSolver<g2o::LinearSolverCSparse<MatrixSE2>, BlockSolver_3_2>(m, "CSparseSE2");
  templatedPyLinearSolver<g2o::LinearSolverCSparse<MatrixSE3>, BlockSolver_6_3>(m, "CSparseSE3");
  templatedPyLinearSolver<g2o::LinearSolverCSparse<MatrixSim3>, BlockSolver_7_3>(m, "CSparseSim3");
  templatedPyLinearSolver<g2o::LinearSolverCSparse<MatrixX>, BlockSolverX>(m, "CSparseX");
#endif

#if _EIGEN_FOUND
  templatedPyLinearSolver<g2o::LinearSolverEigen<MatrixSE2>, BlockSolver_3_2>(m, "EigenSE2");
  templatedPyLinearSolver<g2o::LinearSolverEigen<MatrixSE3>, BlockSolver_6_3>(m, "EigenSE3");
  templatedPyLinearSolver<g2o::LinearSolverEigen<MatrixSim3>, BlockSolver_7_3>(m, "EigenSim3");
  templatedPyLinearSolver<g2o::LinearSolverEigen<MatrixX>, BlockSolverX>(m, "EigenX");
#endif

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

#undef _CHOLMOD_FOUND
#undef _CSPARSE_FOUND
#undef _EIGEN_FOUND
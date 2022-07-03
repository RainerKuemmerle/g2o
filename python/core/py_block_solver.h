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
      .def("set_block_ordering", [](CLS& ls, bool blockOrdering) {
        ls.solver->setBlockOrdering(blockOrdering);
      });
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

#if G2O_HAVE_CHOLMOD
  PyBlockSolver(PyLinearSolver<
                g2o::LinearSolverCholmod<typename BlockSolverT::PoseMatrixType>,
                BlockSolverT>& linearSolver)
      : block_solver(g2o::make_unique<BlockSolverT>(
            (std::unique_ptr<typename BlockSolverT::LinearSolverType>)std::move(
                linearSolver.solver))){};
#endif

#if G2O_HAVE_CSPARSE
  PyBlockSolver(PyLinearSolver<
                g2o::LinearSolverCSparse<typename BlockSolverT::PoseMatrixType>,
                BlockSolverT>& linearSolver)
      : block_solver(g2o::make_unique<BlockSolverT>(
            (std::unique_ptr<typename BlockSolverT::LinearSolverType>)std::move(
                linearSolver.solver))){};
#endif

  PyBlockSolver(PyLinearSolver<
                g2o::LinearSolverEigen<typename BlockSolverT::PoseMatrixType>,
                BlockSolverT>& linearSolver)
      : block_solver(g2o::make_unique<BlockSolverT>(
            (std::unique_ptr<typename BlockSolverT::LinearSolverType>)std::move(
                linearSolver.solver))){};

  PyBlockSolver(PyLinearSolver<
                g2o::LinearSolverDense<typename BlockSolverT::PoseMatrixType>,
                BlockSolverT>& linearSolver)
      : block_solver(g2o::make_unique<BlockSolverT>(
            (std::unique_ptr<typename BlockSolverT::LinearSolverType>)std::move(
                linearSolver.solver))){};

  PyBlockSolver(PyLinearSolver<
                g2o::LinearSolverPCG<typename BlockSolverT::PoseMatrixType>,
                BlockSolverT>& linearSolver)
      : block_solver(g2o::make_unique<BlockSolverT>(
            (std::unique_ptr<typename BlockSolverT::LinearSolverType>)std::move(
                linearSolver.solver))){};

  std::unique_ptr<Solver> solver() { return std::move(block_solver); };
  std::unique_ptr<BlockSolverBase> base_solver() {
    return std::move(block_solver);
  };
};

template <typename BlockSolverT>
void templatedPyBlockSolver(py::module& m, const std::string& suffix) {
  using CLS = PyBlockSolver<BlockSolverT>;

  py::class_<CLS, PyBlockSolverBase>(m, ("BlockSolver" + suffix).c_str())
#if G2O_HAVE_CHOLMOD
      .def(py::init<PyLinearSolver<
               g2o::LinearSolverCholmod<typename BlockSolverT::PoseMatrixType>,
               BlockSolverT>&>())
#endif
#if G2O_HAVE_CSPARSE
      .def(py::init<PyLinearSolver<
               g2o::LinearSolverCSparse<typename BlockSolverT::PoseMatrixType>,
               BlockSolverT>&>())
#endif
      .def(py::init<PyLinearSolver<
               g2o::LinearSolverEigen<typename BlockSolverT::PoseMatrixType>,
               BlockSolverT>&>())
      .def(py::init<PyLinearSolver<
               g2o::LinearSolverDense<typename BlockSolverT::PoseMatrixType>,
               BlockSolverT>&>())
      .def(py::init<PyLinearSolver<
               g2o::LinearSolverPCG<typename BlockSolverT::PoseMatrixType>,
               BlockSolverT>&>());
}

}  // namespace

void declareBlockSolver(py::module& m);

}  // end namespace g2o

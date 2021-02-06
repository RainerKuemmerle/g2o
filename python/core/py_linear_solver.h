#pragma once
#include <g2o/core/linear_solver.h>

#include "g2opy.h"

namespace g2o {

template <typename MatrixType>
void templatedLinearSolver(py::module& m, const std::string& suffix) {
  py::class_<LinearSolver<MatrixType>>(m, ("LinearSolver" + suffix).c_str())
      //.def("solve_blocks", &LinearSolver<MatrixType>::solveBlocks)
      //.def("solve_pattern", &LinearSolver<MatrixType>::solvePattern)
      //.def("write_debug", &LinearSolver<MatrixType>::writeDebug)
      //.def("set_write_debug", &LinearSolver<MatrixType>::setWriteDebug)
      ;
}

template <typename MatrixType>
void templatedLinearSolverCCS(py::module& m, const std::string& suffix) {
  templatedLinearSolver<MatrixType>(m, suffix);

  py::class_<LinearSolverCCS<MatrixType>, LinearSolver<MatrixType>>(
      m, ("LinearSolverCCS" + suffix).c_str())
      //.def(py::init<>())
      ;
}

}  // namespace g2o

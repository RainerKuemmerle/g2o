#include "py_optimization_algorithm.h"

#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_property.h"
#include "g2o/core/optimization_algorithm_with_hessian.h"
#include "py_block_solver.h"

namespace g2o {

void declareOptimizationAlgorithm(py::module_& m) {
  py::class_<OptimizationAlgorithm>(m, "OptimizationAlgorithm");  // NOLINT

  py::class_<OptimizationAlgorithmProperty>(  // NOLINT
      m, "OptimizationAlgorithmProperty");

  py::class_<OptimizationAlgorithmWithHessian, OptimizationAlgorithm>(
      m, "OptimizationAlgorithmWithHessian");

  py::class_<OptimizationAlgorithmGaussNewton,
             OptimizationAlgorithmWithHessian>(
      m, "OptimizationAlgorithmGaussNewton")
      .def(py::new_([](PyBlockSolverBase& blockSolver) {
        auto solver = blockSolver.solver();
        return new OptimizationAlgorithmGaussNewton(std::move(solver));
      }));

  py::class_<OptimizationAlgorithmLevenberg, OptimizationAlgorithmWithHessian>(
      m, "OptimizationAlgorithmLevenberg")
      .def(py::new_([](PyBlockSolverBase& blockSolver) {
        auto solver = blockSolver.solver();
        return new OptimizationAlgorithmLevenberg(std::move(solver));
      }));

  py::class_<OptimizationAlgorithmDogleg, OptimizationAlgorithmWithHessian>(
      m, "OptimizationAlgorithmDogleg")
      .def(py::new_([](PyBlockSolverBase& blockSolver) {
        auto solver = blockSolver.base_solver();
        return new OptimizationAlgorithmDogleg(std::move(solver));
      }));

  py::class_<AbstractOptimizationAlgorithmCreator>(  // NOLINT
      m, "AbstractOptimizationAlgorithmCreator");

  py::class_<RegisterOptimizationAlgorithmProxy>(
      m, "RegisterOptimizationAlgorithmProxy");
}

}  // end namespace g2o

#include "py_block_solver.h"

#include <memory>

#include "g2o/config.h"
#include "g2o/core/solver.h"

namespace g2o {
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
void templatedPyLinearSolverWithoutOrdering(py::module& m,
                                            const std::string& suffix) {
  using CLS = PyLinearSolver<LinearSolverT, BlockSolverT>;

  py::class_<CLS>(m, ("LinearSolver" + suffix).c_str()).def(py::init<>());
}

template <typename BlockSolverT>
class PyBlockSolver : public PyBlockSolverBase {
 public:
  std::unique_ptr<BlockSolverT> block_solver;

  using PoseMatrixType = typename BlockSolverT::PoseMatrixType;

#if G2O_HAVE_CHOLMOD
  explicit PyBlockSolver(
      PyLinearSolver<g2o::LinearSolverCholmod<PoseMatrixType>, BlockSolverT>&
          linearSolver)
      : block_solver(
            g2o::make_unique<BlockSolverT>(std::move(linearSolver.solver))){};
#endif

#if G2O_HAVE_CSPARSE
  explicit PyBlockSolver(
      PyLinearSolver<g2o::LinearSolverCSparse<PoseMatrixType>, BlockSolverT>&
          linearSolver)
      : block_solver(
            g2o::make_unique<BlockSolverT>(std::move(linearSolver.solver))){};
#endif

  explicit PyBlockSolver(PyLinearSolver<g2o::LinearSolverEigen<PoseMatrixType>,
                                        BlockSolverT>& linearSolver)
      : block_solver(
            g2o::make_unique<BlockSolverT>(std::move(linearSolver.solver))){};

  explicit PyBlockSolver(PyLinearSolver<g2o::LinearSolverDense<PoseMatrixType>,
                                        BlockSolverT>& linearSolver)
      : block_solver(
            g2o::make_unique<BlockSolverT>(std::move(linearSolver.solver))){};

  explicit PyBlockSolver(PyLinearSolver<g2o::LinearSolverPCG<PoseMatrixType>,
                                        BlockSolverT>& linearSolver)
      : block_solver(
            g2o::make_unique<BlockSolverT>(std::move(linearSolver.solver))){};

  std::unique_ptr<Solver> solver() override { return std::move(block_solver); };
  std::unique_ptr<BlockSolverBase> base_solver() override {
    return std::move(block_solver);
  };
};

template <typename BlockSolverT>
void templatedPyBlockSolver(py::module& m, const std::string& suffix) {
  using CLS = PyBlockSolver<BlockSolverT>;
  using PoseMatrixType = typename CLS::PoseMatrixType;

  py::class_<CLS, PyBlockSolverBase>(m, ("BlockSolver" + suffix).c_str())
#if G2O_HAVE_CHOLMOD
      .def(py::init<PyLinearSolver<g2o::LinearSolverCholmod<PoseMatrixType>,
                                   BlockSolverT>&>())
#endif
#if G2O_HAVE_CSPARSE
      .def(py::init<PyLinearSolver<g2o::LinearSolverCSparse<PoseMatrixType>,
                                   BlockSolverT>&>())
#endif
      .def(py::init<PyLinearSolver<g2o::LinearSolverEigen<PoseMatrixType>,
                                   BlockSolverT>&>())
      .def(py::init<PyLinearSolver<g2o::LinearSolverDense<PoseMatrixType>,
                                   BlockSolverT>&>())
      .def(py::init<PyLinearSolver<g2o::LinearSolverPCG<PoseMatrixType>,
                                   BlockSolverT>&>());
}

}  // namespace

void declareBlockSolver(py::module& m) {
  py::class_<BlockSolverBase, Solver>(m, "BlockSolverBase");  // NOLINT

  py::class_<PyBlockSolverBase>(m, "PyBlockSolverBase");  // NOLINT

  using MatrixSE2 = BlockSolver_3_2::PoseMatrixType;
  using MatrixSE3 = BlockSolver_6_3::PoseMatrixType;
  using MatrixSim3 = BlockSolver_7_3::PoseMatrixType;
  using MatrixX = BlockSolverX::PoseMatrixType;

#if G2O_HAVE_CHOLMOD
  templatedPyLinearSolver<g2o::LinearSolverCholmod<MatrixSE2>, BlockSolver_3_2>(
      m, "CholmodSE2");
  templatedPyLinearSolver<g2o::LinearSolverCholmod<MatrixSE3>, BlockSolver_6_3>(
      m, "CholmodSE3");
  templatedPyLinearSolver<g2o::LinearSolverCholmod<MatrixSim3>,
                          BlockSolver_7_3>(m, "CholmodSim3");
  templatedPyLinearSolver<g2o::LinearSolverCholmod<MatrixX>, BlockSolverX>(
      m, "CholmodX");
#endif

#if G2O_HAVE_CSPARSE
  templatedPyLinearSolver<g2o::LinearSolverCSparse<MatrixSE2>, BlockSolver_3_2>(
      m, "CSparseSE2");
  templatedPyLinearSolver<g2o::LinearSolverCSparse<MatrixSE3>, BlockSolver_6_3>(
      m, "CSparseSE3");
  templatedPyLinearSolver<g2o::LinearSolverCSparse<MatrixSim3>,
                          BlockSolver_7_3>(m, "CSparseSim3");
  templatedPyLinearSolver<g2o::LinearSolverCSparse<MatrixX>, BlockSolverX>(
      m, "CSparseX");
#endif

  templatedPyLinearSolver<g2o::LinearSolverEigen<MatrixSE2>, BlockSolver_3_2>(
      m, "EigenSE2");
  templatedPyLinearSolver<g2o::LinearSolverEigen<MatrixSE3>, BlockSolver_6_3>(
      m, "EigenSE3");
  templatedPyLinearSolver<g2o::LinearSolverEigen<MatrixSim3>, BlockSolver_7_3>(
      m, "EigenSim3");
  templatedPyLinearSolver<g2o::LinearSolverEigen<MatrixX>, BlockSolverX>(
      m, "EigenX");

  templatedPyLinearSolverWithoutOrdering<g2o::LinearSolverDense<MatrixSE2>,
                                         BlockSolver_3_2>(m, "DenseSE2");
  templatedPyLinearSolverWithoutOrdering<g2o::LinearSolverDense<MatrixSE3>,
                                         BlockSolver_6_3>(m, "DenseSE3");
  templatedPyLinearSolverWithoutOrdering<g2o::LinearSolverDense<MatrixSim3>,
                                         BlockSolver_7_3>(m, "DenseSim3");
  templatedPyLinearSolverWithoutOrdering<g2o::LinearSolverDense<MatrixX>,
                                         BlockSolverX>(m, "DenseX");

  templatedPyLinearSolverWithoutOrdering<g2o::LinearSolverPCG<MatrixSE2>,
                                         BlockSolver_3_2>(m, "PCGSE2");
  templatedPyLinearSolverWithoutOrdering<g2o::LinearSolverPCG<MatrixSE3>,
                                         BlockSolver_6_3>(m, "PCGSE3");
  templatedPyLinearSolverWithoutOrdering<g2o::LinearSolverPCG<MatrixSim3>,
                                         BlockSolver_7_3>(m, "PCGSim3");
  templatedPyLinearSolverWithoutOrdering<g2o::LinearSolverPCG<MatrixX>,
                                         BlockSolverX>(m, "PCGX");

  templatedPyBlockSolver<BlockSolver_3_2>(m, "SE2");
  templatedPyBlockSolver<BlockSolver_6_3>(m, "SE3");
  templatedPyBlockSolver<BlockSolver_7_3>(m, "Sim3");
  templatedPyBlockSolver<BlockSolverX>(m, "X");
}

}  // end namespace g2o
